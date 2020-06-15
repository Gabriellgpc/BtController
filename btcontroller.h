#include "bluetoothAction/bluetoothAction.h"

#define MAC_ESP_TEST   "30:AE:A4:3B:A4:26"
#define MAC_ESP_ROBO_1 "30:AE:A4:20:0E:12"
#define MAC_ESP_ROBO_2 "30:AE:A4:13:F8:AE"
#define MAC_ESP_ROBO_3 "30:AE:A4:20:0E:12"
#define MAC_ESP_ROBO_4 "30:AE:A4:41:74:F6"

#define F_IS_NEG(x) (*(uint32_t*)&(x) >> 31)
#define ABS_F(x) (((x)<0.0)?-(x):(x))

//Comandos do bluetooth
#define CMD_HEAD           0xA0

#define CMD_REQ_CAL        0x00
#define CMD_REQ_OMEGA      0x03

#define CMD_CALIBRATION    0x04
#define CMD_IDENTIFY       0x05

#define CMD_REF            0x0A
#define CMD_CONTROL_SIGNAL 0x0B

#define CMD_RESET          0x0E
#define CMD_PING           0x0F

#define RADIUS    1.5/100 //metros
#define REDUCTION 30 //30x1

using namespace std;
BluetoothAction btAction;

typedef struct
{
  double  rawOmega;  //ultimo omega medido, sem filtro
  double  omega;     //omega filtrado
  double  pOmega;    //predicted omega
  double  kGain;     //kalman gain
  double  p;      //preditec variance
}encoder_data_t;

typedef struct
{
  double ang; //coef. angular da reta
  double lin;  //coef. linear da reta
}coefLine_t;

typedef struct
{
  double omegaMax;
  double Kp[4];        //ganho do controlador
  double K[2];         //ganho do sistema, motor esquerdo e direito
  double tau[2];       //constante de tempo, motor esquerdo e direito
  coefLine_t coef[4];
}parameters_t;

string getDate()
{
  time_t rawtime;
  tm *timeinfo;
  time(&rawtime);
  timeinfo = localtime(&rawtime);
  std::string strTime = asctime(timeinfo);
  strTime.replace(strTime.find(' '), 1, "_");
  strTime.replace(strTime.find(' '), 1, "_");
  strTime.replace(strTime.find(' '), 1, "_");
  strTime.replace(strTime.find(' '), 1, "_");
  return strTime;
}

void saveToFile(encoder_data_t vec_data[], const int size, const float timeout, const double omegaRef, const string &fileName);

void bytes2float(const uint8_t *bitstream, float*f, uint32_t num_float)
{
  memcpy((float*)bitstream, f, num_float*sizeof(float));
}

void float2bytes(const float*f, uint8_t *bitstream, uint32_t num_float)
{
  memcpy((uint8_t*)f, bitstream, num_float*sizeof(float));
}

void encodeFloat(const float* vec_f, uint8_t *bitstream)
{
  uint16_t ref_b[2];

	ref_b[0] = (!F_IS_NEG(vec_f[0]) << 15)  | (uint16_t)(ABS_F(vec_f[0])*32767.0);
  ref_b[1]= (!F_IS_NEG(vec_f[1]) << 15)   | (uint16_t)(ABS_F(vec_f[1])*32767.0);

  bitstream[0] = (ref_b[0] & 0xFF00) >> 8;
  bitstream[1] = (ref_b[0] & 0x00FF);
  bitstream[2] = (ref_b[1] & 0xFF00) >> 8;
  bitstream[3] = (ref_b[1] & 0x00FF);
}

enum OPTION{
  _rec_omegas  = 'O',
  _connect     = 'C',
  _disconnect  = 'D',
  _ping        = 'P',
  _send_ref    = 'R',
  _send_pwm    = 'S',
  _calibration = 'T',
  _identify    = 'I',
  _graphic     = 'V',
  _rec_coef    = 'M',
  _reset       = '1',
  _close       = 'Q'
};

void _printMainMenu()
{
  printf("****************MENU DE AÇÕES*************\n");
  printf("O -> PEDIR OMEGAS ATUAIS\n");
  printf("************************************\n");
  printf("C -> CONECTAR\n");
  printf("D -> DESCONECTAR\n");
  printf("P -> PING\n");
  printf("************************************\n");
  printf("R -> ENVIAR REFERÊNCIAS\n");
  printf("S -> ENVIAR PWM\n");
  printf("************************************\n");
  printf("T -> AUTO TUNNING\n");
  printf("I -> IDENTIFICAÇÃO\n");
  printf("V -> VISUALIZAR GRAFICOS\n");
  printf("M -> PEDIR DADOS DA CALIBRACAO\n");
  printf("************************************\n");
  printf("1 -> RESETAR O MICROCONTROLADOR\n");
  printf("Q -> ENCERRAR O PROGRAMA\n");
};

void _printParameters(const parameters_t& parameters)
{
  printf("Omega Max: %f rad/s = %f m/s\n", parameters.omegaMax, parameters.omegaMax*RADIUS/(REDUCTION));//reducao de 30 e 24 interrupcoes por volta
  printf("Left  Front  => a = %f , b = %f \n", parameters.coef[0].ang, parameters.coef[0].lin);
  printf("Left  Back   => a = %f , b = %f \n", parameters.coef[1].ang, parameters.coef[1].lin);
  printf("Right Front  => a = %f , b = %f \n", parameters.coef[2].ang, parameters.coef[2].lin);
  printf("Right Back   => a = %f , b = %f \n", parameters.coef[3].ang, parameters.coef[3].lin);
  printf("Left  Kp_frente = %.12f, Kp_tras %.12f\n", parameters.Kp[0], parameters.Kp[1]);
  printf("Right Kp_frente = %.12f, Kp_tras %.12f\n", parameters.Kp[2], parameters.Kp[3]);
  printf("Left  K = %.12f\n", parameters.K[0]);
  printf("Right K = %.12f\n", parameters.K[1]);
  printf("Left  tau = %.12f\n", parameters.tau[0]);
  printf("Right tau = %.12f\n", parameters.tau[1]);
}

void _pause(const char* msg = ""){
  printf("%s\n(PRESSIONE QUALQUER TECLA PARA CONTINUAR)\n", msg);
  cin.get();
}

void _printListMACs(){
  printf("****************MACs Conhecidos*************\n");
  vector<string> addrs = btAction.getDest();
  for(int i = 0; i < (int)addrs.size(); i++)
  {
    printf("%d -> Robo_%d = %s\n",i, i, addrs[i].c_str());
  }
};

// void _funcIdentify(const int& idBt )
// {
//   uint8_t *bitstream;
//   double time_stemp[2];
//   int rec, send, choice;
//
//   encoder_data_t *vec_data = NULL;
//   double omegaRef;
//   uint8_t motor, typeC;
//   float setpoint;
//   int size, sumRec = 0;
//   char motorChoice;
//   do{
//     cout << "Motor ? (L -> Left \t R -> Right): ";
//     motorChoice = cin.get();
//     motorChoice = toupper(motorChoice);
//   }while(motorChoice != 'L' && motorChoice != 'R');
//
//   cout << "Setpoint: ";
//   cin >> setpoint;
//
//   cout << "Desabilitar controlador ? 0(nao), 1(sim): ";
//   cin >> typeC;
//
//   bitstream  = new uint8_t[2 + 1*sizeof(float)];
//
//   bitstream[0]                   = CMD_HEAD | CMD_IDENTIFY;
//   bitstream[1]                   = ((motor << 7)  | typeC) & 0b10000001;
//   *(float*)&bitstream[2 + 0*sizeof(float)] = setpoint;
//
//   //envia comando para iniciar a identificação
//   btAction.sendBluetoothMessage(idBt, bitstream, 2 + 1*sizeof(float));
//   time_stemp[0] = omp_get_wtime();
//
//   printf("Esperando...\n");
//   //aguarda receber a informação da quantidade de medições
//   rec = btAction.recvBluetoothMessage(idBt, (uint8_t*)&size, sizeof(int), 10);
//   if(rec == -1)
//   {
//     _pause("timeout! erro ao receber informacao do size");
//     return;
//   }
//   else printf("Medicoes:%d\n", size);
//   sumRec += rec;
//   vec_data = new encoder_data_t[size];
//   memset(vec_data, 0, size*sizeof(encoder_data_t));
//   //aguarda receber a informação da quantidade de medições
//   rec = btAction.recvBluetoothMessage(idBt, (uint8_t*)&omegaRef, sizeof(double), 10);
//   if(rec == -1)
//   {
//     _pause("timeout! erro ao receber informacao do omega de referencia");
//     return;
//   }
//   else printf("Omega ref:%lf\n", omegaRef);
//   sumRec += rec;
//
//   int step = 20;
//   for(int i = 0; i < size; i += step)
//   {
//     rec = btAction.recvBluetoothMessage(idBt, (uint8_t*)&vec_data[i], step*sizeof(encoder_data_t), 20);
//     printf("Recebi:%d bytes\n", rec);
//     if(rec == -1)
//       puts("timeout!");
//     sumRec += rec;
//   }
//   time_stemp[1] = omp_get_wtime();
//   printf("Tempo decorrido: %f s\n", time_stemp[1] - time_stemp[0]);
//
//   printf("%d Bytes recebidos, deseja salvar ? (0/1)", sumRec);
//   cin >> choice;
//
//   if(choice == 1)
//   {
//     char fileName[50];
//     cout << "Que nome devo colocar no arquivo ? ";
//     scanf("%50s", fileName);
//
//     saveToFile(vec_data, size, 2.0, omegaRef, string(fileName));
//     cout << "Salvando...\n";
//     printf("Salvo! Em: %s\n", (string("etc/") + string(fileName)).c_str());
//     _pause();
//   }else{
//     printf("Tudo bem então...\n");
//     _pause();
//   }
//
//   delete[] bitstream;
//   delete[] vec_data;
// }
