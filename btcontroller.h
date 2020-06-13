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
  _close       = 27
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
  // printf("%d -> ALTERAR Kp\n", OPTION::_send_kp);
  printf("T -> AUTO TUNNING\n");
  printf("I -> IDENTIFICAÇÃO\n");
  printf("V -> VISUALIZAR GRAFICOS\n");
  printf("M -> PEDIR DADOS DA CALIBRACAO\n");
  printf("************************************\n");
  printf("ESQ -> ENCERRAR O PROGRAMA\n");
};

void _pause(const char* msg = ""){
  printf("%s\n(PRESSIONE QUALQUER TECLA PARA CONTINUAR)\n", msg);
  cin.get();
  cin.ignore(1,'\n');
}

void _printListMACs(){
  printf("****************MACs Conhecidos*************\n");
  vector<string> addrs = btAction.getDest();
  for(int i = 0; i < (int)addrs.size(); i++)
  {
    printf("%d -> Robo_%d = %s\n",i, i, addrs[i].c_str());
  }
};
