#include <locale>         // std::locale, std::toupper
#include <limits>         // std::numeric_limits
#include <string>         //string
#include <fstream>        //ofstream
#include <iostream>          //cout, cerr, cin
#include <cstdio>            //printf, uintx_t, intx_t...
#include <unistd.h>          //sleep
#include <cmath>        //fabs, pi
#include <omp.h>        //omp_get_wtime

// #include <stdlib.h>     /* system, NULL, EXIT_FAILURE */

#include "btcontroller.hpp"

using namespace std;

void func_manager(void*X)
{
  ((BtRemoteCtrl*)X)->_manager();
}

BtRemoteCtrl::BtRemoteCtrl():
tr_manager(),
bitstream(NULL),
running(false),
idBt(-1)
{

}

BtRemoteCtrl::~BtRemoteCtrl()
{
  this->stop();
}

void BtRemoteCtrl::start()
{
  if(this->running == false)
  {
    this->running = true;
    tr_manager = std::thread(func_manager, (void*)this);
    tr_manager.join();
  }
}

void BtRemoteCtrl::stop()
{
  // desligar bluetooth
  if(idBt != -1)closeBluetoothById(idBt);
  // encerrar thread
  this->running = false;
}

void BtRemoteCtrl::_manager()
{
  int choice;
  while(this->running)
  {
    choice = _mainMenu();
    if(idBt == -1 && (choice != OPTION::_graphic &&
                      choice != OPTION::_connect &&
                      choice != OPTION::_reset &&
                      choice != OPTION::_close))
    {
          _pause("Nenhum dispositivo conectado!");
          continue;
    }

    switch (choice)
    {
      case OPTION::_rec_omegas://solicitar omegas
        _reqOmegas();
      break;
      case OPTION::_connect://conectar
        _BtConnect();
      break;
      case OPTION::_disconnect://desconectar
        _BtDisconnect();
      break;
      case OPTION::_ping: //_ping
        _reqPing();
      break;
      case OPTION::_send_ref://omegas ref
        _sendRef();
      break;
      case OPTION::_send_pwm://omegas sinal de controle
        _sendPwm();
      break;
      case OPTION::_calibration://calibrar
        _reqAutoCal();
      break;
      case OPTION::_identify://identificar
        for(int i = 0; i < 10; i++)
        {
          std::string name = "motor_esquerdo_" + std::to_string(i);
          _reqIdentify(0,1.0, false, name.c_str());
          name = "motor_direito_" + std::to_string(i);
          _reqIdentify(1,1.0, false, name.c_str());
        }
      break;
      case OPTION::_graphic://graficos
        _graphic();
      break;
      case OPTION::_rec_coef://dados da calibracao
        _reqParam();
      break;
      case OPTION::_reset: //reseta o microcontrolador
        _reqReset();
      break;
      case OPTION::_close: //terminar
        stop();
      break;
      default:
        _pause("Comando não reconhecido");
      break;
    }
  }
}

void BtRemoteCtrl::_reqOmegas()
{
  double omegas[2];

  bitstream = new uint8_t[1];
  bitstream[0] = CMD_HEAD | CMD_REQ_OMEGA;

  sendBluetoothMessage(idBt, bitstream, 1*sizeof(uint8_t));
  if(read_bluetooth((uint8_t*)omegas, 2*sizeof(double)) == false){
    delete[] bitstream;
    return;
  }
  printf("Left Omega:%f rad/s     Right Omega:%f rad/s\n", omegas[0], omegas[1]);
  printf("Left Speed:%f m/s   Right Speed:%f m/s\n", omegas[0]*RADIUS/REDUCTION, omegas[1]*RADIUS/REDUCTION);
  _pause();
  delete[] bitstream;
}

void BtRemoteCtrl::_BtConnect()
{
  _printListMACs();
  cin >> idBt;
  initBluetoothById(idBt);
  _pause();
}

void BtRemoteCtrl::_BtDisconnect()
{
  closeBluetoothById(idBt);
  _pause("Dispositivo desconectado!");
  idBt = -1;
}

void BtRemoteCtrl::_reqPing()
{
  double time_stemp[2];
  int size = 10, send, rec;

  bitstream = new uint8_t[size];
  memset(bitstream, 0, size);
  bitstream[0] = CMD_HEAD | CMD_PING;

  time_stemp[0] = omp_get_wtime();
  send = sendBluetoothMessage(idBt, bitstream, size);
  rec  = recvBluetoothMessage(idBt, bitstream, size, 1);
  time_stemp[1] = omp_get_wtime();

  if(rec < 0){
    std::cerr << "Erro na leitura do bluetooth" << '\n';
    _BtDisconnect();
  }

  printf("send:%d  \t rec:%d bytes \t time=%0.2fms\n",send, rec, (time_stemp[1] - time_stemp[0])*1000.0);
  delete[] bitstream;

  _pause();
}
void BtRemoteCtrl::_sendRef()
{
  float ref[2];

  bitstream = new uint8_t[5]; //1 byte de HEAD|CMD  e 4 bytes de referencia, 2byte por motor
  memset(bitstream, 0, 5*sizeof(uint8_t));

  printf("Referência para o motor esquerdo ? [-1.0 a 1.0]\n");
  cin >> ref[0];
  printf("Referência para o motor direito ? [-1.0 a 1.0]\n");
  cin >> ref[1];
  cin.ignore(256, '\n');

  bitstream[0] = CMD_HEAD | CMD_REF;
  _encodeFloat(ref[0], ref[1]);

  sendBluetoothMessage(idBt, bitstream, 5*sizeof(uint8_t));
  delete[] bitstream;
}
void BtRemoteCtrl::_sendPwm()
{
  float ref[2];

  bitstream = new uint8_t[5]; //1 byte de HEAD|CMD  e 4 bytes de referencia, 2byte por motor
  memset(bitstream, 0, 5*sizeof(uint8_t));

  printf("PWM para o motor esquerdo ? [-1.0 a 1.0]\n");
  cin >> ref[0];
  printf("PWM para o motor direito ? [-1.0 a 1.0]\n");
  cin >> ref[1];
  cin.ignore(256, '\n');

  bitstream[0] = CMD_HEAD | CMD_CONTROL_SIGNAL;
  _encodeFloat(ref[0], ref[1]);

  sendBluetoothMessage(idBt, bitstream, 5*sizeof(uint8_t));
  delete[] bitstream;
}
void BtRemoteCtrl::_reqAutoCal()
{
  bitstream = new uint8_t;
  bitstream[0] = CMD_HEAD | CMD_CALIBRATION;
  sendBluetoothMessage(idBt, bitstream, 1*sizeof(uint8_t));
  delete bitstream;
}

// func_identify(const bool motor, const bool controller, const float setpoint, const float stopTime)
void BtRemoteCtrl::_reqIdentify(bool motor, float ref, bool controller, const char* fileName)
{
  float stime = 2.0;
  import_data_t import;
  double times[2];

  #define MEM_SIZE  (3 + 2*sizeof(float))
  bitstream = new uint8_t[MEM_SIZE];
  memset(bitstream, 0, MEM_SIZE);
  bitstream[0] = CMD_HEAD | CMD_IDENTIFY;

  // identify config.
  import.motor = motor;
  import.controller = controller;
  import.setpoint = ref;

  // sending identification request
  bitstream[1] = import.motor;
  bitstream[2] = import.controller;
  memcpy(bitstream + 3, (void*)&import.setpoint, sizeof(float));
  memcpy(bitstream+3+sizeof(float), (void*)&stime, sizeof(float));
  sendBluetoothMessage(idBt, bitstream, MEM_SIZE*sizeof(uint8_t));

  times[0] = omp_get_wtime();

  if(read_bluetooth((uint8_t*)&import.params, sizeof(parameters_t), 5) == false){
    delete[] bitstream;
    return;
  }
  if(read_bluetooth((uint8_t*)&import.OmegaMax, sizeof(double), 2) == false){
    delete[] bitstream;
    return;
  }
  if(read_bluetooth((uint8_t*)&import.size, sizeof(uint16_t), 2) == false){
    delete[] bitstream;
    return;
  }

  cout << (int)import.size <<" amostras\n";
  import.datas = new export_data_t[import.size];
  memset(import.datas, 0, import.size * sizeof(export_data_t));
  cout << "Recebendo:" << (int)import.size << " amostras ..." <<"\n";
  for(int i = 0; i < import.size; i++)
  {
    if(read_bluetooth((uint8_t*)&import.datas[i], sizeof(export_data_t), 5) == false){
      delete[] import.datas;
      delete[] bitstream;
      return;
    }
  }
  cout << "Leitura completada com sucesso!\n";
  times[1]= omp_get_wtime();
  _saveToFile(fileName, import, false);
  cout << "A rotina levou "<< times[1] - times[0] << "s para ser concluída.\n";
  // _pause();

  delete[] import.datas;
  delete[] bitstream;
}
void BtRemoteCtrl::_graphic()
{
  int r = system("python3 etc/_pyplotter.py");
  if(!!r)std::cerr << "Falha com o system" << '\n';
}
void BtRemoteCtrl::_reqParam()
{
  bitstream = new uint8_t[1];
  parameters_t parameters[2];
  bitstream[0] = CMD_HEAD | CMD_REQ_CAL;
  sendBluetoothMessage(idBt, bitstream, 1*sizeof(uint8_t));
  if(read_bluetooth((uint8_t*)&parameters, 2*sizeof(parameters_t)) == false){
      delete[] bitstream;
      return;
  }
  _printParameters(parameters);
  delete[] bitstream;
  _pause();
}
void BtRemoteCtrl::_reqReset()
{
  int send;
  bitstream = new uint8_t[1];
  bitstream[0] = (CMD_HEAD | CMD_RESET);

  send = sendBluetoothMessage(idBt, bitstream, sizeof(uint8_t));
  if(send < 0){
    _BtDisconnect();
    _pause("Falha ao enviar o comando. Bluetooth Desconectado");
  }

  delete[] bitstream;
}

void BtRemoteCtrl::_saveToFile(const char* file, const import_data_t import, bool append)const
{
  string fileName;
  ofstream arq;
  fileName = string("etc/")+ string(file) +string(".csv");

  if(append)
    arq.open(fileName , std::ofstream::app);
  else
    arq.open(fileName);

  if(!append){
    arq << "MOTOR,CONTROLLER,SET_POINT,OMEGA_MAX,";
    arq << "K,TAU,FORWARD_KP,BACK_KP,FORWARD_ANG_COEF,FORWARD_LIN_COEF,BACK_ANG_COEF,BACK_LIN_COEF,";
    arq << "TIME,OMEGA_RAW,OMEGA_FILTERED,OMEGA_PREDICTED,K_GAIN,PREDIC_ERR,MEASURE_ERR";
  }

  for(int i = 0; i < import.size; i++)
  {
    arq << '\n';
    arq << import.motor << ',' << import.controller << ',' << import.setpoint << ',' << import.OmegaMax << ',';
    arq << import.params.K << ',' << import.params.tau << ',' << import.params.Kp[0] << ',' << import.params.Kp[1] << ',';
    arq << import.params.coef[0].ang << ',' << import.params.coef[0].lin << ',';
    arq << import.params.coef[1].ang << ',' << import.params.coef[1].lin << ',';
    arq << import.datas[i].dt << ',';
    arq << import.datas[i].encoder.rawOmega << ',' << import.datas[i].encoder.omega <<','<<import.datas[i].encoder.pOmega << ',';
    arq << import.datas[i].encoder.kGain << ',' << import.datas[i].encoder.p << ',' << import.datas[i].encoder.r;
  }

  arq.close();
  return;
}

void BtRemoteCtrl::_encodeFloat(const float left_f, const float right_f)
{
  uint16_t ref_b[2];

	ref_b[0] = ( (left_f > 0) << 15)   | (uint16_t)(fabs(left_f)*32767.0);
  ref_b[1]= ( (right_f > 0) << 15)   | (uint16_t)(fabs(right_f)*32767.0);

  // bitstream[0] reservado para o HAED e CMD
  bitstream[1] = (ref_b[0] & 0xFF00) >> 8;
  bitstream[2] = (ref_b[0] & 0x00FF);
  bitstream[3] = (ref_b[1] & 0xFF00) >> 8;
  bitstream[4] = (ref_b[1] & 0x00FF);
}

OPTION BtRemoteCtrl::_mainMenu()
{
  static int choice;
  int r = system("clear");

  if(idBt != -1)cout << "****************Status: Conectado****************\n";
  else cout << "****************Status: Desconectador****************\n";

  if(r == -1)cerr << "System Error\n";
  cout <<"****************MENU DE AÇÕES*************\n";
  cout <<"O -> PEDIR OMEGAS ATUAIS\n";
  cout <<"************************************\n";
  cout <<"C -> CONECTAR\n";
  cout <<"D -> DESCONECTAR\n";
  cout <<"P -> PING\n";
  cout <<"************************************\n";
  cout <<"R -> ENVIAR REFERÊNCIAS\n";
  cout <<"S -> ENVIAR PWM\n";
  cout <<"************************************\n";
  cout <<"T -> AUTO TUNNING\n";
  cout <<"I -> IDENTIFICAÇÃO\n";
  cout <<"V -> VISUALIZAR GRAFICOS\n";
  cout <<"M -> PEDIR DADOS DA CALIBRACAO\n";
  cout <<"************************************\n";
  cout <<"1 -> RESETAR O MICROCONTROLADOR\n";
  cout <<"Q -> ENCERRAR O PROGRAMA\n";
  cout << "Escolha uma opção: ";
  // cin.clear();
  // cin.ignore(numeric_limits<streamsize>::max(),'\n');
  choice = cin.get();
  choice = toupper(choice);

  return (OPTION)choice;
}

void BtRemoteCtrl::_printParameters(const parameters_t parameters[])const
{
  // printf("Omega Max: %f rad/s = %f m/s\n", parameters.omegaMax, parameters.omegaMax*RADIUS/(REDUCTION));//reducao de 30 e 24 interrupcoes por volta
  printf("Left  Front  => a = %f , b = %f \n", parameters[0].coef[0].ang, parameters[0].coef[0].lin);
  printf("Left  Back   => a = %f , b = %f \n", parameters[0].coef[1].ang, parameters[0].coef[1].lin);
  printf("Right Front  => a = %f , b = %f \n", parameters[1].coef[0].ang, parameters[1].coef[0].lin);
  printf("Right Back   => a = %f , b = %f \n", parameters[1].coef[1].ang, parameters[1].coef[1].lin);
  printf("Left  Kp_frente = %.12f, Kp_tras %.12f\n", parameters[0].Kp[0], parameters[0].Kp[1]);
  printf("Right Kp_frente = %.12f, Kp_tras %.12f\n", parameters[1].Kp[0], parameters[1].Kp[1]);
  printf("Left  K = %.12f\n", parameters[0].K);
  printf("Right K = %.12f\n", parameters[1].K);
  printf("Left  tau = %.12f\n", parameters[0].tau);
  printf("Right tau = %.12f\n", parameters[1].tau);
}

void BtRemoteCtrl::_pause(const char* msg)const
{
  printf("%s\n(PRESSIONE ENTER PARA CONTINUAR)\n", msg);
  cin.clear();
  cin.ignore(numeric_limits<streamsize>::max(),'\n');
  cin.get();
}

void BtRemoteCtrl::_printListMACs()
{
  printf("****************MACs Conhecidos*************\n");
  vector<string> addrs = getDest();
  for(int i = 0; i < (int)addrs.size(); i++)
  {
    printf("%d -> Robo_%d = %s\n",i, i, addrs[i].c_str());
  }
};

bool BtRemoteCtrl::read_bluetooth(uint8_t*msg, const size_t lengthMax, const int timeout)
{
  int rec;
  if((msg == NULL) || (lengthMax == 0) || (idBt < 0))return false;
  rec = recvBluetoothMessage(idBt, msg, lengthMax, timeout);
  if(rec < 0){
    cerr << "Erro na leitura do bluetooth\n";
    _BtDisconnect();
    return false;
  }
  return true;
}

/***********************************************************************/
