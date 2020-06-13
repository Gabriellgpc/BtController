#include <locale>         // std::locale, std::toupper
#include <limits>       // std::numeric_limits
#include <string>
#include <fstream>        //ofstream
#include <iostream>          //cout, cerr, cin
#include <cstdio>            //printf
#include <unistd.h>          //sleep
#include <cmath>
#include <omp.h>

#include "bluetoothAction/bluetoothAction.h"
#include "btcontroller.h"

int main(int argc, char** argv)
{
  double  time_stemp[2];
  uint8_t *bitstream;
  int     idBt = -1;
  int choice = 0;

  btAction.setBluetoothAddr(MAC_ESP_TEST);
  btAction.setBluetoothAddr(MAC_ESP_ROBO_1);
  btAction.setBluetoothAddr(MAC_ESP_ROBO_2);
  btAction.setBluetoothAddr(MAC_ESP_ROBO_3);
  btAction.setBluetoothAddr(MAC_ESP_ROBO_4);

  bool run = true;
  int  send = 0,rec = 0;
  while(run)
  {
    system("clear");
    _printMainMenu();
    cout << "Escolha uma opção: ";
    choice = cin.get();
    choice = toupper(choice);
    cin.ignore(numeric_limits<streamsize>::max(),'\n');

    if(idBt == -1 && (choice != _graphic &&
                      choice != _connect &&
                      choice != _close))
    {
          _pause("Nenhum dispositivo conectado!");
          continue;
    }

    switch (choice)
    {
    case OPTION::_rec_omegas://solicitar omegas
    {
      bitstream = new uint8_t[1];
      double *vec_double = new double[2];

      memset(vec_double, 0, 2*sizeof(double));

      bitstream[0] = CMD_HEAD | CMD_REQ_OMEGA;
      btAction.sendBluetoothMessage(idBt, bitstream, 1*sizeof(uint8_t));
      rec = btAction.recvBluetoothMessage(idBt, (uint8_t*)vec_double, 2*sizeof(double), 2);

      if(rec != 2*sizeof(double))
      {
        _pause("Erro na leitura");
        break;
      }
      printf("   Omega Left:%f rad/s     Omega Right:%f rad/s\n", vec_double[0], vec_double[1]);
      printf("Velocity Left:%f m/s   Velocity Right:%f m/s\n", vec_double[0]*RADIUS/REDUCTION, vec_double[1]*RADIUS/REDUCTION);
      _pause();

      delete[] bitstream;
      delete[] vec_double;
      break;
    }
    case OPTION::_connect://conectar
      _printListMACs();
      cin >> idBt;
      btAction.initBluetoothById(idBt);
      _pause();
      break;
    case OPTION::_disconnect://desconectar
      btAction.closeBluetoothById(idBt);
      _pause("Dispositivo desconectado!");
      idBt = -1;
      break;
    case OPTION::_ping: //_ping
      bitstream = new uint8_t[64];

      memset(bitstream, 'A', 64);
      bitstream[0] = CMD_HEAD | CMD_PING;

      time_stemp[0] = omp_get_wtime();
      send = btAction.sendBluetoothMessage(idBt, bitstream, 64);
      rec  = btAction.recvBluetoothMessage(idBt, bitstream, 64, 1);
      time_stemp[1] = omp_get_wtime();

      printf("send:%d  \t rec:%d bytes \t time=%0.2fms\n",send, rec, (time_stemp[1] - time_stemp[0])*1000.0);
      delete[] bitstream;
      _pause();

      break;
    case OPTION::_send_ref://omegas ref
    case OPTION::_send_pwm://omegas sinal de controle
    {
      float *vec_float = new float[2];
      bitstream = new uint8_t[5];
      memset(bitstream, 0, 5*sizeof(uint8_t));
      memset(vec_float, 0, 2*sizeof(float));

      printf("Referencia do motor esquerdo ? (-1.0 a 1.0)\n");
      cin >> vec_float[0];
      printf("Referencia do motor direito ? (-1.0 a 1.0)\n");
      cin >> vec_float[1];

      encodeFloat(vec_float, bitstream+1);

      if(choice == 5){
        bitstream[0] = CMD_HEAD | CMD_REF;
      }else{ //control signal
        bitstream[0] = CMD_HEAD | CMD_CONTROL_SIGNAL;
      }

      btAction.sendBluetoothMessage(idBt, bitstream, 5*sizeof(uint8_t));
      delete[] bitstream;
      delete[] vec_float;
      break;
    }
    case OPTION::_calibration://calibrar
      bitstream = new uint8_t;
      bitstream[0] = CMD_HEAD | CMD_CALIBRATION;
      btAction.sendBluetoothMessage(idBt, bitstream, 1*sizeof(uint8_t));
      delete bitstream;
      break;
    case OPTION::_identify://identificar
    {
      encoder_data_t *vec_data = NULL;
      double omegaRef;
      uint8_t motor, typeC;
      float setpoint;
      int size;
      int sumRec = 0;

      cout << "Motor ? (0 -> Left \t 1 -> Right): ";
      cin >> motor;

      cout << "Setpoint: ";
      cin >> setpoint;

      cout << "Desabilitar controlador ? 0(nao), 1(sim): ";
      cin >> typeC;

      bitstream  = new uint8_t[2 + 1*sizeof(float)];

      bitstream[0]                   = CMD_HEAD | CMD_IDENTIFY;
      bitstream[1]                   = ((motor << 7)  | typeC) & 0b10000001;
      *(float*)&bitstream[2 + 0*sizeof(float)] = setpoint;

      //envia comando para iniciar a identificação
      btAction.sendBluetoothMessage(idBt, bitstream, 2 + 1*sizeof(float));
      time_stemp[0] = omp_get_wtime();

      printf("Esperando...\n");
      //aguarda receber a informação da quantidade de medições
      rec = btAction.recvBluetoothMessage(idBt, (uint8_t*)&size, sizeof(int), 10);
      if(rec == -1)
      {
        _pause("timeout! erro ao receber informacao do size");
        continue;
      }
      else printf("Medicoes:%d\n", size);
      sumRec += rec;
      vec_data = new encoder_data_t[size];
      memset(vec_data, 0, size*sizeof(encoder_data_t));
      //aguarda receber a informação da quantidade de medições
      rec = btAction.recvBluetoothMessage(idBt, (uint8_t*)&omegaRef, sizeof(double), 10);
      if(rec == -1)
      {
        _pause("timeout! erro ao receber informacao do omega de referencia");
        continue;
      }
      else printf("Omega ref:%lf\n", omegaRef);
      sumRec += rec;

      int step = 20;
      for(int i = 0; i < size; i += step)
      {
        rec = btAction.recvBluetoothMessage(idBt, (uint8_t*)&vec_data[i], step*sizeof(encoder_data_t), 20);
        printf("Recebi:%d bytes\n", rec);
        if(rec == -1)
          puts("timeout!");
        sumRec += rec;
      }
      time_stemp[1] = omp_get_wtime();
      printf("Tempo decorrido: %f s\n", time_stemp[1] - time_stemp[0]);

      printf("%d Bytes recebidos, deseja salvar ? (0/1)", sumRec);
      cin >> choice;

      if(choice == 1)
      {
        char fileName[50];
        cout << "Que nome devo colocar no arquivo ? ";
        scanf("%50s", fileName);

        saveToFile(vec_data, size, 2.0, omegaRef, string(fileName));
        cout << "Salvando...\n";
        printf("Salvo! Em: %s\n", (string("etc/") + string(fileName)).c_str());
        _pause();
      }else{
        printf("Tudo bem então...\n");
        _pause();
      }

      delete[] bitstream;
      delete[] vec_data;
      break;
    }
    case OPTION::_graphic://graficos      
      system("python3 etc/_pyplotter.py");
    break;
    case OPTION::_rec_coef://dados da calibracao
      bitstream = new uint8_t[1];
      parameters_t parameters;
      bitstream[0] = CMD_HEAD | CMD_REQ_CAL;
      btAction.sendBluetoothMessage(idBt, bitstream, 1*sizeof(uint8_t));
      rec = btAction.recvBluetoothMessage(idBt, (uint8_t*)&parameters, sizeof(parameters_t), 5);

      printf("Coeficientes da calibracao tamanho total:%d bytes\n", rec);
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
      _pause();

      delete[] bitstream;
      break;
    case OPTION::_close: //terminar
      run = false;
      break;
    default:
      _pause("Comando não reconhecido");
      break;
    }
  }

  printf("Encerrando conexao e Fechando o programa\n");
  if(idBt != -1)
    btAction.closeBluetoothById(idBt);

  return 0;
}

void
saveToFile(encoder_data_t vec_data[],
           const int size,
           const float timeout,
           const double omegaRef,
           const string &fileName)
{
  string fileR = string("etc/raw_") + fileName + string(".out");
  string fileF = string("etc/filtered_") + fileName + string(".out");
  string fileP = string("etc/predicted_") + fileName + string(".out");
  string fileG = string("etc/gain_") + fileName + string(".out");
  string fileU = string("etc/uncertainty_") + fileName + string(".out");

  ofstream outR(fileR);
  ofstream outF(fileF);
  ofstream outP(fileP);
  ofstream outG(fileG);
  ofstream outU(fileU);

  outR << size << ',' << timeout << ',' << omegaRef;
  outF << size << ',' << timeout << ',' << omegaRef;
  outP << size << ',' << timeout << ',' << omegaRef;
  outG << size << ',' << timeout;
  outU << size << ',' << timeout;

  cout << "Save to File\n";
  for(int i = 0; i < size; i++){
    outR << ',' << vec_data[i].rawOmega;
    outF << ',' << vec_data[i].omega;
    outP << ',' << vec_data[i].pOmega;
    outG << ',' << vec_data[i].kGain;
    outU << ',' << vec_data[i].p;
  }

  outR.close();
  outF.close();
  outP.close();
  outG.close();
  outU.close();
}
