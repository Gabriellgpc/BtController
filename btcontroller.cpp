#include <locale>         // std::locale, std::toupper
#include <limits>       // std::numeric_limits
#include <string>
#include <fstream>        //ofstream
#include <iostream>          //cout, cerr, cin
#include <cstdio>            //printf
#include <unistd.h>          //sleep
#include <cmath>
#include <omp.h>

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
    if(idBt != -1)
      cout << "Status: Conectado\n";
    else
      cout << "Status: Desconectado\n";
    _printMainMenu();
    cout << "Escolha uma opção: ";
    choice = cin.get();
    choice = toupper(choice);
    cin.ignore(numeric_limits<streamsize>::max(),'\n');

    if(idBt == -1 && (choice != _graphic &&
                      choice != _connect &&
                      choice != _reset &&
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
      {
        int size = 10;
        // for(size = 10; size < 990; size += 10)
        // {
          bitstream = new uint8_t[size];
          memset(bitstream, 0, size);
          bitstream[0] = CMD_HEAD | CMD_PING;

          time_stemp[0] = omp_get_wtime();
          send = btAction.sendBluetoothMessage(idBt, bitstream, size);
          rec  = btAction.recvBluetoothMessage(idBt, bitstream, size, 1);
          time_stemp[1] = omp_get_wtime();

          printf("send:%d  \t rec:%d bytes \t time=%0.2fms\n",send, rec, (time_stemp[1] - time_stemp[0])*1000.0);
          delete[] bitstream;
        // }

        _pause();
        break;
      }
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
        // _funcIdentify(idBt);
        break;
      }
      case OPTION::_graphic://graficos
        system("python3 etc/_pyplotter.py");
      break;
      case OPTION::_rec_coef://dados da calibracao
        bitstream = new uint8_t[1];
        parameters_t parameters[2];
        bitstream[0] = CMD_HEAD | CMD_REQ_CAL;
        btAction.sendBluetoothMessage(idBt, bitstream, 1*sizeof(uint8_t));
        rec = btAction.recvBluetoothMessage(idBt, (uint8_t*)&parameters, 2*sizeof(parameters_t), 5);
        printf("Coeficientes da calibracao tamanho total:%d bytes\n", rec);
        _printParameters(parameters);
        _pause();
        delete[] bitstream;
        break;
      case OPTION::_reset: //reseta o microcontrolador
        bitstream = new uint8_t[1];
        bitstream[0] = (CMD_HEAD | CMD_RESET);
        send = btAction.sendBluetoothMessage(idBt, bitstream, 1);
        if(send < 0)_pause("Falha ao enviar o comando");
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
