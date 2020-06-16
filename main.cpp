#include <thread>
#include <iostream>
#include "btcontroller.hpp"

using namespace std;

#define MAC_ESP_TEST   "30:AE:A4:3B:A4:26"
#define MAC_ESP_ROBO_1 "30:AE:A4:20:0E:12"
#define MAC_ESP_ROBO_2 "30:AE:A4:13:F8:AE"
#define MAC_ESP_ROBO_3 "30:AE:A4:20:0E:12"
#define MAC_ESP_ROBO_4 "30:AE:A4:41:74:F6"


int main()
{
  BtRemoteCtrl remoteCtrl;

  remoteCtrl.setBluetoothAddr(MAC_ESP_TEST);
  remoteCtrl.setBluetoothAddr(MAC_ESP_ROBO_1);
  remoteCtrl.setBluetoothAddr(MAC_ESP_ROBO_2);
  remoteCtrl.setBluetoothAddr(MAC_ESP_ROBO_3);
  remoteCtrl.setBluetoothAddr(MAC_ESP_ROBO_4);

  remoteCtrl.start();
  cout << "Encerrando conexao e Fechando o programa\n";

  return 0;
}
