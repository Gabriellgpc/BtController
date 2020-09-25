#ifndef __BTCONTROLL_HPP__
#define __BTCONTROLL_HPP__

#include <thread>
#include "common.h"
#include "bluetoothAction/bluetoothAction.h"

void func_manager(void*X);

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

struct import_data_t{
  //config
  int motor, controller;
  float setpoint;
  //import from microC
  parameters_t params;
  double OmegaMax;
  export_data_t *datas;
  uint16_t size;
};

class BtRemoteCtrl: public BluetoothAction
{
public:
  void start();
  void stop();

  BtRemoteCtrl();
  ~BtRemoteCtrl();
private:
  // metodos
  void _manager();

  void _reqOmegas();
  void _BtConnect();
  void _BtDisconnect();
  void _reqPing();
  void _sendRef();
  void _sendPwm();
  void _reqAutoCal();
  void _reqIdentify(bool motor, float ref = 1.0, bool controller = false,const char* fileName = "output");
  void _graphic();
  void _reqParam();
  void _reqReset();

  // metodos auxiliares
  void _encodeFloat(const float left_f = 0.0, const float right_f = 0.0);

  OPTION _mainMenu();
  void _printParameters(const parameters_t parameters[])const;
  void _pause(const char* msg = "")const;
  void _printListMACs();
  void _saveToFile(const char* file, const import_data_t import, bool append = false)const;
  bool read_bluetooth(uint8_t*msg = NULL, const size_t lengthMax = 0, const int timeout = 1);
  // propriedades
  std::thread tr_manager;
  uint8_t *bitstream;
  bool running;
  int idBt;
  // friends
  friend void func_manager(void*X);
};
#endif
