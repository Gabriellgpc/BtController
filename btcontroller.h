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

enum OPTION{
  _rec_omegas  = 1,
  _connect     = 2,
  _disconnect  = 3,
  _ping        = 4,
  _send_ref    = 5,
  _send_pwm    = 6,
  _calibration = 7,
  _identify    = 8,
  _graphic     = 9,
  _rec_coef    = 10,
  _close       = 11
};

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
