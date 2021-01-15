#ifndef __COMMON_H__
#define __COMMON_H__

#include <cstdint>

//Comandos do bluetooth
#define CMD_HEAD           0xA0

#define CMD_REQ_CAL        0x00
#define CMD_REQ_OMEGA      0x03

#define CMD_CALIBRATION    0x04
#define CMD_IDENTIFY       0x05

#define CMD_REF            0x0A
#define CMD_CONTROL_SIGNAL 0x0B
#define CMD_SET_KI         0x0C

#define CMD_RESET          0x0E
#define CMD_PING           0x0F

#define RADIUS    1.5/100 //metros
#define REDUCTION 30 //30x1

enum ROTATE_S
{
  FRONT = 0,
  BACK  = 1
};
enum MOTOR
{
  LEFT = 0,
  RIGHT= 1
};

typedef struct
{
  double ang; //coef. angular da reta
  double lin;  //coef. linear da reta
}coefLine_t;

typedef struct
{
  uint8_t *data;
  uint32_t len;
}bt_data_t;

typedef struct
{
  double  rawOmega;  //ultimo omega medido, sem filtro
  double  omega;     //omega filtrado
  double  pOmega;    //predicted omega
  double  kGain;     //kalman gain
  double  p;         //preditec variance, incerteza da estimativa
  double  r;         //measure variance, incerteza da medição
}encoder_data_t;

typedef struct{
  double wss;
  double tau;
}input_encoder_t;

typedef struct
{
  //Parâmetros natural do Sistema
  double K;      //Ganho do sistema
  double tau;    //Constante de tempo do sistema
  // Controlador Proporcional e Forward
  double Kp[2];  //Ganho do controlador proporcional, para cada sentido
  double Ki;  //Ganho do controlador proporcional, para cada sentido
  // Coef. Angular == 1/K
  // Coef. Linear  == Zona morta
  coefLine_t coef[2];// Coef. das retas Omega x PWM para cada sentido de rotação
}parameters_t;

typedef struct
{
  //Velocidade máxima do robô
  double omegaMax;
  // Parametros dos motores
  parameters_t params[2];
}memory_data_t;

typedef struct
{
  double dt;    //delta t desde considerando t0 como tempo onde que foi aplicado o input
  encoder_data_t encoder;
}export_data_t;


#endif
