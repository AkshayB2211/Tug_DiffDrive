
#define LFT 0
#define RGT 1

#define L1  2
#define L2  3
#define R1  4
#define R2  5

#define DRIVE 10
#define LIFT  11

#define CW 1
#define CCW 0

#define DEBUG 0
#define BAUDRATE 115200
#define I2C_ADDR 8

#define BUFF_SIZE 10

#define N_MOTORS  6
#define N_DRIVE   2
#define N_LIFT    4 

#define in(P) ((P->IDR))
#define out(P) ((P->ODR))

#define CPR 280     // for lift motors

#define MAX_PWM 255
#define MIN_PWM 8


#include <PID_v1.h>
#include <Wire.h>


//                  left    right   L1      L2      R1      R2    
int encoderA[]  = { PC14  , PA11  , PB13  , PB14  , PB12  , PB5   };
int encoderB[]  = { PC13  , PA12  , PB3   , PB4   , PA15  , PB15  };
int PWM[]       = { PB8   , PB9   , PB1   , PB0   , PA1   , PA0   };
int DIR[]       = { PA7   , PC15  , PB11  , PB10  , PA5   , PA4   };

// for encoderB direct port manipulation
int pinB[]      = { 13    , 12    , 3     , 4     , 15    , 15    };
int portB[]     = { GPIOC->IDR, GPIOA->IDR, GPIOB->IDR, GPIOB->IDR, GPIOA->IDR, GPIOB->IDR};


unsigned long prev, now;
float cpr = 1000 / 1.3;           // counts per revolution, 1.3-gear ratio
float dpc = 360.0 / cpr;          // degrees per count
float rpc = 2 * PI / cpr;         // radians per count
int dt = 200;                     // sampling time for pid
