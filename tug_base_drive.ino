#include "defines.h"
#include "SimplePID.h"

volatile long encoderPos[] = {0, 0};
volatile long lastPos[] = {0, 0};

// PID constants
double kp = 0.22;
double ki = 20;
double kd = 0;

double input_r = 0, output_r = 0, setpoint_r = 0;
double input_l = 0, output_l = 0, setpoint_l = 0;

PID R_PID(&input_r, &output_r, &setpoint_r, kp, ki, kd, DIRECT);
PID L_PID(&input_l, &output_l, &setpoint_l, kp, ki, kd, DIRECT);

PID pid[2];

double ang_r = 0;
double ang_l = 0;


union {
  byte buff[BUFF_SIZE];
  float value[2];
} converter;


void setPwm(int motor, int pwm_cmd, bool rev = 0) {
  if (pwm_cmd < MIN_PWM)
  {
    analogWrite(PWM[motor], abs(pwm_cmd));
    digitalWrite(DIR[motor], rev - 1);
  }
  else if (pwm_cmd > MIN_PWM)
  {
    analogWrite(PWM[motor], abs(pwm_cmd));
    digitalWrite(DIR[motor], rev - 0);
  }
  else
  {
    analogWrite(PWM[motor], 0);
    digitalWrite(DIR[motor], 0);
  }
}

void readEvent(int num) {
  for (int i = 0; i < BUFF_SIZE; i++) {
    converter.buff[i] = Wire.read();
  }

  setpoint_l = converter.value[LFT];
  setpoint_r = converter.value[RGT];

#ifdef DEBUG
  //  Serial.print("Num of bytes: ");
  //  Serial.print(num);
  //  Serial.print(", id: ");
  //  Serial.print(converter.buff[4]);
  //  Serial.print(", setpoint: ");
  //  Serial.print(converter.value, 4);
  //  Serial.println("");
#endif
}

void writeEvent() {
  converter.value[LFT] = ang_l;
  converter.value[RGT] = ang_r;

  ang_l = 0;
  ang_r = 0;

  Wire.write(converter.buff, BUFF_SIZE);

#ifdef DEBUG
  //  Serial.print("Num of bytes: ");
  //  Serial.print(num);
  //  Serial.print(", id: ");
  //  Serial.print(converter.buff[4]);
  //  Serial.print(", setpoint: ");
  //  Serial.print(converter.value, 4);
  //  Serial.println("");
#endif
}

void setup() {
  // I2C setup
  Wire.begin(I2C_ADDR);         // join i2c bus with address #4
  Wire.onRequest(writeEvent); // register event
  Wire.onReceive(readEvent); // register event

#ifdef DEBUG
  Serial.begin(BAUDRATE);           // start serial for output
#endif

  // PID setup
  R_PID.SetMode(AUTOMATIC);
  R_PID.SetSampleTime(1);
  R_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);

  L_PID.SetMode(AUTOMATIC);
  L_PID.SetSampleTime(1);
  L_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);

  analogWriteResolution(8);

  pinMode(PWM[RGT], OUTPUT);   //  motor PWMs
  pinMode(DIR[RGT], OUTPUT);

  pinMode(encoderA[RGT], INPUT_PULLUP);
  pinMode(encoderB[RGT], INPUT_PULLUP);

  attachInterrupt(encoderA[RGT], rightEncoder, RISING);


  pinMode(PWM[LFT], OUTPUT);   //  motor PWMs
  pinMode(DIR[LFT], OUTPUT);

  pinMode(encoderA[LFT], INPUT_PULLUP);
  pinMode(encoderB[LFT], INPUT_PULLUP);

  attachInterrupt(encoderA[LFT], leftEncoder, RISING);

  resetMotors();
  //      setMotors(6);
}


void loop() { //Do stuff here
  now = millis();
  int timeChange = (now - prev);
  if (timeChange >= dt )
  {
    int diff_r = encoderPos[RGT] - lastPos[RGT];
    int diff_l = encoderPos[LFT] - lastPos[LFT];

    lastPos[RGT] = encoderPos[RGT];
    lastPos[LFT] = encoderPos[LFT];

    ang_r += dpc * diff_r; //change in position in degrees of the wheel
    ang_l += dpc * diff_l; //change in position in degrees of the wheel

    input_r = dpc * diff_r * 1000.0 / (float)timeChange;
    input_l = dpc * diff_l * 1000.0 / (float)timeChange;

    R_PID.Compute();
    L_PID.Compute();

    setPwm(RGT, output_r, 1);
    setPwm(LFT, output_l);

    prev = now;
  }
}




void setMotors(int val) {
  analogWrite(PWM[RGT], val);
  digitalWrite(DIR[RGT], 0);     // ccw

  analogWrite(PWM[LFT], val);
  digitalWrite(DIR[LFT], 0);     //ccw
}

void resetMotors() {
  analogWrite(PWM[RGT], 0);
  digitalWrite(DIR[RGT], 0);

  analogWrite(PWM[LFT], 0);
  digitalWrite(PWM[LFT], 0);
}




// encoder 1
void rightEncoder() {
  // pulse and direction, direct port reading to save cycles
  //  if (digitalRead(encoderB1) == HIGH)   count --;
  //  else                                  count ++;

  // PA12
  if (GPIOA->IDR & (1 << 12)) encoderPos[RGT]--;
  else                        encoderPos[RGT]++;
}

// encoder 2
void leftEncoder() {
  // PC13
  if (GPIOC->IDR & (1 << 13)) encoderPos[LFT]--;
  else                        encoderPos[LFT]++;
}
