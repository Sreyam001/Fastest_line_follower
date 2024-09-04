#include <SparkFun_TB6612.h>
#include <QTRSensors.h>
QTRSensors qtr;
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];


#define AIN1 5
#define BIN1 2
#define AIN2 7
#define BIN2 4
#define PWMA 6
#define PWMB 3
#define STBY 9

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;

float Kp = 0;
float Kd = 0;
float Ki = 0 ;
 

int minValues[9], maxValues[9], threshold[9];

void setup()
{
  Serial.begin(9600);
  //Serial.begin(9600);
  qtr.setTypeRC();
 
  qtr.setSensorPins((const uint8_t[]){A0,A1,A2,A3,A4,A5,A6,A7}, SensorCount);
  qtr.setEmitterPin(1);//LEDON PIN
  //pinMode(11, INPUT_PULLUP);
  //pinMode(12, INPUT_PULLUP);
}


void loop()
{
 // while (digitalRead(11)) {}
  delay(5000);
  calibrate();
  //while (digitalRead(12)) {}
  delay(15000);

  while (1)
  {
    if (analogRead(1) > threshold[1] && analogRead(2) > threshold[2]&& analogRead(7) < threshold[7]&& analogRead(8) < threshold[8] )
    {
      lsp = 0; rsp = lfspeed;
      motor1.drive(0);
      motor2.drive(lfspeed);
    }

    else if (analogRead(8) > threshold[8] && analogRead(7) > threshold[7]&& analogRead(2) < threshold[2]&& analogRead(1) < threshold[1] )
    { lsp = lfspeed; rsp = 0;
      motor1.drive(lfspeed);
      motor2.drive(0);
    }
    else if (analogRead(4) > threshold[4]&& analogRead(5) > threshold[5])
    {
      Kp = 0.0006 * (1000 - (analogRead(4)+analogRead(5)));
      Kd = 10 * Kp;
      //Ki = 0.0001;
      linefollow();
    }
  }
}

void linefollow()
{
  int error = (analogRead(3) - analogRead(6));

  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1.drive(lsp);
  motor2.drive(rsp);

}

void calibrate()
{
  for ( int i = 1; i < 9; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 3000; i++)
  {
    motor1.drive(50);
    motor2.drive(-50);

    for ( int i = 1; i < 9; i++)
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for ( int i = 1; i < 9; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  motor1.drive(0);
  motor2.drive(0);
}
