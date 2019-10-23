#include <QTRSensors.h>

 // experiment to determine this, slowly increase the speeds and adjust this value. ( Note: Kp < Kd) 
#define rightMaxSpeed 75 // max speed of the robot
#define leftMaxSpeed 75 // max speed of the robot
#define rightBaseSpeed 50 // this is the speed at which the motors should spin when the robot is perfectly on the line
#define leftBaseSpeed 50  // this is the speed at which the motors should spin when the robot is perfectly on the line
#define NUM_SENSORS  8     // number of sensors used
#define TIMEOUT       2500  // waits for 2500 us for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

//#define rightMotor1 12
#define rightMotor2 4
#define rightMotorPWM 10
//#define leftMotor1 12
#define leftMotor2 13
#define leftMotorPWM 11
//#define motorPower 8

QTRSensorsRC qtrrc((unsigned char[]) {3, 4, A0, A1, A2, A3, A4, A5} ,NUM_SENSORS, TIMEOUT, EMITTER_PIN); // sensor connected through analog pins A0 - A5 i.e. digital pins 14-19

unsigned int sensorValues[NUM_SENSORS];

void setup()
{
 // pinMode(rightMotor1, OUTPUT);
  pinMode(rightMotor2, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);
  //pinMode(leftMotor1, OUTPUT);
  pinMode(leftMotor2, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
 // pinMode(motorPower, OUTPUT);
  
  int i;
for (int i = 0; i < 100; i++) // calibrate for sometime by sliding the sensors across the line, or you may use auto-calibration instead

  /* comment this part out for automatic calibration 
  if ( i  < 25 || i >= 75 ) // turn to the left and right to expose the sensors to the brightest and darkest readings that may be encountered
     turn_right();  
   else
     turn_left(); */ 
   qtrrc.calibrate();   
   delay(20);
wait();  
delay(2000); // wait for 2s to position the bot before entering the main loop 
    
     
    
    Serial.begin(9600);
    
    
  } 

int lastError = 0;

void loop()
{
  unsigned int sensors[8];
  int position = qtrrc.readLine(sensors);
  int error = position - 3500;
  
  double Kp = 50; // experiment to determine this, start by something small that just makes your bot follow the line at a slow speed
  double Ki = 0;
  double Kd = 0;
  double I = I + error;
    
  
  

  int motorSpeed = (Kp * error) + (Ki * I) +  (Kd * (error - lastError));
  lastError = error;

  int rightMotorSpeed = rightBaseSpeed + motorSpeed;
  int leftMotorSpeed = leftBaseSpeed - motorSpeed;
  
    if (rightMotorSpeed > rightMaxSpeed ) rightMotorSpeed = rightMaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > leftMaxSpeed ) leftMotorSpeed = leftMaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < 0) rightMotorSpeed = 0; // keep the motor speed positive
  if (leftMotorSpeed < 0) leftMotorSpeed = 0; // keep the motor speed positive
  
   {
  //digitalWrite(motorPower, HIGH); // move forward with appropriate speeds
  //digitalWrite(rightMotor1, HIGH);
  digitalWrite(rightMotor2, HIGH);
  analogWrite(rightMotorPWM, rightMotorSpeed);
 // digitalWrite(motorPower, HIGH);
 // digitalWrite(leftMotor1, HIGH);
  digitalWrite(leftMotor2, LOW);
  analogWrite(leftMotorPWM, leftMotorSpeed);


  Serial.println(position);
  
}
}
  
void wait(){
   analogWrite(rightMotorPWM, 0);
   analogWrite(leftMotorPWM, 0);
}
