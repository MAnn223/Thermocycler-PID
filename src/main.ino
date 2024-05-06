#include <Arduino.h>
#include <PID_v1.h>
#include <PID_v1.h>
double driverOut = 10;
double difference = 10;
double setPoint = 10;
unsigned long changeTime = 0;
unsigned long currentTime = millis();
unsigned long prevTime = millis();
int Kp = 0.01;
int Ki = 0.2;
int Kd = 0.02;
double tempError;
double targetTemp;
double currentTemp;
PID myPID(&difference, &driverOut, &setPoint,Kp,Ki,Kd, DIRECT);
void setup() {
  // put your setup code here, to run once:
  myPID.SetMode(AUTOMATIC);
  //attachInterrupt(23, control, CHANGE);
  //8 pwm, 7 tacho, 27 5v
  pinMode(5, OUTPUT); //Heat
  pinMode(6, OUTPUT); //Cool
  pinMode(23, OUTPUT); //SDA
  pinMode(24, INPUT); //SCL
  interrupts();
  noInterrupts();

}

void control() {
  //noInterrupts();
  //get temp data
  //interrupts();
  
  myPID.Compute();
  digitalWrite(5, driverOut);
};

void calculate() {
  tempError = currentTemp - targetTemp;
  currentTime = millis();
  difference = currentTime - prevTime;
  prevTime = currentTime;
}
//attachInterrupt();
// void loop() {
//   // put your main code here, to run repeatedly:
  

// }

