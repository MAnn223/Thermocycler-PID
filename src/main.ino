#include <Arduino.h>
#include <PID_v1.h>
#include <PID_v1.h>
#include <Wire.h>
#include <TMP1075.h>
double driverOut = 10;
double difference = 10;
//double setPoint = 10;
unsigned long changeTime = 0;
unsigned long currentTime = millis();
unsigned long prevTime = millis();
int Kp = 0.01;
int Ki = 0.2;
int Kd = 0.02;
double tempError;
double targetTemp = 0;
double currentTemp = 0;
const int HEAT = 5;
const int COOL = 6;
const int SDA = 23;
const int SCL = 24;
const int READ_TEMP = 23;
const int TEMP_CLK = 24;
int heatState = 0;
int coolState = 0;
const int INITIALSTATE = 0; // 0 for HEAT, 1 for COOL
unsigned long phase12 = 195000; //3 min and 15 sec, 95
unsigned long phase3 = 30000; //30 sec, 60
unsigned long phase4 = 15000; //15 sec, 72
unsigned long phase6 = 240000; //4 min
unsigned long oneCycle = 240000;
unsigned long pidInterval = 5000; //figure out what sampling rate would be best
int cycleCount = 0;
int tempSensorAddress = 0x75;

PID myPID(&currentTemp, &driverOut, &targetTemp,Kp,Ki,Kd, DIRECT);
void setup() {
  // put your setup code here, to run once:
  myPID.SetMode(AUTOMATIC);
  //attachInterrupt(23, calculate, CHANGE);
  //8 pwm, 7 tacho, 27 5v
  pinMode(HEAT, OUTPUT); //Heat
  pinMode(COOL, OUTPUT); //Cool
  pinMode(SDA, OUTPUT); //SDA
  pinMode(SCL, INPUT); //SCL
  heatState = INITIALSTATE;
  coolState = !INITIALSTATE;
  Serial.begin(9600);

}
void tempCurve() 
{
  
  for(int i = 0; i <30; i++) {
    unsigned long currentTime = millis();
    while (currentTemp < oneCycle) {
      if(currentTime < phase12) {
        targetTemp = 95;
      }
      else if (currentTime < phase12 + phase3) {
        targetTemp = 60;
      }
      else if (currentTime < phase12 + phase3 + phase4) {
        targetTemp = 72;
      } else {
        break;
      }
      if (millis() - currentTime >= pidInterval) {
        calculate();  
        currentTime = millis();  
    }
    }
    cycleCount++;
  }

}
void loop() {
  tempCurve();
  delay(1000);  //delay for 1 sec
}
// void control() {
//   //noInterrupts();
//   //get temp data
//   //interrupts();
  
// };

float getTemp() {
  //code to get temp reading 
  // Wire.beginTransmission(tempSensorAddress); //need to figure out temp sesnor address
  // Wire.requestFrom(tempSensorAddress, 2); //how many bytes of data needed?
  // if(Wire.available()) {
  //   int tempData = Wire.read();
  // }
  TwoWire tWire = Wire;
  TMP1075::TMP1075 tmp1075 = TMP1075::TMP1075(tWire);  
  tWire.begin();
  tmp1075.begin();
  //prevent serial buffer overflow
  // while(Serial.available()) {
  //   Serial.read();
  // }
  double currentTemp = tmp1075.getTemperatureCelsius();
  return tmp1075.getTemperatureCelsius();
}
void updateTemp() {
  //only one supports pwm
  if (driverOut > 0) {
    // Turn on heating
    digitalWrite(HEAT, HIGH);
    digitalWrite(COOL, LOW);
  } else if (driverOut < 0) {
    // Turn on cooling
    digitalWrite(HEAT, LOW);
    digitalWrite(COOL, HIGH);
  } else {
    // Turn off 
    digitalWrite(HEAT, LOW);
    digitalWrite(COOL, LOW);
  }
}

void calculate() {
  // tempError = currentTemp - targetTemp;
  // currentTime = millis();
  // difference = currentTime - prevTime;
  // prevTime = currentTime;
  // currentTemp = getTemp();
  myPID.Compute();
  updateTemp();
}


//functions from simulation code
void swapModes(int &heatState, int &coolState);
void onHeat();
void onCool();
void heatUp(int targetTemp);
void coolDown(int targetTemp);
void shutDown();





void swapModes(int &heatState, int &coolState)
{
  heatState = coolState;
  coolState = !heatState;
  digitalWrite(HEAT, heatState);
  digitalWrite(COOL, coolState);
}

void onHeat() {
  digitalWrite(COOL, LOW);
  digitalWrite(HEAT, HIGH);
}

void onCool() {
  digitalWrite(HEAT, LOW);
  digitalWrite(COOL, HIGH);
}

void heatUp(int targetTemp) {
  while (currentTemp < targetTemp) {
    onHeat();
    delay(100);
    currentTemp++;
    Serial.print(currentTemp);
  }
  shutDown();
}

void coolDown(int targetTemp) {
   while (currentTemp > targetTemp) {
    onCool();
    delay(100);
    currentTemp--;
    Serial.print(currentTemp);
  }
  shutDown();
}

void shutDown() {
  digitalWrite(HEAT, LOW);
  digitalWrite(COOL, LOW);
}

