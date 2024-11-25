#include <Servo.h>
#include <CanSatKit.h>
#include <LM35.h>
#include <mpu9250.h>
//#include biblioteka dla kart microSD

using namespace CanSatKit;

bool isFlying = false;
bool isAirbagDeployed = false;
int pressure = 0;
int temperature = 0;
int altitude = 0;
int prevAltitude = 0;
int acceleration = 0;
int altChange = 0;
int t = 0;

BMP280 bmp;

void setup() {
  // put your setup code here, to run once:
  //Only for testing:
  Serial.begin(9600);
  if(!bmp.begin()){
    Serial.println(BMP280 init failed!);
  }
  else{
    Seiral.println(BMP280 init succesful);
  }
   bmp.setOversampling(16);
}

void loop() {
  // put your main code here, to run repeatedly:
  //On the ground mode, detect if acceleration is greater than 10 m/s^2
  if(!isFlying){
    //prevAltitude = altitude
    //get altitude
    // if altitude - prev altitude >= 10m/s, flying = 1; break;
  }
  // Flight mode, conduct all measurements and check to deploy airbag, send and record data
  else{
    t++;
    //get acceleration;
    //get temperature;
    //get pressure;
    bmp.measurePressure(pressure);
    //store previous altitude;
    //use barometric formula to calculate approx altitude;
    //calculate change in altitude (altitude - previous altitude);
    //if change in altitude <= (-)TBD and isAirbagDeployed == false, deploy airbag(servo to 90 degrees); airbagDeployed = true;
    //record all data on sd card;
    //send all data (acceleration, temperature, pressure, altitude, change in altitude, airbagStatus) via radio;
    //delay(1000/750/500);
  }
}

