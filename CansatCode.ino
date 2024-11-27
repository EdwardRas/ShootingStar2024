#include <Servo.h>
#include <CanSatKit.h>
#include <LM35.h>
//#include biblioteka dla kart microSD

using namespace CanSatKit;

bool isFlying = false;
bool isAirbagDeployed = false;
float pressure = 0;
float temperature = 0;
float altitude = 0;
float prevAltitude = 0;
float acceleration = 0;
float altChange = 0;
int t = 0;

BMP280 PresSensor;
LM35 TempSensor(LM35pin);

void setup() {
  // put your setup code here, to run once:
  PresSensor.begin();
  //not sure if its needed
  TempSensor.begin();
  //Only for testing:
  Serial.begin(9600);
  if(!PresSensor.begin()){
    Serial.println(BMP280 init failed!);
  }
  else{
    Seiral.println(BMP280 init succesful);
  }
   PresSensor.setOversampling(16);
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
    temperature = TempSensor.cel();
    //get pressure;
    pressure = PresSensor.getPressure();
    //store previous altitude;
    prevAltitude = altitude;
    //use barometric formula to calculate approx altitude;
    //calculate change in altitude (altitude - previous altitude);
    //if change in altitude <= (-)TBD and isAirbagDeployed == false, deploy airbag(servo to 90 degrees); airbagDeployed = true;
    //record all data on sd card;
    //send all data (acceleration, temperature, pressure, altitude, change in altitude, airbagStatus) via radio;
    //delay(1000/750/500);
  }
}
