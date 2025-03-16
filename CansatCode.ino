#include <CanSatKit.h>
#include <floatToString.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <SD.h>

using namespace CanSatKit;

#define externalLM35Pin A0
#define airbagPin 3

bool isFlying = false;
bool isAirbagDeployed = false;
bool isLanded = false;
float altitude = 0;
float prevAltitude = 0;
float zAcceleration = 0;
float altChange = 0;
float prevAltChange = 0;
float rawTemp = 0;
float voltage = 0;
int t = 0;
int airbagCounter = 0;
double temperature = 0;
double pressure = 0;
const int chipSelect = 10;

const char filename[] = "datalog.txt";
// File object to represent file
File myFile;
// string to buffer output
String dataBuffer;
// last time data was written to card:
unsigned long lastMillis = 0;

BMP280 PresSensor;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

Radio radio(Pins::Radio::ChipSelect,
Pins::Radio::DIO0,
433.0,
Bandwidth_125000_Hz,
SpreadingFactor_9,
CodingRate_4_8);

void sendMeasurement (float data){
  char measurement [7];
  floatToString(data, measurement, sizeof(measurement), 4);
//SerialUSB.println(measurement);
radio.transmit(measurement);
myFile.println(measurement);
}

void sendClock(){
  float x = t;
 char time [7];
  floatToString(x, time, sizeof(time), 1);
//SerialUSB.println(time);
radio.transmit(time);
myFile.println(time);
t++;
}

void sendAllMeasurements (void){
  sendClock();
  sendMeasurement(temperature);
  sendMeasurement(pressure);
  sendMeasurement(zAcceleration);
  sendMeasurement(altitude);
  sendMeasurement(altChange);
  sendMeasurement(isAirbagDeployed);
}

float getExternalTemperature(int raw){
  voltage = raw * 5 / (std::pow(2, 12));
  temperature = 100.0 * voltage;
  return temperature;
}

void setup() {
  // put your setup code here, to run once:
  analogReadResolution(12);
  Serial.begin(9600);
  dataBuffer.reserve(1024);
  myFile = SD.open(filename, FILE_WRITE);
  if (!myFile) {
    Serial.print("error opening ");
    Serial.println(filename);
    while (true);
  }  
  radio.begin();
  myFile=SD.open(filename, FILE_WRITE);
  PresSensor.begin();
  Wire.begin();
  byte deviceID = accel.getDeviceID();
  pinMode(airbagPin, OUTPUT);
  #ifndef ESP8266
  while (!SerialUSB); // for Leonardo/Micro/Zero
  #endif
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    SerialUSB.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  //Only for testing:
  SerialUSB.begin(9600);
  if(!PresSensor.begin()){
    SerialUSB.println("BMP280 init failed!");
  }
  else{
    SerialUSB.println("BMP280 init succesful");
  }
   PresSensor.setOversampling(16);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Flight mode, conduct all measurements and check to deploy airbag, send and record data
  if(!isLanded){
    t++;
    //get acceleration;
    sensors_event_t event; 
    accel.getEvent(&event);
    zAcceleration = event.acceleration.z;
    //get pressure;
    PresSensor.measureTemperatureAndPressure(temperature, pressure);
    //get external temperature;
    rawTemp = analogRead(externalLM35Pin);
    getExternalTemperature(rawTemp);
    //store previous altitude;
    prevAltitude = altitude;
    //use formula from wikipedia to calculate approx altitude;
    altitude = (1013 - pressure) / 0.12;
    //calculate change in altitude (altitude - previous altitude);
    prevAltChange = altChange;
    altChange = altitude - prevAltitude;
    //if change in altitude <= (-)TBD and isAirbagDeployed == false, deploy airbag(power on heating); isAirbagDeployed = true;
    if (altitude>=500){
      isFlying = true;
    }    
    if (altChange >= 10){
      if (prevAltChange >= 10){
        if (!isAirbagDeployed){
          //set airbagPin to high to let power through(transistor)
          isAirbagDeployed = true;
          digitalWrite(airbagPin, HIGH);
        }
      }
    }
    if (isAirbagDeployed){
      airbagCounter++;
      if(airbagCounter >= 320){
        digitalWrite(airbagPin, LOW);
      }
    }
    if (isFlying==true){ 
     if (altitude <= 500){
      if (altChange <= 1){
      isLanded = true;
      isFlying = false;
    }
   }
    //send all data (zAcceleration, temperature, pressure, altitude, change in altitude, airbagStatus) via radio;
    sendAllMeasurements();
    delay(750);
  }
  else if(isLanded){
    t++;
    sendClock();
    delay(750);
 }
}

