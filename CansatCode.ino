#include <CanSatKit.h>
#include <floatToString.h>
#include <SD.h>
#include <SPI.h>

using namespace CanSatKit;

#define externalLM35Pin A0
#define airbagPin 5 //digital 3 is likely broken, no clue why

bool isFlying = true;
bool isAirbagDeployed = false;
bool isLanded = false;
bool isLaunched = false;
float altitude = 0;
float prevAltitude = 0;
float zAcceleration = 0;
float altChange = 0;
float prevAltChange = 0;
float rawTemp = 0;
float voltage = 0;
float temperature = 0;
int t = 0;
int airbagCounter = 0;
double te, pressure, groundPressure;

const int chipSelect = 11;

const char filename[] = "datalog.txt";
// File object to represent file
File dataFile;
// string to buffer output
String dataBuffer;
// last time data was written to card:
unsigned long lastMillis = 0;

Frame frame;

BMP280 PresSensor;


Radio radio(Pins::Radio::ChipSelect,
            Pins::Radio::DIO0,
            433.0,
            Bandwidth_125000_Hz,
            SpreadingFactor_9,
            CodingRate_4_8);

void sendMeasurement (float data){
  char measurement [7];
  floatToString(data, measurement, sizeof(measurement), 2);
  SerialUSB.println(measurement);
  frame.println(measurement);
  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    SerialUSB.print("error opening ");
    SerialUSB.println(filename);
    //while (true);
  }
  if (dataFile) {
    dataFile.println(measurement);
    SerialUSB.println(measurement);
    dataFile.close();
  }
}

void sendClock(){
  float x = t;
  char time [5];
  floatToString(x, time, sizeof(time), 0);
  SerialUSB.println(time);
  frame.println(time);
  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    SerialUSB.print("error opening ");
    SerialUSB.println(filename);
    //while (true);
  }
  if (dataFile) {
    dataFile.println(time);
    SerialUSB.println(time);
    dataFile.close();
  }
  t++;
}

void sendAllMeasurements(){
  sendClock();
  frame.print("temp:");
  sendMeasurement(temperature);
  frame.print("pressure:");
  sendMeasurement(pressure);
  frame.print("alt:");
  sendMeasurement(altitude);
  frame.print("altChange:");
  sendMeasurement(altChange);
  if (isAirbagDeployed == true){
    frame.println("airbag is out");
  }
  else{
    frame.println("airbag not out");
  }
  //calculates a control sum so that it may be cross refrenced to detect data corruption in transmission
  int ctrlSum = altChange + temperature + pressure + altitude;
  sendMeasurement(ctrlSum);
  radio.transmit(frame);
  frame.clear();

}

float getExternalTemperature(int raw) {
  voltage = raw * 3.3 / (std::pow(2, 12));
  temperature = 100.0 * voltage;
  return temperature;
}

void setup() {
  //safety delay for code upload
  delay(1500);
  SD.begin(chipSelect);
  PresSensor.begin();
  // put your setup code here, to run once:
  analogReadResolution(12);
  SerialUSB.begin(115200);
  dataBuffer.reserve(1024);
  //radio.begin();
  PresSensor.begin();
  pinMode(airbagPin, OUTPUT);
  //Only for testing:
  if(!PresSensor.begin()){
    SerialUSB.println("BMP280 init failed!");
  }
  else{
    SerialUSB.println("BMP280 init succesful");
  }
   PresSensor.setOversampling(16);
   PresSensor.measureTemperatureAndPressure(te, groundPressure);
}

void loop() {
  // put your main code here, to run repeatedly:
  //safety delay for code upload
  if(t == 0){
    delay(1500);
    t = 1;
  }
  // Flight mode, conduct all measurements and check to deploy airbag, send and record data
  if(isFlying){
    //get pressure;
    PresSensor.measureTemperatureAndPressure(te, pressure);
    //get external temperature;
    rawTemp = analogRead(externalLM35Pin);
    getExternalTemperature(rawTemp);
    //store previous altitude;
    prevAltitude = altitude;
    //use formula from wikipedia to calculate approx altitude;
    altitude = (groundPressure - pressure) / 0.12;
    //calculate change in altitude (altitude - previous altitude);
    prevAltChange = altChange;
    altChange = (altitude - prevAltitude);
    //to stop any misfire of the airbag on the ground
    if(altitude >= 200 && prevAltitude >= 200){
      isLaunched = true;
    }
    //if change in altitude <= (-)TBD and isAirbagDeployed == false, deploy airbag(power on heating); isAirbagDeployed = true; 
    if (isLaunched){
      if (prevAltChange <= -5 && altChange <= -5){
        if (altitude >= 1000){
          if (!isAirbagDeployed){
            //set airbagPin to high to let power through(transistor)
             isAirbagDeployed = true;
            digitalWrite(airbagPin, HIGH);
            SerialUSB.println("Airbag deployed");
            }
          }
        }  
      }
    if (isAirbagDeployed){
      airbagCounter++;
      if(airbagCounter >= 50){
        digitalWrite(airbagPin, LOW);
      }
    }
     /*if (altitude <= 500){
      if (altChange <= 1, altChange >= -1){
      isLanded = true;
      isFlying = false;
    }*/
  
    //send all data (temperature, pressure, altitude, change in altitude, airbagStatus) via radio;
    sendAllMeasurements();
    delay(750);
  }
   else if(isLanded){
    sendClock();
    delay(750);
  }
}
