#include <CanSatKit.h>
#include <floatToString.h>
//#include <Wire.h>
//#include <Adafruit_Sensor.h>
//#include <Adafruit_ADXL345_U.h>
#include <SPI.h>
#include <SD.h>
#include <cmath>

using namespace CanSatKit;

#define externalLM35Pin A0
#define airbagPin 5 //digital 3 is likely broken, no clue why

bool isFlying = true;
bool isAirbagDeployed = false;
bool isLanded = false;
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

//const char filename[] = "datalog.txt";
// File object to represent file
//File dataFile;
// string to buffer output
//String dataBuffer;
// last time data was written to card:
//unsigned long lastMillis = 0;

Frame frame;
BMP280 PresSensor;
//Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

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
  /*dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (!dataFile) {
    SerialUSB.print("error opening ");
    SerialUSB.println(filename);
    while (true);
  }*/
  /*if (dataFile) {
    dataFile.println(measurement);
    dataFile.close();
  }*/
}

void sendClock(){
  float x = t;
  char time [5];
  floatToString(x, time, sizeof(time), 0);
  SerialUSB.println(time);
  frame.println(time);
  t++;
}

void sendAllMeasurements(){
  sendClock();
  frame.print("temp:");
  sendMeasurement(temperature);
  frame.print("pressure:");
  sendMeasurement(pressure);
  //frame.println("acceleration: ");
  //sendMeasurement(zAcceleration);
  frame.print("alt:");
  sendMeasurement(altitude);
  frame.print("altChange:");
  sendMeasurement(altChange);
  if (isAirbagDeployed == true){
    radio.transmit("airbag is out");
  }
  else{
    radio.transmit("airbag not out");
  }
  radio.transmit(frame);
  frame.clear();

}

float getExternalTemperature(int raw) {
  voltage = raw * 3.3 / (std::pow(2, 10));
  temperature = 100.0 * voltage;
  return temperature;
}

void setup() {
  PresSensor.begin();
  // put your setup code here, to run once:
  dataBuffer.reserve(1024);
  analogReadResolution(12);
  SerialUSB.begin(115200);
  /*dataBuffer.reserve(1024);
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (!dataFile) {
    SerialUSB.print("error opening ");
    SerialUSB.println(filename);
    while (true);
  }*/  
  radio.begin();
  PresSensor.begin();
  //Wire.begin();
  //byte deviceID = accel.getDeviceID();
  pinMode(airbagPin, OUTPUT);
  //#ifndef ESP8266
  //while (!SerialUSB); // for Leonardo/Micro/Zero
  //#endif
  /* Initialise the sensor */
  /*if(!accel.begin())
  {
    // There was a problem detecting the ADXL345 ... check your connections
    SerialUSB.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }*/

  /* Set the range to whatever is appropriate for your project */
  //accel.setRange(ADXL345_RANGE_16_G);
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
  // Flight mode, conduct all measurements and check to deploy airbag, send and record data
  if(isFlying){
    //get acceleration;
    //sensors_event_t event; 
    //accel.getEvent(&event);
    //zAcceleration = event.acceleration.z;
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
    //if change in altitude <= (-)TBD and isAirbagDeployed == false, deploy airbag(power on heating); isAirbagDeployed = true; 
    //if (altChange <= -10){
      //if (prevAltChange <= -10){
        //if (altitude >= 1000){
    if(t >= 10){ 
          if (!isAirbagDeployed){
            //set airbagPin to high to let power through(transistor)
             isAirbagDeployed = true;
            digitalWrite(airbagPin, HIGH);
            SerialUSB.println("Airbag deployed");
          }
          //}
        //}  
      //}
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
    }
   }*/
    //send all data (zAcceleration, temperature, pressure, altitude, change in altitude, airbagStatus) via radio;
    sendAllMeasurements();
    delay(750);
 }
   else if(isLanded){
    //sendClock();
    delay(750);
  }
}
