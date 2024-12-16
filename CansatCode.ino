#include <Servo.h>
#include <CanSatKit.h>
#include <floatToString.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
//#include biblioteka dla kart microSD

using namespace CanSatKit;

#define lm35Pin A0
#define diodePin 1
#define heaterPin 2

bool isFlying = false;
bool isAirbagDeployed = false;
float altitude = 0;
float prevAltitude = 0;
float zAcceleration = 0;
float altChange = 0;
float rawTemp = 0;
float voltage = 0;
int t = 0;
int heaterCounter = 0;
double temperature = 0;
double pressure = 0;

BMP280 PresSensor;
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void sendMeasurement (float data){
  char measurement [7];
  floatToString(data, measurement, sizeof(measurement), 4);
SerialUSB.println(measurement);
  // change to radio.transmit(measurement);
}

void sendClock(){
  float x = t;
 char time [7];
  floatToString(x, time, sizeof(time), 1);
SerialUSB.println(time);
  // change to radio.transmit(time);
t++;
}

void sendAllMeasurements (void){
  sendClock();
  sendMeasurement(temperature);
  sendMeasurement(pressure);
  sendMeasurement(acceleration);
  sendMeasurement(altitude);
  sendMeasurement(altChange);
  sendMeasurement(isAirbagDeployed);
}

void setup() {
  // put your setup code here, to run once:
  PresSensor.begin();
  Wire.begin();
  byte deviceID = accelerometer.readDeviceID();
  pinMode(diodePin, OUTPUT);
  pinMode(heaterPin, OUTPUT);
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
  acceler.setRange(ADXL345_RANGE_16_G);
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
  //On the ground mode, detect if acceleration is greater than 10 m/s^2
  if(!isFlying){
    prevAltitude = altitude;
    //get altitude
    if (altitude - prevAltitude >= 10){
      isFlying = true;
    }
  }
  // Flight mode, conduct all measurements and check to deploy airbag, send and record data
  else{
    t++;
    //get acceleration;
    sensors_event_t event; 
    accel.getEvent(&event);
    zAcceleration = event.acceleration.z
    //get pressure;
    PresSensor.measureTemperatureAndPressure(temperature, pressure);
    //get temperature;
    voltage = rawTemp * 5 / (std::pow(2, 12));
    temperature = 100.0 * voltage;
    //store previous altitude;
    prevAltitude = altitude;
    //use barometric formula to calculate approx altitude;
    //calculate change in altitude (altitude - previous altitude);
    altChange = altitude - prevAltitude;
    //if change in altitude <= (-)TBD and isAirbagDeployed == false, deploy airbag(power on heating); isAirbagDeployed = true;
    
    if (altChange <= 10){
      if (!isAirbagDeployed){
        //set heaterPin to high to let power through(double transistor);
        isAirbagDeployed = true;
        digitalWrite(heaterPin, HIGH);
      }
    }
    if (isAirbagDeployed){
      heaterCounter++;
      if(heaterCounter >= 320){
        digitalWrite(heaterPin, LOW);
      }
    }
    //record all data on sd card;
    //send all data (acceleration, temperature, pressure, altitude, change in altitude, airbagStatus) via radio;
    sendAllMeasurements();
    delay(750);
  }
}
