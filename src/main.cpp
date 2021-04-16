#define BLYNK_PRINT Serial


//#include <Arduino.h>
//#include <WiFi.h>
//#include <WiFiClient.h>
#include <Credentials.h>
#include <BlynkSimpleEsp32.h>

//ACS 712 20A
#include "ACS712.h"
ACS712 battery_sensor(ACS712_20A, 39);
ACS712 load_sensor(ACS712_05B, 36);

#include "PCF8574.h"
#include <Arduino.h>

// adjust addresses if needed
#define PCF_adress 0x20
PCF8574 PCF(PCF_adress);


// Deep Sleep definitions
#define uS_TO_S_FACTOR 60000000  /* Conversion factor for micro seconds to minutes */
#define TIME_TO_SLEEP  20        /* Time ESP32 will go to sleep (in minues) */

RTC_DATA_ATTR int bootCount = 0; // Can save some data to memory


#include <OneWire.h>
#include <DallasTemperature.h>
// Data wire is plugged into pin ...
#define ONE_WIRE_BUS 13 //był 21
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);
// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

DeviceAddress DBsensor1 = {0x28, 0xD8, 0xB7, 0x16, 0xA8, 0x1, 0x3C, 0x5E};
DeviceAddress DBsensor2 = {0x28, 0x5A, 0x48, 0x16, 0xA8, 0x1, 0x3C, 0x13};
DeviceAddress DBsensor3 = {0x28, 0xF9, 0x87, 0x16, 0xA8, 0x1, 0x3C, 0x5B};
DeviceAddress DBsensor4 = {0x28, 0x83, 0x2E, 0x16, 0xA8, 0x1, 0x3C, 0xCC};
/*
float t_db1;
float t_db2;
float t_db3;
float t_db4;
*/

/*
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>

#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C Create an object of Adafruit_BME280 class
float t_bme;         // Declare the temperature variable (bme sensor)
float h_bme;         // Declare the humidity variable (bme sensor)
float p_bme;         // Declare the pressure variable (bme sensor)
//float alt_bme;      // Declare the altitude variable (bme sensor)
*/

// MOBILE APP CONFIG
BlynkTimer timer;       // Create an object of BlynkTimer class
char auth[] = APIKEY;   // Auth Token in the Blynk App.
char ssid[] = SSIDHOME; // WiFi credentials (Credentails.h)
char pass[] = PASSHOME; // WiFi credentials (Credentails.h)

// PCF8574 pins to virtual pins

 BLYNK_WRITE (V15) // V0 is the number of Virtual Pin For Read From BLYNK APP
  {
  int pinValue = param.asInt ();

  PCF.digitalWrite (0, pinValue);
  Serial.print ("pinValue0 =");
  Serial.println (pinValue);
  }

/*
// setting PWM properties (dc motor speed settings)
const int freq = 30000;
const int pumpChannel = 0;
const int resolution = 8;
*/

const byte voltageBatteryPin = 33;
const byte lightSensorPin = 32; // pin for wiring GND photoresistor
const byte waterPumpPin = 23;                                            // water pump switch Pin
const byte waterLevelInputPin = 34;                                      // water tank (level) pin
const byte moistureSensorPin = 35;                                               // analog sensor input pin
const byte valvePins[] = {15, 2, 4, 16, 17, 5, 18, 19};                  // pins for turning valves on and off {15, 2, 4, 16, 17, 5, 18, 19};
const byte soilPins[] = {0,1,2,3,4,5,6,7};                         // PCF8574 pins for soil moisture sensors turning on / off 
//const byte soilPins[] = {13, 12, 14, 27, 26, 25, 33, 32};                // ESP32 pins {13, 12, 14, 27, 26, 25, 33, 32};
const int airValue[] = {1700, 1700, 1660, 1550, 1450, 1400, 1550, 1500}; // soil moisture sensors calibration (to be determined experimentally)
const int waterValue[] = {1050, 840, 900, 800, 730, 1000, 780, 820};     // soil moisture sensors calibration (to be determined experimentally)

byte sectionToWatering[sizeof(soilPins)]; // Array of valve numbers that require opening and watering
int sS[sizeof(soilPins)];                 // Array of measured humidity in all sections
byte counterToWatering;                   // Variable for the number of opened valves during watering

int sensorState = 0; // soil moisture measurement variable (sensorPin)

#define numOfMeasurements 3                                 // Number of measurements in one loop by secion
#define measDelay 2000                                     // delay between measurements in one section
float sensorDelay = (measDelay * numOfMeasurements) + 1000; // delay between turing on the next sensor
int h_cptv_meas[numOfMeasurements];                         // array of measurement results in one section


float measurementTime = (sensorDelay / 1000) * sizeof(soilPins) / 60;
//float mainTimer = measurementTime + 1; // Timer interval in minutes [soil moisture] delay between measurements MINIMAL !!!! 0.15 = 9 seconds
byte minHumidity = 75;                   // [%] minimal soil humidity when the pump starts
byte wateringLeft = 0;                   // empty water tank variable
byte maxWateringLeft = 15;               // how many waterings left (sensor detect low water level)
int waterTank;                           // water level variable

/*
void checkWeather()
{
  //     ...::: BME280 :::...
  t_bme = bme.readTemperature();                  // Temperature measurement
  h_bme = bme.readHumidity();                     // Air Humidity measurement
  p_bme = (bme.readPressure() / 100.0F);          // Pressure measurement
  //float Alt = bme.readAltitude(SEALEVELPRESSURE_HPA); //Altitude measurement

  //Blynk.virtualWrite(V8, t_bme);                // Send temperature to mobile app
  //Blynk.virtualWrite(V9, h_bme);                // Send humidity to mobile app
  //Blynk.virtualWrite(V10, p_bme);               // Send pressure to mobile app

  Serial.println();
  Serial.println(".......BME280......");
  Serial.print("Temperature = ");
  Serial.print(t_bme);
  Serial.println(" °C");
  Serial.print("Humidity = ");
  Serial.print(h_bme);
  Serial.println(" %");
  Serial.print("Pressure = ");
  Serial.print(p_bme);
  Serial.println(" hPa");
  //Serial.print("Approx. Altitude = ");
  //Serial.print(alt_bme);
  //Serial.println(" m");
  Serial.println();
  Serial.println();
}
*/
//----------------------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------------------
const int lightIntensityLimit = 20; // do ustawienia eksperymentalnie
const int dayLightIntensity = 4096; // ADC value for day
const int nightLightIntensity = 0; // ADC value for night
int lightIntensity;
byte isDay = 0;
WidgetLCD lcd(V1);

void isDayOrNight()
{
  lcd.clear();
  lcd.print(0,1,"Wstałem");
  lightIntensity = map(analogRead(lightSensorPin), dayLightIntensity, nightLightIntensity, 100, 0);
  Serial.print("Natężenie światła: ");
  Serial.print(lightIntensity);
  Blynk.virtualWrite(V51, lightIntensity);
  Serial.println(" %");
  if (lightIntensity > lightIntensityLimit)
  {
    Serial.println("Pracuję cały czas, ponieważ mamy piękny dzień!");
    lcd.print(0,0, "Pracuje...");
    isDay = 1;
  }
  else
  {
    if (isDay == 1)
    {
      Serial.println("Wygląda na to, że zrobiło się już ciemno, więc idę spać :)");
      lcd.print(0,0,  "Robi sie ciemno...");
    }
    else
    {
      Serial.println("Wygląda na to, że nadal mamy noc, więc idę spać dalej :)");
      lcd.print(0,0, "Nadal ciemno...");
    }
  Serial.flush();
  Serial.println("ZzZzZ...");
  lcd.print(0,1, "Ide spac...");
  sleep(500);
  esp_deep_sleep_start();
  Serial.println("Tego już nie napiszę ponieważ zasnąłęm...");
  }
}

//----------------------------------------------------------------------------------------------------------------------
byte i; // counter for sensors and valves
byte j; // counter for measurements

void avgMeas()
{
  h_cptv_meas[j] = analogRead(moistureSensorPin);
  Serial.print(h_cptv_meas[j]);
  Serial.print(" ");
  Serial.print("zrobiłem pomiar nr : ");
  Serial.println(j + 1);
  sensorState += h_cptv_meas[j]; // Add the j-th measurement result to the variable

  if (j == numOfMeasurements - 1)
    {
      Serial.print("wyłączyłem czujnik nr : ");
      Serial.println(i+1);
      PCF.digitalWrite(soilPins[i], LOW);
      
      sensorState /= numOfMeasurements; // calculate the average of the measurements
      Serial.print("Average value = ");
      Serial.println(sensorState);
      sS[i] = map(sensorState, waterValue[i], airValue[i], 100, 1); // map the measurement to a range of 0 to 100

      Serial.print("Soil humidity cptv_");
      Serial.print(i + 1);
      Serial.print(" = ");
      Serial.println(sS[i]);
      Serial.println();
      Blynk.virtualWrite(i, sS[i]); // Send humidity to mobile app

      if (sS[i] < minHumidity) // check if plants should be watered on the i-th sensor
      {
        sectionToWatering[counterToWatering] = i; // write to the array the section number that requires watering
        counterToWatering += 1;                   // increment the variable of number watering in the next step
      }
      sensorState = 0;
      j = 0;
      i++;
      if (i==sizeof(soilPins))
      {
        isDayOrNight();
      }
    }
  else
    {
      j++;
    }
}

void sensorOn()
{
  PCF.digitalWrite(soilPins[i], HIGH);
  Serial.print("włączyłem czujnik nr : ");
  Serial.println(i + 1);
  // if (millis() - previousMillis > measDelay)
  //{
  //unsigned long previousMillis = millis();
  timer.setTimer(measDelay, avgMeas, numOfMeasurements);
  //}
}

void moistureMeasurement()
{
  counterToWatering = 0;
  i=0;
  j=0;
  timer.setTimer(sensorDelay, sensorOn, sizeof(soilPins));
}

//--------------------------------------------------------------------------------------------------------------------------
void soilMoistureMeasurement()
{

  // ...::: SOIL MOISTURE MEASUREMENTS :::...
  counterToWatering = 0;
  for (byte i = 0; i < sizeof(soilPins); i++) // Take as many measurements as there are sensors connected
  {
    digitalWrite(soilPins[i], HIGH); // Turn on the power of the i-th sensor
                                     //delay (3000);
    sensorState = 0;
    for (byte j = 0; j < numOfMeasurements; j++) // Take as many measurements on the j-th sensor as defined in the variable
    {
      delay(measDelay);                       //Tnterval between subsequent measurements within the i-th sensor
      h_cptv_meas[j] = analogRead(moistureSensorPin); // Save the j-th measurement to array
      sensorState += h_cptv_meas[j];          // Add the j-th measurement result to the variable
      Serial.print(h_cptv_meas[j]);
      Serial.print(" ");
    }
    digitalWrite(soilPins[i], LOW);   // Turn off the power of the i-th sensor
    sensorState /= numOfMeasurements; // calculate the average of the measurements
    Serial.print("Average value = ");
    Serial.println(sensorState);
    sS[i] = map(sensorState, waterValue[i], airValue[i], 100, 1); // map the measurement to a range of 0 to 100

    Serial.print("Soil humidity cptv_");
    Serial.print(i + 1);
    Serial.print(" = ");
    Serial.println(sS[i]);
    Serial.println();
    Blynk.virtualWrite(i, sS[i]); // Send humidity to mobile app

    if (sS[i] < minHumidity) // check if plants should be watered on the i-th sensor
    {
      sectionToWatering[counterToWatering] = i; // write to the array the section number that requires watering
      counterToWatering += 1;                   // increment the variable of number watering in the next step
    }
  }
}
//--------------------------------------------------------------------------------------------------------------------------
void watering()
{
  // ...::: WATER LEVEL :::...
  waterTank = digitalRead(waterLevelInputPin); // Check the water level sensor
  if (waterTank == 0)                          // Water level sensor is OK
  {
    Serial.println("The water tank is full");
    wateringLeft = maxWateringLeft; // Set the max waterings when the sensor detect low water level
    Serial.print("Watering left more than ");
    Serial.println(wateringLeft);
    Serial.println();
  }
  else if (waterTank == 1 && wateringLeft == 0) // Water level sensor detect low water level and there's all waterings used
  {
    Serial.println("The water tank is EMPTY!");
    Blynk.notify("The water tank is EMPTY!"); // Send notofication to mobile app
  }
  // ...::: WATERING :::...

  if (counterToWatering > 0 && wateringLeft > 0) // When there's some section to watering and there's some water in the tank
  {
    digitalWrite(waterPumpPin, HIGH); // Turn on the water pump
    Serial.println("The pump is starting...");
    /*
      digitalWrite(valvePins[sectionToWatering[0]], HIGH);
      for (int fadeValue = 100 ; fadeValue <= 245; fadeValue += 5) 
      {
      ledcWrite(pumpChannel, fadeValue);
      delay(100);
      if (fadeValue == 245) 
        {
        fadeValue = 255;
        break;
        }
      }
      */
    for (byte i = 0; i < counterToWatering; i++) // Valves to turn on as many as the number of sections to water
    {
      Serial.print("The valve ");
      Serial.print(valvePins[sectionToWatering[i]]);
      Serial.println(" is going to be open...");
      digitalWrite(valvePins[sectionToWatering[i]], HIGH); // Turn on the i-th valve as many times as the number of sections to water
      Serial.print("The soil sensor ");
      Serial.print(soilPins[sectionToWatering[i]]);
      Serial.println(" is going to measure...");
      digitalWrite(soilPins[sectionToWatering[i]], HIGH);
      byte maxWateringTime = 0;
      do
      {
        sensorState = map(analogRead(moistureSensorPin), waterValue[sectionToWatering[i]], airValue[sectionToWatering[i]], 100, 1);
        Serial.print("I'm watering for ");
        Serial.print(maxWateringTime);
        Serial.print(" seconds  ");
        Serial.print("and there's ");
        Serial.print(sensorState);
        Serial.println(" % of soil humidity! Keep watering...");
        delay(500); // [seconds] interval between measurement
        maxWateringTime += 0.5;
        if (maxWateringTime == 60) // [seconds] max time of watering
        {
          String notify1 = "Alert! There's too long watering on section:  ";
          String notify = notify1 + sectionToWatering[i];
          Blynk.notify(notify);
          break;
        }
        else if (sensorState >= 95) // [%] if soil moisture increases to 95%, keep watering for..
        {
          delay(5000); // [seconds]
        }
      } while (sensorState < 95);
      digitalWrite(valvePins[sectionToWatering[i]], LOW); // Turn off the i-th valve and go to the next valve
      Serial.print("The valve ");
      Serial.print(valvePins[sectionToWatering[i]]);
      Serial.println(" is closed...");

      /*
        Serial.print("The valve " + valvePins[sectionToWatering[i]]);
        Serial.println(" is going to be open...");
        digitalWrite(valvePins[sectionToWatering[i]], HIGH);
        delay(1000 * wateringTime);                         // Keep the i-th valve on for wateringTime seconds
        digitalWrite(valvePins[sectionToWatering[i]], LOW); // Turn off the i-th valve and go to the next valve
        */
    }

    //ledcWrite(pumpChannel, 0);
    digitalWrite(waterPumpPin, LOW); // When all required sections are watered then turn off the water pump
    Serial.println("The pump has stopped working...");
  }

  // ...::: UPDATE WATERTANK STATUS :::...
  if (waterTank == 1 && wateringLeft > 0) // After watering the water level sensor is NOK and the tank is not empty
  {
    Serial.println("The water is running out!");
    Serial.print("Watering left: ");
    Serial.println(wateringLeft);
    Serial.println();
    if (wateringLeft % 2 == 0) // Every second watering
    {
      String notify1 = "Alert! ";
      String notify2 = " waterings left";
      String notify = notify1 + wateringLeft + notify2;
      Blynk.notify(notify); // Send the notification about running out of water to mobile app
    }
    wateringLeft--; // Decrement amount of watering left
  }
}

//--------------------------------------------------------------------------------------------------------------------------
byte reconnectTimer = 5; // Timer interval in seconds [blynk reconection]
unsigned long previousMillis = millis();
unsigned long currentMillis = millis();
float secDC = 0;
float secC = 0;

void reconnectBlynk()
{
  if (!Blynk.connected())
  {
    
    WiFi.begin(ssid, pass); // Non-blocking if no WiFi available
    Serial.println("Lost connection");
    if (Blynk.connect(5000))
    {
      Serial.println("Reconnected");
      Serial.print("Nie było neta przez: ");
      Serial.println(secDC);
    }
    else
    {
      Serial.println("Not reconnected");
    }
  }
}
//-----------------------------------------------------------------------------------
//---------------------------------------------------

void ifConnected()
{
  if (!Blynk.connected())
    {
      currentMillis = millis();
      secDC += ((currentMillis - previousMillis)/1000);
      
    }
    previousMillis = millis();
  Blynk.virtualWrite(V26,(secDC/60));
  secC += 1;
  Blynk.virtualWrite(V25, (secC/60));
}

//-----------------------------------------------------------------------------------

float battery_sensor_I;
float battery_sensor_calibrating = 7.595;

float load_sensor_I;
float load_sensor_calibrating = 4.104;

const int numReadings = 5;
int measCurrentDelay = 50;
const byte numReadingsAux = 15;

float battery_readings[numReadings];      // the readings from the analog input
float load_readings[numReadings];

float battery_readingsAux[numReadingsAux];
float load_readingsAux[numReadingsAux];

byte readIndex = 0;
byte readIndexAux = 0;
float battery_totalAux = 0;             // the index of the current reading
float load_totalAux = 0;
float battery_total = 0;                 // the running total
float load_total = 0;
float battery_average = 0;                // the average
float load_average = 0;

float battery_averageAux = 0;
float load_averageAux = 0;

void avgCurrentMeas()
{
  battery_totalAux = battery_totalAux - battery_readingsAux[readIndexAux];
  load_totalAux = load_totalAux - load_readingsAux[readIndexAux];

  battery_readingsAux[readIndexAux] = battery_sensor.getCurrentDC() - battery_sensor_calibrating;
  load_readingsAux[readIndexAux] = load_sensor.getCurrentDC() - load_sensor_calibrating;
  
   // add the reading to the total:
  battery_totalAux = battery_totalAux + battery_readingsAux[readIndexAux];
  load_totalAux = load_totalAux + load_readingsAux[readIndexAux];

  // advance to the next position in the array:
  readIndexAux = readIndexAux + 1;

  // if we're at the end of the array...
  if (readIndexAux >= numReadingsAux) {
    // ...wrap around to the beginning:
    readIndexAux = 0;
  }
     // calculate the average:
  battery_averageAux = battery_totalAux / numReadingsAux;
  load_averageAux = load_totalAux / numReadingsAux;
}

void currentMeasurement() // do uzupełnienia
{
	load_sensor_I = load_sensor.getCurrentDC()-load_sensor_calibrating; 
  battery_sensor_I = battery_sensor.getCurrentDC()-battery_sensor_calibrating;

  // Send it to serial
  Serial.println(String("I = ") + battery_averageAux + " mA");  //battery_sensor_I
  Blynk.virtualWrite(V45, battery_averageAux);                  //battery_sensor_I
  Serial.println(String("I = ") + load_averageAux + " mA");     //load_sensor_I
  Blynk.virtualWrite(V46, load_averageAux);                     //load_sensor_I


  // Average from a few last measurements
  timer.setTimer(measCurrentDelay, avgCurrentMeas, numReadingsAux);

  battery_total = battery_total - battery_readings[readIndex];
  load_total = load_total - load_readings[readIndex];

  // read from the sensor:
  battery_readings[readIndex] = battery_averageAux;
  load_readings[readIndex] = load_averageAux;

  // add the reading to the total:
  battery_total = battery_total + battery_readings[readIndex];
  load_total = load_total + load_readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  battery_average = battery_total / numReadings;
  load_average = load_total / numReadings;
  // send it to the computer as ASCII digits
  Serial.print("Średnia battery z nowej metody = ");
  Serial.println(battery_average);
  Serial.print("Średnia load z nowej metody = ");
  Serial.println(load_average);
  Serial.println();
  
  Blynk.virtualWrite(V47, battery_average);
  Blynk.virtualWrite(V48, load_average);
  
}
//--------------------------------------------------------------------------------------------------------------------------
void getTemp()
  {
    float t_db1 = sensors.getTempC(DBsensor1);
    Serial.print("Sensor 1(*C): ");
    Serial.print(t_db1);

    float t_db2 = sensors.getTempC(DBsensor2);
    Serial.print("Sensor 2(*C): ");
    Serial.print(t_db2);

    float t_db3 = sensors.getTempC(DBsensor3);
    Serial.print("Sensor 3(*C): ");
    Serial.print(t_db3);

    float t_db4 = sensors.getTempC(DBsensor4);
    Serial.print("Sensor 3(*C): ");
    Serial.print(t_db4);
    Serial.println();

    Blynk.virtualWrite(V21, t_db1);
    Blynk.virtualWrite(V22, t_db2);
    Blynk.virtualWrite(V23, t_db3);
    Blynk.virtualWrite(V24, t_db4);
  }

void temperatureMeasurement()
{
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  float temperatureC = sensors.getTempCByIndex(0);
  Serial.print(temperatureC); //jhh
  Serial.println("ºC"); //kk
  Serial.println("DONE");
  timer.setTimeout(500, getTemp);
  //delay(250);
}

//--------------------------------------------------------------------------------------------------------------------------
int battery_meas;
float battery_voltage;
float voltage_ratio = 6.13;
float voltage_calibration = 0.615;
float battery_voltage_totalAux = 0;
float battery_voltage_readingsAux[numReadingsAux];
byte voltage_readIndexAux = 0;
byte voltage_readIndex = 0;
float battery_voltage_averageAux = 0;
float battery_voltage_total = 0;
float battery_voltage_readings[numReadings];
float battery_voltage_average = 0;

void avgVoltageMeas()
{
  battery_voltage_totalAux = battery_voltage_totalAux - battery_voltage_readingsAux[voltage_readIndexAux];
  
  battery_voltage_readingsAux[voltage_readIndexAux] = (((analogRead(voltageBatteryPin)/4095.0) * 3.3) * voltage_ratio) + voltage_calibration;
    
   // add the reading to the total:
  battery_voltage_totalAux = battery_voltage_totalAux + battery_voltage_readingsAux[voltage_readIndexAux];

  // advance to the next position in the array:
  voltage_readIndexAux = voltage_readIndexAux + 1;

  // if we're at the end of the array...
  if (voltage_readIndexAux >= numReadingsAux) {
    // ...wrap around to the beginning:
    voltage_readIndexAux = 0;
  }
     // calculate the average:
  battery_voltage_averageAux = battery_voltage_totalAux / numReadingsAux;
}


void voltageMeasurement()
{
    
  battery_meas = analogRead(voltageBatteryPin);
  battery_voltage = (((battery_meas/4095.0) * 3.3) * voltage_ratio) + voltage_calibration;
  //((battery_meas * 3.2) / 4095) * ((4700 + 909.0909090909)/909.090909090);
  //battery_meas*(3.3/4096);
  //(battery_meas/(4096 * 20.361));
  //((battery_meas * 3.3) / 4095) * ((4700 + 909.0909090909)/909.090909090);

  //Serial.println(battery_voltage);
  Blynk.virtualWrite(V49, battery_voltage);

  timer.setTimer(measCurrentDelay, avgVoltageMeas, numReadingsAux);


  battery_voltage_total = battery_voltage_total - battery_voltage_readings[voltage_readIndex];
  
  battery_voltage_readings[voltage_readIndex] = battery_voltage_averageAux; //+ voltage_calibration;
    
   // add the reading to the total:
  battery_voltage_total = battery_voltage_total + battery_voltage_readings[voltage_readIndex];

  // advance to the next position in the array:
  voltage_readIndex = voltage_readIndex + 1;

  // if we're at the end of the array...
  if (voltage_readIndex >= numReadings) {
    // ...wrap around to the beginning:
    voltage_readIndex = 0;
  }
     // calculate the average:
  battery_voltage_average = battery_voltage_total / numReadings;
  Blynk.virtualWrite(V50, battery_voltage_average);

 }
//--------------------------------------------------------------------------------------------------------------------------
void mainProgram()
{
  //soilMoistureMeasurement(); / delays
  //watering(); // delays
  
  //temperatureMeasurement(); // timers
  //moistureMeasurement(); // timers
  //isDayOrNight();
}
//--------------------------------------------------------------------------------------------------------------------------
void setup()
{
  Serial.begin(9600); // Debug console
  /*
    // configure LED PWM functionalitites
    ledcSetup(pumpChannel, freq, resolution);
    // attach the channel to the GPIO to be controlled
    ledcAttachPin(waterPumpPin, pumpChannel);
    */

  // Light sensor INIT
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR); // Set ESP to sleep for 30 minutes

// currentMeasurement array values set to 0
for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    battery_readings[thisReading] = 0;
    load_readings[thisReading] = 0;
  }
for (int thisReading = 0; thisReading < numReadingsAux; thisReading++) {
    battery_readingsAux[thisReading] = 0;
    load_readingsAux[thisReading] = 0;
  }

  // PCF8574 setup
   Serial.print("Init pcf8574...");
	if (PCF.begin()){
		Serial.println("OK");
	}else{
		Serial.println("KO");
	}
  for (byte i = 0; i < sizeof(soilPins); i++)
  {
    PCF.pinMode(soilPins[i], OUTPUT);
  }


  //ACS712 setup
  //Serial.println("Calibrating... Ensure that no current flows through the sensor at this moment");
  //int zero = sensor.calibrate();
  //Serial.println("Done!");
  //Serial.println("Zero point for this sensor = " + zero);
   
  // ESP pins setup
  pinMode(waterLevelInputPin, INPUT); // water pump switch Pin
  pinMode(waterPumpPin, OUTPUT);      // water tank (level) pin

  //...::: SETUP THE SOIL SENSOR PINS AND VALVES PINS :::...
  
  
  for (byte i = 0; i < sizeof(valvePins); i++)
  {
    pinMode(valvePins[i], OUTPUT);
  }
  
  pinMode(moistureSensorPin, INPUT); // set up analog sensor pin
  pinMode(lightSensorPin, INPUT);
  pinMode(voltageBatteryPin, INPUT);

  sensors.begin();        // DS18B20s
  WiFi.begin(ssid, pass); // Non-blocking if no WiFi available
  Blynk.config(auth);     // Non-blocking if no WiFi available
  Blynk.connect(5000);   // Non-blocking if no WiFi available
  //Blynk.begin(auth, ssid, pass); // set up connection to internet - the code never run without internet connection

  //bme.begin(0x76);              // set up bme sensor
  //timer.setInterval(measurementTime + 1 * 60000L, mainProgram);        // Set up interwal mainProgram [minutes]. Min value: 0.15 = 9 seconds

  timer.setInterval( 0.5 * 60000L, temperatureMeasurement);  // Set up interval of temperatureMeasurement [minutes]. DS18B20 x5
  //timer.setInterval( 1 + measurementTime * 60000L, moistureMeasurement);  // Set up interwal mosistureMeasurement [minutes]. Min value: 0.15 = 9 seconds
  timer.setInterval( 5 * 1000L, reconnectBlynk); // Set up interwal reconnect function call [seconds]
  timer.setInterval( 1 * 1000L, ifConnected);                       // Check for every 1 seconds if blynk is connected to internet
  timer.setInterval( 1 * 60000L, isDayOrNight);                      // Check for every XX seconds if is day or night
  timer.setInterval( 1 * 1000L, currentMeasurement);
  timer.setInterval( 1 * 1000L, voltageMeasurement);

  
  //temperatureMeasurement();
  //moistureMeasurement();

  //mainProgram();                                             // Call function at start
}
//--------------------------------------------------------------------------------------------------------------------------
void loop() // Main loop of the program
{

  timer.run(); // Run interwal main function (mainProgram)
  if (Blynk.connected())
  {
    Blynk.run();
  }
}
