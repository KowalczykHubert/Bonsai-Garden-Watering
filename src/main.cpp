#define BLYNK_PRINT Serial

//#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <Credentials.h>
#include <BlynkSimpleEsp32.h>

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

/*
// setting PWM properties (dc motor speed settings)
const int freq = 30000;
const int pumpChannel = 0;
const int resolution = 8;
*/

const byte waterPumpPin = 23;       // water pump switch Pin
const byte waterLevelInputPin = 34; // water tank (level) pin
const byte sensorPin = 35;          // analog sensor input pin
const byte valvePins[] = {15, 2, 4, 16, 17, 5, 18, 19};              // pins for turning valves on and off
const byte soilPins[] = {13, 12, 14, 27, 26, 25, 33, 32};            // pins for turning soil sensors on and off
int airValue[] = {2700, 2700, 2700, 2700, 2700, 2700, 2700, 2700};   // soil moisture sensors calibration (to be determined experimentally)
int waterValue[] = {1200, 1200, 1200, 1200, 1200, 1200, 1200, 1200}; // soil moisture sensors calibration (to be determined experimentally)

byte sectionToWatering[sizeof(soilPins)]; // Array of valve numbers that require opening and watering
byte sS[sizeof(soilPins)];                // Array of measured humidity in all sections
byte counterToWatering;                   // Variable for the number of opened valves during watering

int sensorState;                    // soil moisture measurement variable (sensorPin)

#define numOfMeasurements 1         // Number of measurements in one loop by secion
#define measDelay 500               // delay between measurements in one section
int h_cptv_meas[numOfMeasurements]; // array of measurement results in one section

byte reconnectTimer = 60;  // Timer interval in seconds [blynk reconection]
int measuringTimer = 5;    // Timer interval in minutes [watering]
byte minHumidity = 75;     // [%] minimal soil humidity when the pump starts
byte wateringTime = 2;     // [seconds] water pump running time
byte wateringLeft = 0;     // empty water tank variable
byte maxWateringLeft = 15; // how many waterings left (sensor detect low water level)
int waterTank;             // water level variable

/*
void checkWather()
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
//--------------------------------------------------------------------------------------------------------------------------
void soilMoistureMeasurement()
{

  // ...::: SOIL MOISTURE MEASUREMENTS :::...
  counterToWatering = 0;
  for (byte i = 0; i < sizeof(soilPins); i++)    // Take as many measurements as there are sensors connected
  {
    digitalWrite(soilPins[i], HIGH);             // Turn on the power of the i-th sensor
    sensorState = 0;
    for (byte j = 0; j < numOfMeasurements; j++) // Take as many measurements on the i-th sensor as defined in the variable
    {
      delay(measDelay);                           // Tnterval between subsequent measurements within the i-th sensor
      h_cptv_meas[j] = analogRead(sensorPin);     // Save the j-th measurement to array
      sensorState += h_cptv_meas[j];              // Add the j-th measurement result to the variable
      Serial.print(h_cptv_meas[j]);
      Serial.print(" ");
    }
    digitalWrite(soilPins[i], LOW);             // Turn off the power of the i-th sensor
    sensorState /= numOfMeasurements;           // calculate the average of the measurements
    Serial.print("Average value = ");
    Serial.println(sensorState);
    sS[i] = map(sensorState, waterValue[i], airValue[i], 100, 1); // map the measurement to a range of 0 to 100

    Blynk.virtualWrite(i, sS[i]);               // Send humidity to mobile app

    if (sS[i] < minHumidity)                    // check if plants should be watered on the i-th sensor
    {
      sectionToWatering[counterToWatering] = i; // write to the array the section number that requires watering
      counterToWatering += 1;                   // increment the variable of number watering in the next step
    }

    Serial.print("Soil humidity cptv_");
    Serial.print(i + 1);
    Serial.print(" = ");
    Serial.println(sS[i]);
    Serial.println();
  }
}

void watering()
{
  // ...::: WATER LEVEL :::...
  waterTank = digitalRead(waterLevelInputPin);  // Check the water level sensor
  if (waterTank == 0)                           // Water level sensor is OK
  {
    Serial.println("The water tank is full");
    wateringLeft = maxWateringLeft;             // Set the max waterings when the sensor detect low water level
    Serial.print("Watering left more than ");
    Serial.println(wateringLeft);
    Serial.println();
  }
  else if (waterTank == 1 && wateringLeft == 0) // Water level sensor detect low water level and there's all waterings used
  {
    Serial.println("The water tank is EMPTY!");
    Blynk.notify("The water tank is EMPTY!");   // Send notofication to mobile app
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
        Serial.print("The valve " + valvePins[sectionToWatering[i]]);
        Serial.println(" is going to be open...");
        digitalWrite(valvePins[sectionToWatering[i]], HIGH); // Turn on the i-th valve as many times as the number of sections to water
        Serial.print("The soil sensor " + soilPins[sectionToWatering[i]]);
        Serial.println(" is going to measure...");
        digitalWrite(soilPins[sectionToWatering[i]], HIGH);
        byte secondsOfWatering = 0;
        do
        {
          sensorState = map(analogRead(sensorPin), waterValue[sectionToWatering[i]], airValue[sectionToWatering[i]], 100, 1);
          Serial.print("I'm watering for " + secondsOfWatering);
          Serial.print(" seconds  ");
          Serial.print("and there's " + sensorState);
          Serial.println(" % of soil humidity! Keep watering...");
          delay(500);                           // [seconds] interval between measurement
          secondsOfWatering += 0.5;
          if (secondsOfWatering == 45)          // [seconds] max time of watering
          {
            String notify1 = "Alert! There's too long watering on section:  ";
            String notify = notify1 + sectionToWatering[i];
            Blynk.notify(notify);
          break;
          }
          else if (sensorState >= 95)           // [%] if soil moisture increases to 95%, keep watering for..
          {
            delay(5000);                        // [seconds]
          }
        } while (sensorState < 95);
        digitalWrite(valvePins[sectionToWatering[i]], LOW); // Turn off the i-th valve and go to the next valve
        Serial.print("The valve " + valvePins[sectionToWatering[i]]);
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
void reconnectBlynk()
{
  if (!Blynk.connected())
  {
    WiFi.begin(ssid, pass); // Non-blocking if no WiFi available
    Serial.println("Lost connection");
    if (Blynk.connect(10000))
    {
      Serial.println("Reconnected");
    }
    else
    {
      Serial.println("Not reconnected");
    }
  }
}
void mainProgram()
{
  soilMoistureMeasurement();
  watering();
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

  pinMode(waterLevelInputPin, INPUT); // water pump switch Pin
  pinMode(waterPumpPin, OUTPUT);      // water tank (level) pin

  //...::: SETUP THE SOIL SENSOR PINS AND VALVES PINS :::...
  for (byte i = 0; i < sizeof(soilPins); i++)
  {
    pinMode(soilPins[i], OUTPUT);
  }
  for (byte i = 0; i < sizeof(valvePins); i++)
  {
    pinMode(valvePins[i], OUTPUT);
  }

  pinMode(sensorPin, INPUT); // set up analog sensor pin

  WiFi.begin(ssid, pass); // Non-blocking if no WiFi available
  Blynk.config(auth);     // Non-blocking if no WiFi available
  Blynk.connect(10000);   // Non-blocking if no WiFi available
  //Blynk.begin(auth, ssid, pass); // set up connection to internet - the code never run without internet connection

  //bme.begin(0x76);              // set up bme sensor
  timer.setInterval(measuringTimer * 60000L, mainProgram);   // Set up interwal checkPlants function call
  timer.setInterval(reconnectTimer * 1000L, reconnectBlynk); // Set up interwal checkPlants function call
  mainProgram();                                             // Call function at start
}
//--------------------------------------------------------------------------------------------------------------------------
void loop() // Main loop of the program
{

  timer.run(); // Run interwal main function (checkPlants)
  if (Blynk.connected())
  {
    Blynk.run();
  }
}
