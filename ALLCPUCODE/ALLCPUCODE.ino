/* Authors: 
 * Fall 2020
 */
//#include stuff
#include <Wire.h>
#include <SparkFun_TMP117.h> //http://librarymanager/All#SparkFun_TMP117
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
#include <cmath> as std
#include <SPI.h>
#include <SD.h>
#include "FS.h"
#include <WiFi.h>
#include <WebServer.h>


//Pin definitions
#define HEATER_OUTPUT 16
#define I2C_SDA 21
#define I2C_SCL 22
#define SD_CS 14
#define BATT_1 34 //21V nominal
#define BATT_2 35 //4.2V nominal
#define BATT_3 36 //4.2V nominal

//other configuration option definitions
#define MIN_TEMP_SINGLE 10.0 //degrees celsius
#define MIN_TEMP_AVG 18.0
#define MAX_TEMP_SINGLE 20.0
#define MAX_TEMP_AVG 22.0
#define HEATER_CHECK_TIME 500 //in milliseconds
#define GPS_CHECK_TIME 5000 //Also affects frequency of mcu/gps time logging
#define DISPLACEMENT_AVG_FRAME 10
#define DESCENT_RATE 10 //in meters per second
#define TIME_UNTIL_CUTDOWN 21600000 //How long until time-delay cutdown is triggered, in ms
#define BATTERY_CHECK_TIME 500 //in milliseconds

//Global variables
TMP117 sensor1;
TMP117 sensor2;
TMP117 sensor3;
TMP117 sensor4;
SFE_UBLOX_GPS myGPS;

boolean launched; //This can be set to true through the web terminal right before the balloon is launched.  Ensures cutdown is not triggered while waiting on GPS fix.
float temp1, temp2, temp3, temp4;
float latitude, longitude, altitude, lastLatitude, lastLongitude;
byte SIV;
float pastLatitudeDisplacement[DISPLACEMENT_AVG_FRAME];
float pastLongitudeDisplacement[DISPLACEMENT_AVG_FRAME];
int avgFramePos = 0;
int heater_state; //0 -> waiting for temperature drop | 1 -> heating because of single sensor | 2 -> heating because of sensor average
unsigned long next_heater_check; //Timer to ensure temperature isn't checked too often
unsigned long lastTime = 0; //Simple local timer. Limits amount of I2C traffic to Ublox module.
String dataMessage;//SD Data
unsigned long launch_time; //Set to the current GPS time whenever the balloon is launched.
unsigned long last_valid_GPS_time;
unsigned long time_last_GPS;
float batt_1_voltage;
float batt_2_voltage;
float batt_3_voltage;
unsigned long next_battery_check;
boolean SDstatus;

/* Put your SSID & Password */
const char* ssid = "ESP32";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);

WebServer server(80);

void setup() {
  //Communication stuff
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); //Can choose pretty much any pins on the esp32 for I2C
  Wire.setClock(400000);

  //Set pin modes
  pinMode(HEATER_OUTPUT, OUTPUT);
  pinMode(BATT_1, INPUT);
  pinMode(BATT_2, INPUT);
  pinMode(BATT_3, INPUT);

  //wifi setup
  WiFi.softAP(ssid, password);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  delay(100);

  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);
  
  server.begin();
  Serial.println("HTTP server started");

  //Initialize variables if necessary
  heater_state = 0;
  next_heater_check = millis();
  
  //Initialize sensors/modules if necessary
  if(sensor1.begin(0x48, Wire)) {
    Serial.println("Temp Sensor 1 OK");
  } else {
    Serial.println("Temp Sensor 1 failed to initialize");
  }
  if(sensor2.begin(0x49, Wire)) {
    Serial.println("Sensor 2 OK");
  } else {
    Serial.println("Sensor 2 failed to initialize");
  }
  if(sensor3.begin(0x4A, Wire)) {
    Serial.println("Sensor 3 OK");
  } else {
    Serial.println("Sensor 3 failed to initialize");
  }
  if(sensor4.begin(0x4B, Wire)) {
    Serial.println("Sensor 4 OK");
  } else {
    Serial.println("Sensor 4 failed to initialize");
  }
  if (myGPS.begin(Wire) == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  
  //SD card
  SD.begin(SD_CS);  
  if(!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    SDstatus = false;
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    SDstatus = false;
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
    SDstatus = false;
  }

  //Create files for logging data
  File file = SD.open("/temperature_data.csv");
  if(!file) {
    Serial.println("File temperature_data.csv doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/temperature_data.csv", "MCU Time,GPS Time,Heater State,Temp 1,Temp 2,Temp 3,Temp 4\r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();
  
  file = SD.open("/GPS_data.csv");
  if(!file) {
    Serial.println("File GPS_data.csv doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/GPS_data.csv", "MCU Time,GPS Time,Latitude,Longitude,Altitude,SIV,Ground Speed,Heading\r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();

  file = SD.open("/battery_data.csv");
  if(!file) {
    Serial.println("File battery_data.csv doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/battery_data.csv", "MCU Time,GPS Time,Battery 1 Voltage,Battery 2 Voltage,Battery 3 Voltage\r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();

  file = SD.open("/time_data.csv");
  if(!file) {
    Serial.println("File time_data.csv doesn't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/time_data.csv", "Microcontroller Time,GPS Time,Ublox Time Valid,Ublox Date Valid\r\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close();

}

void loop() {
  //blocks of code for each subsystem are in separate functions for organization
  heater();
  GPS();
  server.handleClient();
    
  readBatteryVoltage();
  
}

void handle_OnConnect() {
  server.send(200, "text/html", SendHTML(SDstatus)); 
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML(boolean SDstatus){
 String s = " ";
 return s;
  
}

void heater() { //Contains the loop code for the heater system
  if(millis() >= next_heater_check) {
    
    temp1 = sensor1.readTempC();
    temp2 = sensor2.readTempC();
    temp3 = sensor3.readTempC();
    temp4 = sensor4.readTempC();

    Serial.print("Heater state: "); Serial.print(heater_state);
    Serial.print("Temperature 1: "); Serial.print(temp1);
    Serial.print(" | Temperature 2: "); Serial.print(temp2);
    Serial.print(" | Temperature 3: "); Serial.print(temp3);
    Serial.print(" | Temperature 4: "); Serial.print(temp4);
    Serial.print(" | Average: "); Serial.println((temp1 + temp2 + temp3 + temp4)/4.0);
    
    if(heater_state == 0) {
      if(temp1 < MIN_TEMP_SINGLE || temp2 < MIN_TEMP_SINGLE || temp3 < MIN_TEMP_SINGLE || temp4 < MIN_TEMP_SINGLE) {
        heater_state = 1;
        digitalWrite(HEATER_OUTPUT, HIGH);
      } else if ( (temp1 + temp2 + temp3 + temp4) / 4.0 < MIN_TEMP_AVG) {
        heater_state = 2;
        digitalWrite(HEATER_OUTPUT, HIGH);
      }
    } else if(heater_state == 1) {
      if(temp1 > MAX_TEMP_SINGLE && temp2 > MAX_TEMP_SINGLE && temp3 > MAX_TEMP_SINGLE && temp4 > MAX_TEMP_SINGLE) {
        heater_state = 0;
        digitalWrite(HEATER_OUTPUT, LOW);
      }
    } else if(heater_state == 2) {
      if( (temp1 + temp2 + temp3 + temp4) / 4.0 > MAX_TEMP_AVG) {
        heater_state = 0;
        digitalWrite(HEATER_OUTPUT, LOW);
      }
    }

    logTemperature();
    
    next_heater_check = millis() + HEATER_CHECK_TIME; //Wait and check temperatures again
  }
}

void GPS() {//Contains code for getting GPS position
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > GPS_CHECK_TIME)
  {
    lastTime = millis(); //Update the timer

    lastLatitude = latitude; //Save previous values for geofence subroutine
    lastLongitude = longitude;
    
    latitude = myGPS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print(((latitude)/10000000),4);
    //Serial.print((latitude));
    Serial.print(" degrees N;");

    longitude = myGPS.getLongitude();
    Serial.print(F(" Long: "));
    if(longitude < 0)
    {
      Serial.print(abs(longitude/10000000),4);
      Serial.print(F(" degrees W;"));
    }
    else
    {
      Serial.print(abs(longitude/10000000),4);
      Serial.print(F(" degrees E;"));
    }

    altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print((altitude/1000),2);
    Serial.print(F(" m;"));

    SIV = myGPS.getSIV();
    Serial.print(F(" Sats in view: "));
    Serial.print(SIV);

    Serial.println();

    geofenceCheck(); //Run the geofence check only after the GPS subroutine has pulled new data from the GPS

    logGPS();
    logTime();

    if(myGPS.getTimeValid()) {
      last_valid_GPS_time = getGPSTime();
      time_last_GPS = millis();
    }

    checkTimeCutdown();
  }
}

void readBatteryVoltage() {
  if(next_battery_check < millis()) {
    float pinb1v = map(analogRead(BATT_1), 0, 4095, 0, 3.3);
    float pinb2v = map(analogRead(BATT_2), 0, 4095, 0, 3.3);
    float pinb3v = map(analogRead(BATT_3), 0, 4095, 0, 3.3);
  
    batt_1_voltage = map(pinb1v, 1.7949, 2.6923, 14, 21);
    batt_2_voltage = map(pinb2v, 1.958, 2.937, 2.8, 4.2);
    batt_3_voltage = map(pinb3v, 1.958, 2.937, 2.8, 4.2);

    next_battery_check = millis() + BATTERY_CHECK_TIME;
  }
}

void geofenceCheck() {    //Run every time there's new GPS data available
  //Record current latitude and longitude displacement for averaging
  pastLatitudeDisplacement[avgFramePos] = latitude - lastLatitude;
  pastLongitudeDisplacement[avgFramePos] = longitude - lastLongitude;
  avgFramePos++;
  avgFramePos = avgFramePos % DISPLACEMENT_AVG_FRAME;

  //Determine average latitude and longitude drift rate
  float latSum, longSum;
  for(int i = 0; i < DISPLACEMENT_AVG_FRAME; i++){
    latSum = latSum + pastLatitudeDisplacement[i];
    longSum = longSum + pastLongitudeDisplacement[i];
  }
  float latDriftRate = latSum / (DISPLACEMENT_AVG_FRAME * GPS_CHECK_TIME);
  float longDriftRate = longSum / (DISPLACEMENT_AVG_FRAME * GPS_CHECK_TIME);

  //Find predicted latitude and longitude of landing position
  float latPredicted = latDriftRate * (altitude / 1000 / DESCENT_RATE); //Altitude is given by the gps in mm
  float longPredicted = longDriftRate * (altitude / 1000 / DESCENT_RATE);

  //Find if cutdown is required
  if( (latPredicted / 10000000) < 30.736 ) { //latitude and longitude are given by the GPS in degrees * 10^7
    triggerCutdown();
  } else if( (latPredicted / 10000000) < 32.851 ) {
    if( (longPredicted / 10000000) > ((latPredicted / 10000000 / 2.2175) - 96.017) ) {
      triggerCutdown();
    }
  } else if( (latPredicted / 10000000) < 35.031 ) {
    if( (longPredicted / 10000000) > ((latPredicted / 10000000 / .63537) - 132.61) ) {
      triggerCutdown();
    }
  } else {
    if( (longPredicted / 10000000) > -77.468 ) {
      triggerCutdown();
    }
  }
}

void checkTimeCutdown() {
  if(launched) {
    if(myGPS.getTimeValid()) {
      if( (last_valid_GPS_time - launch_time) > TIME_UNTIL_CUTDOWN ) {
        triggerCutdown();
      }
    } else {
      if( (last_valid_GPS_time - launch_time) + (millis() - time_last_GPS) > TIME_UNTIL_CUTDOWN ) {
        triggerCutdown();
      }
    }
  }
}

//Write temperature data to the corresponding file
void logTemperature() {
  String dataMessage = String(millis()) + "," + String(getGPSTime()) + "," + String(heater_state) + "," + String(temp1) +
                       "," + String(temp2) + "," + String(temp3) + "," + String(temp4) + "\r\n";
  appendFile(SD, "/temperature_data.csv", dataMessage.c_str());
}

//Write GPS data to the corresponding file
void logGPS() {
  String dataMessage = String(millis()) + "," + String(getGPSTime()) + "," + String(latitude) + "," + String(longitude) + "," +
                       String(altitude) + "," + String(SIV) + "," + String(myGPS.getGroundSpeed()) + "," +
                       String(myGPS.getHeading()) + "\r\n";
  appendFile(SD, "/GPS_data.csv", dataMessage.c_str());
}

//Write battery data to the corresponding file
void logBattery() {
  String dataMessage = String(millis()) + "," + String(getGPSTime()) + "," + String(batt_1_voltage) + "," + String(batt_2_voltage) + "," + String(batt_3_voltage) + "\r\n";
  appendFile(SD, "/battery_data.csv", dataMessage.c_str());
}

//Write mcu/gps time data to the corresponding file
void logTime() {
  String dataMessage = String(millis()) + "," + String(getGPSTime()) + "," + String(myGPS.getTimeValid()) + 
                       "," + String(myGPS.getDateValid()) + "\r\n";
  appendFile(SD, "/time_data.csv", dataMessage.c_str());
}

void triggerCutdown() {
  //Placeholder for now
}

//Get the time from the UBLOX module to the millisecond
//Note that zero for this time is midnight on the first of the month, so it "overflows" back to zero then.
unsigned long getGPSTime() {
  return (((myGPS.getDay() * 24 + myGPS.getHour()) * 60 + myGPS.getMinute()) * 60 + myGPS.getSecond()) * 1000 + myGPS.getMillisecond();
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}
