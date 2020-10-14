/* Author: 
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

//Pin definitions
#define HEATER_OUTPUT 13
#define I2C_SDA 21
#define I2C_SCL 22
#define SD_CS 5

//other configuration option definitions
#define MIN_TEMP_SINGLE 10.0 //degrees celsius
#define MIN_TEMP_AVG 18.0
#define MAX_TEMP_SINGLE 20.0
#define MAX_TEMP_AVG 22.0
#define HEATER_CHECK_TIME 500 //in milliseconds
#define GPS_CHECK_TIME 5000 
#define DISPLACEMENT_AVG_FRAME 10
#define DESCENT_RATE 10 //in meters per second

//Global variables
TMP117 sensor1;
TMP117 sensor2;
TMP117 sensor3;
TMP117 sensor4;
SFE_UBLOX_GPS myGPS;

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

void setup() {
  //Communication stuff
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); //Can choose pretty much any pins on the esp32 for I2C
  Wire.setClock(400000);

  //Set pin modes
  pinMode(HEATER_OUTPUT, OUTPUT);

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
  }
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    Serial.println("No SD card attached");
  }
  Serial.println("Initializing SD card...");
  if (!SD.begin(SD_CS)) {
    Serial.println("ERROR - SD card initialization failed!");
  }
  File file = SD.open("/data.txt");
  if(!file) {
    Serial.println("File doens't exist");
    Serial.println("Creating file...");
    writeFile(SD, "/data.txt", "Reading ID, Date, Hour, Temperature \r\n");
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
  logSDCard();
  
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

// Write the sensor readings on the SD card
void logSDCard() {
  dataMessage = String(heater_state) + "," + String(temp1) + "," + String(temp2) + "," + 
                String(temp3) + "," + String(temp4) + "\r\n" + String(latitude) + "," + 
                String(longitutde)+ "," + String(altitude) + "," + String(SIV);
  Serial.print("Save data: ");
  Serial.println(dataMessage);
  appendFile(SD, "/data.txt", dataMessage.c_str());
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


void triggerCutdown() {
  //Placeholder for now
}
