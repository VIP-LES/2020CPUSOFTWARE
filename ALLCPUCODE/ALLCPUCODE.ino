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
//#include <WebServer.h>
#include <sstream>
#include <HardwareSerial.h>
#include <math.h>


//Pin definitions
#define HEATER_OUTPUT 25
#define CUTDOWN_SIGNAL 26
#define I2C_SDA 21
#define I2C_SCL 22
#define SD_CS 14
#define BATT_1 33 //21V nominal
#define BATT_2 34 //4.2V nominal
#define BATT_3 35 //4.2V nominal

//other configuration option definitions
#define MIN_TEMP_SINGLE 10.0 //degrees celsius
#define MIN_TEMP_AVG 15.0
#define MAX_TEMP_SINGLE 20.0
#define MAX_TEMP_AVG 22.0
#define HEATER_CHECK_TIME 500 //in milliseconds
#define GPS_CHECK_TIME 5000 //Also affects frequency of mcu/gps time logging
#define DISPLACEMENT_AVG_FRAME 10
#define DESCENT_RATE 10 //in meters per second
#define TIME_UNTIL_CUTDOWN 21600000 //How long until time-delay cutdown is triggered, in ms
#define BATTERY_CHECK_TIME 500 //in milliseconds
#define CUTDOWN_DISTANCE 222 //in km

//Global variables
TMP117 sensor1;
TMP117 sensor2;
//TMP117 sensor3;
//TMP117 sensor4;
SFE_UBLOX_GPS myGPS;

float disX;
float disY;
float fixedLong = 34.15604;
float fixedLat = -84.69975;
boolean launched = false; //This can be set to true through the web terminal right before the balloon is launched.  Ensures cutdown is not triggered while waiting on GPS fix.
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
boolean GPSstatus = false;
float longitudeHTML;
float latitudeHTML;

/* Put your SSID & Password */
const char* ssid = "ESP32";  // Enter SSID here
const char* password = "12345678";  //Enter Password here

boolean SDstatus = false;

WiFiServer server(80);
String header;
String output26State = "off";
String output27State = "off";
 
// Assign output variables to GPIO pins
const int output26 = 26;
const int output27 = 27;


HardwareSerial sender(1);

void setup() {
  //Communication stuff
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); //Can choose pretty much any pins on the esp32 for I2C
  Wire.setClock(400000);
  Serial.begin(115200);
  sender.begin(115200, SERIAL_8N1, 17, 16);//open the other serial port FROM MASTER CODE
  
  //Set pin modes
  pinMode(HEATER_OUTPUT, OUTPUT);
  pinMode(CUTDOWN_SIGNAL, OUTPUT);
  pinMode(BATT_1, INPUT);
  pinMode(BATT_2, INPUT);
  pinMode(BATT_3, INPUT);

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Setting AP (Access Point)…");
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.softAP(ssid, password);
 
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
   
  server.begin();

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
  /*if(sensor3.begin(0x4A, Wire)) {
    Serial.println("Sensor 3 OK");
  } else {
    Serial.println("Sensor 3 failed to initialize");
  }
  if(sensor4.begin(0x4B, Wire)) {
    Serial.println("Sensor 4 OK");
  } else {
    Serial.println("Sensor 4 failed to initialize");
  }*/
  if (myGPS.begin(Wire) == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
  

}

void loop() {
  //blocks of code for each subsystem are in separate functions for organization
  heater();
  GPS();    
  readBatteryVoltage();
  wifi();
}

void wifi() {
  WiFiClient client = server.available();   // Listen for incoming clients
  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
             
            // turns the launched on and off
            if (header.indexOf("GET /launched/on") >= 0) {
              Serial.println("it is launched");
              launched = true;
            } else if (header.indexOf("GET /launched/off") >= 0) {
              Serial.println("notlaunched");
              launched = false;
            }
             
            // Display the HTML web page  
            client.println("<!DOCTYPE html> <html>\n");//start doc
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n");//make
            client.println("<title>Dashboard</title>\n");
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n");//set font style
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");//set button style
            client.println("body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n");//create body of page
            client.println("p {font-size: 14px;color: #888;margin-bottom: 10px;}\n");//set font settings
            client.println("</style>\n");
            client.println("</head>\n");
            client.println("<body>\n");
            client.println("<h1>ESP32 Web Server</h1>\n");//set heading
            client.println("<h3>Lightning at the Edge of Space VIP</h3>\n");//set subheading
              if (!launched) {
                client.println("<p><a href=\"/launched/on\"><button class=\"button\">PRESS TO LAUNCH</button></a></p>");//makes button press to launch if not yet launched
              } else {
                client.println("<p><a href=\"/launched/off\"><button class=\"button button2\">LAUNCHED</button></a></p>");////makes button say launched if launches
              } 
              if(GPSstatus)
                {client.println("<p>GPS is connected and working properly</p>\n");} // says gps is working if data valid
              else
                {client.println("<p>GPS is not properly working</p>\n");} //says gps is not working if data invalid(zeros)
              client.println("<p>Longitude:" + String(longitudeHTML) + " </p> \n");//print longitude
              client.println("<p>Latitude:" + String(latitudeHTML) + " </p> \n");//print latitiyde
    
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }
}





void heater() { //Contains the loop code for the heater system
  if(millis() >= next_heater_check) {
    
    temp1 = sensor1.readTempC();
    temp2 = sensor2.readTempC();
    //temp3 = sensor3.readTempC();
    //temp4 = sensor4.readTempC();

    Serial.print("Heater state: "); Serial.print(heater_state);
    Serial.print("Temperature 1: "); Serial.print(temp1);
    Serial.print(" | Temperature 2: "); Serial.print(temp2);
    //Serial.print(" | Temperature 3: "); Serial.print(temp3);
    //Serial.print(" | Temperature 4: "); Serial.print(temp4);
    Serial.print(" | Average: "); Serial.println((temp1 + temp2 /*+ temp3 + temp4*/)/2.0/*4.0*/);
    
//    if(heater_state == 0) {
      if(temp1 < MIN_TEMP_SINGLE || temp2 < MIN_TEMP_SINGLE /*|| temp3 < MIN_TEMP_SINGLE || temp4 < MIN_TEMP_SINGLE*/) {
        heater_state = 1;
        digitalWrite(HEATER_OUTPUT, HIGH);
      } else if ( (temp1 + temp2 /*+ temp3 + temp4*/) / 2.0/*4.0*/ < MIN_TEMP_AVG) {
        heater_state = 2;
        digitalWrite(HEATER_OUTPUT, HIGH);
      }

      if(temp1 < MAX_TEMP_SINGLE || temp2 < MAX_TEMP_SINGLE || (temp1 + temp2 /*+ temp3 + temp4*/) / 2.0/*4.0*/ > MAX_TEMP_AVG)
      {
        digitalWrite(HEATER_OUTPUT, LOW);  
      }
//    }
//    else if(heater_state == 1) {
//      if(temp1 > MAX_TEMP_SINGLE && temp2 > MAX_TEMP_SINGLE /*&& temp3 > MAX_TEMP_SINGLE && temp4 > MAX_TEMP_SINGLE*/) {
//        heater_state = 0;
//        digitalWrite(HEATER_OUTPUT, LOW);
//      }
//    } else if(heater_state == 2) {
//      if( (temp1 + temp2 /*+ temp3 + temp4*/) / 2.0/*4.0*/ > MAX_TEMP_AVG) {
//        heater_state = 0;
//        digitalWrite(HEATER_OUTPUT, LOW);
//      }
//    }

    
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
    longitudeHTML = longitude;
    latitudeHTML = latitude;

    altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print((altitude/1000),2);
    Serial.print(F(" m;"));

    SIV = myGPS.getSIV();
    Serial.print(F(" Sats in view: "));
    Serial.print(SIV);
    if(SIV > 3){
      GPSstatus = true;
    }

    Serial.println();

    geofenceCheck(); //Run the geofence check only after the GPS subroutine has pulled new data from the GPS


    if(myGPS.getTimeValid()) {
      last_valid_GPS_time = getGPSTime();
      time_last_GPS = millis();
    }
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

void checkTimeCutdown() {
  if(launched) {
    if(myGPS.getTimeValid()) {
      if( (last_valid_GPS_time - launch_time) > TIME_UNTIL_CUTDOWN && (altitude/1000) > 800) {
        triggerCutdown();
      }
    }
//    else {
//      if( (last_valid_GPS_time - launch_time) + (millis() - time_last_GPS) > TIME_UNTIL_CUTDOWN ) {
//        triggerCutdown();
//      }
//    }
  }
}




void triggerCutdown()
{
  //Placeholder for now
  digitalWrite(CUTDOWN_SIGNAL, HIGH);
}

//Get the time from the UBLOX module to the millisecond
//Note that zero for this time is midnight on the first of the month, so it "overflows" back to zero then.
unsigned long getGPSTime() {
  return (((myGPS.getDay() * 24 + myGPS.getHour()) * 60 + myGPS.getMinute()) * 60 + myGPS.getSecond()) * 1000 + myGPS.getMillisecond();
}
