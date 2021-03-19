#include <HardwareSerial.h>
#include <SD.h>

#include <Wire.h>
#include <Adafruit_MPL3115A2.h>
#include <SparkFun_VEML6075_Arduino_Library.h>
#include <SparkFun_ADXL345.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <SparkFun_HIH4030.h>
#include <SparkFun_TMP117.h>
#include "FS.h"
#include <sstream>

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);


int tempAddress = 0x48;             // I2C: TMP102 Slave Address with ADD0 Grounded
float temp = 0;                     // Temperature Value either Static or Sensor Reading

float temp1; //TMP117

// Are You Using a Temperature Sensor? 1 = YES / 0 = NO
int tempSensor = 1;

// Analog IO Pin Connected to OUT
#define HIH4030_OUT 4

// Supply Voltage - Typically 5 V
#define HIH4030_SUPPLY 5

// Library Variables
HIH4030 sensorSpecs(HIH4030_OUT, HIH4030_SUPPLY);


VEML6075 uv; // Create a VEML6075 object



#define SD_CS 14
boolean SDstatus = true;
HardwareSerial receiver(2);

TMP117 sensor1;

String dataMessage;


float UVA; float UVB; float UV_INDEX;



void setup() 
{
  Serial.begin(115200);//open serial via USB to PC on default port
  receiver.begin(115200, SERIAL_8N1, 27, 26);

  Wire.begin();                     // I2C: Utilized by TMP102 Temperature Sensor


  //TMP117
  if(sensor1.begin(0x48, Wire)) {
    Serial.println("Temp Sensor 117 OK");
  } else {
    Serial.println("Temp Sensor 117 failed to initialize");
  }


  //HMC MAGNETOMETER
  Serial.println("HMC5883 Magnetometer Test"); Serial.println("");
  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while(1);
  }
  /* Display some basic information on this sensor */
  displaySensorDetails();




  //UV
  if (uv.begin() == false)
  {
    Serial.println("Unable to communicate with VEML6075.");
    while (1)
      ;
  }
  Serial.println("UVA, UVB, UV Index");

  

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

  File file = SD.open("/dump.txt");
  if(!file) {
    Serial.println("File tester.txt doesn't exist");
    Serial.println("Creating file...");
    //writeFile(SD, "/dump.txt", "File created\n");
  }
  else {
    Serial.println("File already exists");  
  }
  file.close(); 
}



void loop() 
{  
  
  if(receiver.available()) //check incoming on other serial from the other board
  {
    String ricky = receiver.readString();
    Serial.println(ricky);
    
    //String dataMessage = String(stor) + "\n" + "asdf";
    //appendFile(SD, "/tester.txt", dataMessage.c_str());

    dataMessage = "FROM MASTER: \n" + ricky;
    appendFile(SD, "/dump.txt", dataMessage.c_str());  
  }


  Serial.println("CLIENT SENSORS: ");
  String dataMessage = "CLIENT SENSORS: \n";
  appendFile(SD, "/dump.txt", dataMessage.c_str());
  
  //RUN TMP
  temp = sensor1.readTempC();
  Serial.print("TMP117 Temperature: "); Serial.print(temp); Serial.println("*C");
  dataMessage = "TMP117 Temperature: " + String(temp) + " *C";
  appendFile(SD, "/dump.txt", dataMessage.c_str());



  //RUN BAROMETER MPL
  if (! baro.begin()) {
    Serial.println("Couldn't find sensor");
    return;
  }
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  float pascals = baro.getPressure();
  Serial.print(pascals/3377); Serial.println(" Inches (Hg)");
  float altm = baro.getAltitude();
  Serial.print("Altitude: "); Serial.print(altm); Serial.println(" meters");
  float tempC = baro.getTemperature();
  Serial.print("TMP102 (aka VML) Temperature: "); Serial.print(tempC); Serial.println("*C");
  delay(250);
  dataMessage = "Pressure: " + String(pascals/3377) + " Inches (Hg);\n"
                + "Altitude: "+ String(altm) + " meters;\n"
                + "TMP102 (aka VML) Temperature: "+ String(tempC) + " *C\n";
  appendFile(SD, "/dump.txt", dataMessage.c_str());




  /*HMC MAGNETOMETER: Display the results (magnetic vector values are in micro-Tesla (uT)) */
  sensors_event_t event; 
  mag.getEvent(&event);  
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");
  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI;
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  delay(500);
  dataMessage = "X: " + String(event.magnetic.x) + "  "
                + "Y: "+ String(event.magnetic.y) + "  "
                + "Z: "+ String(event.magnetic.z) + "  uT \n"
                + "Heading (degrees): " + String(headingDegrees);
  appendFile(SD, "/dump.txt", dataMessage.c_str());




  
  // Use the uva, uvb, and index functions to read calibrated UVA and UVB values and a
  // calculated UV index value between 0-11.
  UVA = uv.uva();
  UVB = uv.uvb();
  UV_INDEX = uv.index();
  Serial.println(String(uv.uva()) + " UVA, " + String(uv.uvb()) + " UVB, " + String(uv.index()) + " UV Index");
  delay(250);
  dataMessage = "UV Readings: " + String(UVA) + "  UVA,"
                + String(UVB) + "  UVB,"
                + String(UV_INDEX) + "  UV Index";
  appendFile(SD, "/dump.txt", dataMessage.c_str());


  delay(60000);

}





/* Serial Output of Temperature C, Sensor Voltage V, SensorRH %, and TrueRH % */
void printData(HIH4030 sensor, float temperature){
  
  Serial.print("Temperature = ");
  Serial.print(temperature);
  Serial.print(" C ("); Serial.print((temperature * 9/5) + 32); Serial.println(" F)");
  Serial.print("Sensor Voltage = ");
  Serial.print(sensor.vout());
  Serial.println(" V");
  Serial.print("Relative Humidity = ");
  Serial.print(sensor.getSensorRH());
  Serial.println(" %");
  Serial.print("True Relative Humidity = ");
  Serial.print(sensor.getTrueRH(temperature));
  Serial.println(" %");
  
}

























/* Only Called IF Temperature Sensor is Being Used */
float getTemperature(){

  /*  IF Temperature Sensor is different from the TMP102 and not   */      
  /*    utilizing I2C via the Wire.h library, replace the code     */ 
  /*    within the getTemperature() function.                      */ 
  
  Wire.requestFrom(tempAddress,2);  // Wire.requestFrom(address, # of bytes to request)

  byte MSB = Wire.read();           // Receive byte as MSB or left-most bit
  byte LSB = Wire.read();           // Receive byte as LSB or right-most bit

  /*  TMP102 Temperature Register 12-bit, read-only, two's compliment for negative */
  /*  << is bit shift left ; >> is bit shift right ; | is bitwise OR               */
  /*  Syntax:   variable << number_of_bits                                         */
  /*            variable >> number_of_bits                                         */
  int TemperatureSum = ((MSB << 8) | LSB) >> 4; 

  float celsius = TemperatureSum*0.0625;  //One LSB equals 0.0625 C
  return celsius;
  
}






//HMC
void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}



















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
