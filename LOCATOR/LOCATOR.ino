#include <cmath> as std;

#include <Wire.h> //Needed for I2C to GPS

#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
SFE_UBLOX_GPS myGPS;

long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.

void setup()
{
  Serial.begin(115200);
  while (!Serial); //Wait for user to open terminal
  Serial.println("SparkFun Ublox Example");

  Wire.begin();

  if (myGPS.begin() == false) //Connect to the Ublox module using Wire port
  {
    Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
    while (1);
  }

  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
}

void loop()
{
  //Query module only every second. Doing it more often will just cause I2C traffic.
  //The module only responds when a new position is available
  if (millis() - lastTime > 5000)
  {
    lastTime = millis(); //Update the timer
    
    float latitude = myGPS.getLatitude();
    Serial.print(F("Lat: "));
    Serial.print((latitude)/10000000);
    Serial.print(" degrees N;");

    float longitude = myGPS.getLongitude();
    Serial.print(F(" Long: "));
    if(longitude < 0)
    {
      Serial.print(abs(longitude/10000000));
      Serial.print(F(" degrees W;"));
    }
    else
    {
      Serial.print(abs(longitude/10000000));
      Serial.print(F(" degrees E;"));
    }
    

    float altitude = myGPS.getAltitude();
    Serial.print(F(" Alt: "));
    Serial.print(altitude/1000);
    Serial.print(F(" m;"));

    byte SIV = myGPS.getSIV();
    Serial.print(F(" Sats in view: "));
    Serial.print(SIV);

    Serial.println();
  }
}
