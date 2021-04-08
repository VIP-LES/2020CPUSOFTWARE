#include <Wire.h>
#include <Adafruit_MPL3115A2.h> //https://github.com/adafruit/Adafruit_MPL3115A2_Library/blob/master/Adafruit_MPL3115A2.h
#include <SparkFun_VEML6075_Arduino_Library.h> //https://github.com/sparkfun/SparkFun_VEML6075_Arduino_Library
#include <SparkFun_ADXL345.h> //https://github.com/sparkfun/SparkFun_ADXL345_Arduino_Library/blob/master/src/SparkFun_ADXL345.h
#include <Adafruit_Sensor.h> //https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h
#include <Adafruit_HMC5883_U.h> //https://github.com/adafruit/Adafruit_HMC5883_Unified/blob/master/Adafruit_HMC5883_U.h
#include <SparkFun_HIH4030.h>//https://github.com/sparkfun/SparkFun_HIH4030_Arduino_Library
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

// Example Variables
int tempAddress = 0x48;             // I2C: TMP102 Slave Address with ADD0 Grounded
float temp = 0;                     // Temperature Value either Static or Sensor Reading

// Are You Using a Temperature Sensor? 1 = YES / 0 = NO
int tempSensor = 0;

// Analog IO Pin Connected to OUT
#define HIH4030_OUT 4

// Supply Voltage - Typically 5 V
#define HIH4030_SUPPLY 5

// Library Variables
HIH4030 sensorSpecs(HIH4030_OUT, HIH4030_SUPPLY);

void setup(void)
{
  
  Serial.begin(115200);               // Serial Port
  Wire.begin();                     // I2C: Utilized by TMP102 Temperature Sensor
  
}




VEML6075 uv; // Create a VEML6075 object
ADXL345 adxl = ADXL345();

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


  Serial.begin(115200);
  Serial.println("Adafruit_MPL3115A2 test!");
  Wire.begin();

  // the VEML6075's begin function can take no parameters
  // It will return true on success or false on failure to communicate
  if (uv.begin() == false)
  {
    Serial.println("Unable to communicate with VEML6075.");
    while (1)
      ;
  }
  Serial.println("UVA, UVB, UV Index");
  Serial.println("SparkFun ADXL345 Accelerometer Hook Up Guide Example");
  Serial.println();
  
  adxl.powerOn();                     // Power on the ADXL345

  adxl.setRangeSetting(4);           // Give the range settings
                                      // Accepted values are 2g, 4g, 8g or 16g
                                      // Higher Values = Wider Measurement Range
                                      // Lower Values = Greater Sensitivity

  adxl.setSpiBit(0);                  // Configure the device to be in 4 wire SPI mode when set to '0' or 3 wire SPI mode when set to 1
                                      // Default: Set to 1
                                      // SPI pins on the ATMega328: 11, 12 and 13 as reference in SPI Library 
   
  adxl.setActivityXYZ(1, 1, 1);       // Set to activate movement detection in the axes "adxl.setActivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setActivityThreshold(75);      // 62.5mg per increment   // Set activity   // Inactivity thresholds (0-255)
 
  adxl.setInactivityXYZ(1, 1, 1);     // Set to detect inactivity in all the axes "adxl.setInactivityXYZ(X, Y, Z);" (1 == ON, 0 == OFF)
  adxl.setInactivityThreshold(75);    // 62.5mg per increment   // Set inactivity // Inactivity thresholds (0-255)
  adxl.setTimeInactivity(10);         // How many seconds of no activity is inactive?

  adxl.setTapDetectionOnXYZ(1, 1, 1); // Detect taps in the directions turned ON "adxl.setTapDetectionOnX(X, Y, Z);" (1 == ON, 0 == OFF)
 
  // Set values for what is considered a TAP and what is a DOUBLE TAP (0-255)
  adxl.setTapThreshold(50);           // 62.5 mg per increment
  adxl.setTapDuration(15);            // 625 μs per increment
  adxl.setDoubleTapLatency(80);       // 1.25 ms per increment
  adxl.setDoubleTapWindow(200);       // 1.25 ms per increment
 
  // Set values for what is considered FREE FALL (0-255)
  adxl.setFreeFallThreshold(7);       // (5 - 9) recommended - 62.5mg per increment
  adxl.setFreeFallDuration(30);       // (20 - 70) recommended - 5ms per increment
 
  // Setting all interupts to take place on INT1 pin
  //adxl.setImportantInterruptMapping(1, 1, 1, 1, 1);     // Sets "adxl.setEveryInterruptMapping(single tap, double tap, free fall, activity, inactivity);" 
                                                        // Accepts only 1 or 2 values for pins INT1 and INT2. This chooses the pin on the ADXL345 to use for Interrupts.
                                                        // This library may have a problem using INT2 pin. Default to INT1 pin.
  
  // Turn on Interrupts for each mode (1 == ON, 0 == OFF)
  adxl.InactivityINT(1);
  adxl.ActivityINT(1);
  adxl.FreeFallINT(1);
  adxl.doubleTapINT(1);
  adxl.singleTapINT(1);
  
//attachInterrupt(digitalPinToInterrupt(interruptPin), ADXL_ISR, RISING);   // Attach Interrupt
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

void loop() {
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }
  
  float pascals = baro.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals/3377); Serial.println(" Inches (Hg)");

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(" meters");

  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");

  delay(250);
  // Use the uva, uvb, and index functions to read calibrated UVA and UVB values and a
  // calculated UV index value between 0-11.
  Serial.println(String(uv.uva()) + ", " + String(uv.uvb()) + ", " + String(uv.index()));
  delay(250);
  // Accelerometer Readings
  int x,y,z;   
  adxl.readAccel(&x, &y, &z);         // Read the accelerometer values and store them in variables declared above x,y,z

  // Output Results to Serial
  /* UNCOMMENT TO VIEW X Y Z ACCELEROMETER VALUES */  
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z); 

  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
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
  /*  IF tempSensor = 1 Utilizing a Temperature Sensor             */
  /*  IF tempSensor = 0 Utilizing a Static Value for Temperature   */
  if (tempSensor == 1) {
    temp = getTemperature();        // Get Temperature Sensor Reading
  } else if (tempSensor == 0) {
    temp = 25;                      // Static Temperature Value in C
                                    // Set by User to Desired Temperature                            
  } else {
    while (tempSensor != 1 || tempSensor != 0){
      Serial.println("ERROR: tempSensor Value Out of Range");
    }
  }

  printData(sensorSpecs, temp);     // Print Sensor Readings
  Serial.println("");               // Return Space Between Readings
  delay(100);                       // Slow Down Serial Output
  
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
