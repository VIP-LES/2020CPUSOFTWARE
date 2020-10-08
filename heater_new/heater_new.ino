/* Author: Kian Vilhauer
 * Fall 2020
 */

/*
temperature parameters:
  Goal: Keep inside temperature at 20 Celsius
  if any single sensor below 10, turn on heat for 30 seconds OR until all sensors are >= 20
  if average is <= 18 turn on heat for 30 seconds OR untill average is 22
  else do nothing
*/

#include <Wire.h>
#include <SparkFun_TMP117.h>

//Pin definitions
#define heater 1
#define I2C_SDA 32
#define I2C_SCL 33

//Use these definitions to set configuration options
#define MIN_TEMP_SINGLE 10.0
#define MIN_TEMP_AVG 18.0
#define MAX_TEMP_SINGLE 20.0
#define MAX_TEMP_AVG 22.0
#define HEATER_CHECK_TIME 500 //in milliseconds

TMP117 sensor1;
TMP117 sensor2;
TMP117 sensor3;
TMP117 sensor4;

float temp1, temp2, temp3, temp4;
int heater_state; //0 -> waiting for temperature drop | 1 -> heating because of single sensor | 2 -> heating because of sensor average
unsigned long next_heater_check;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); //Can choose pretty much any pins on the esp32 for I2C
  Wire.setClock(400000);

  if(sensor1.begin(0x48, Wire)) {
    Serial.println("Sensor 1 OK");
  } else {
    Serial.println("Sensor 1 failed to initialize");
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

  pinMode(heater, OUTPUT);

  heater_state = 0;
  next_heater_check = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
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
        digitalWrite(heater, HIGH);
      } else if ( (temp1 + temp2 + temp3 + temp4) / 4.0 < MIN_TEMP_AVG) {
        heater_state = 2;
        digitalWrite(heater, HIGH);
      }
    } else if(heater_state == 1) {
      if(temp1 > MAX_TEMP_SINGLE && temp2 > MAX_TEMP_SINGLE && temp3 > MAX_TEMP_SINGLE && temp4 > MAX_TEMP_SINGLE) {
        heater_state = 0;
        digitalWrite(heater, LOW);
      }
    } else if(heater_state == 2) {
      if( (temp1 + temp2 + temp3 + temp4) / 4.0 > MAX_TEMP_AVG) {
        heater_state = 0;
        digitalWrite(heater, LOW);
      }
    }

    next_heater_check = millis() + HEATER_CHECK_TIME; //Wait and check temperatures again
  }
}
