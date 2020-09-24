/* Author: Kian Vilhauer
 * Fall 2020
 */

/*
temperature parameters:
  Goal: Keep inside temperature at 20 Celsius
  if any single sensore below 10, turn on heat for 30 seconds OR until all sensores are >= 20
  if average is <= 18 turn on heat for 30 seconds OR untill average is 22
  else do nothing
*/

#include <Wire.h>
#include <SparkFun_TMP117.h>

#define heater 1

TMP117 sensor1;
TMP117 sensor2;
TMP117 sensor3;
TMP117 sensor4;

float temp1, temp2, temp3, temp4;
int heater_state; //0 -> waiting for temperature drop | 1 -> heating because of single sensor | 2 -> heating because of sensor average
unsigned long next_heater_check;

void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);

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

    if(heater_state == 0) {
      if(sensor1.readTempC() < 10.0 || sensor2.readTempC() < 10.0) {
        
      }
    } else if(heater_state == 1) {
      
    } else if(heater_state == 2) {
      
    }
  }
}
