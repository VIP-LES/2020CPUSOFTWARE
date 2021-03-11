#include <HardwareSerial.h>


HardwareSerial sender(1); //can be Serial2 as well, just use proper pins

void setup() 
{
  Serial.begin(115200);//open serial via USB to PC on default port
  sender.begin(115200, SERIAL_8N1, 17, 16);//open the other serial port
}

void loop() 
{  
  float tester = 3.141592;
  sender.println(tester);
  sender.println("We're no strangers to love");
  Serial.println("data sent");

  delay(3000);
}
