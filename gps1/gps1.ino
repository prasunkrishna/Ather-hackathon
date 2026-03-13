#include <HardwareSerial.h>
HardwareSerial GPS(2);
void setup(){
  Serial.begin(115200); delay(200);
  Serial.println("\nGPS read @ 9600 on RX=4 TX=5");
  GPS.begin(9600, SERIAL_8N1, 4, 5); // move GPS TX to GPIO4 instead of 16
}
void loop(){
  while(GPS.available()) Serial.write(GPS.read());
}
