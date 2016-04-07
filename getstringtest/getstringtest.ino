#include <Arduino.h>

#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);
void setup(){
	Serial.begin(9600);
	
}
void loop(){
	imu::Vector<3> vec = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
	Serial.println(getString(vec));
}
String getString(imu::Vector<3> vec){
  String res = "X: " + String(vec.x()) + "Y: " + String(vec.y()) + "Z: " + String(vec.z());
  Serial.println(res);
  return res;
}
