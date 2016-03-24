/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;
String accelString = "";
String baroString = "";
String gyroString = "";
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }
//
//
//  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
//  if (!SD.begin(chipSelect)) {
//    Serial.println("Card failed, or not present");
//    // don't do anything more:
//    return;
//  }
//  Serial.println("card initialized.");
}

void loop() {
  // make a string for assembling the data to log:
  
  int loopcount = 0;
  // read three sensors and append to the string:
//  for (int analogPin = 0; analogPin < 3; analogPin++) {
//    int sensor = analogRead(analogPin);
//    dataString += String(sensor);
//    if (analogPin < 2) {
//      dataString += ",";
//    }
//  }
  int accel = analogRead(0);
  int baro = analogRead(1);
  int gyro = analogRead(2);

  accelString += String(accel);
  baroString += String(baro);
  gyroString += String(gyro);

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File accelerometer = SD.open("accellog.txt", FILE_WRITE);
  
  // if the file is available, write to it:
  if (accelerometer) {
    accelerometer.println(accelString);
    accelerometer.close();
    // print to the serial port too:
    Serial.println(accelString);
  }
  File barometer = SD.open("barolog.txt", FILE_WRITE);
  if (barometer) {
    barometer.println(baroString);
    barometer.close();
    // print to the serial port too:
    Serial.println(baroString);
  }
  File gyroscope = SD.open("gyrolog.txt", FILE_WRITE);
  if (barometer) {
    barometer.println(gyroString);
    barometer.close();
    // print to the serial port too:
    Serial.println(gyroString);
  }
  // if the file isn't open, pop up an error:
  
}









