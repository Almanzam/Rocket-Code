/*TODO: add logic to make the rocket stop taking data after it hits the ground
 *      make sure what we have is not broken 
 * 
 */





#include <SPI.h>
#include <SD.h>

const int chipSelect = 4;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
}

void loop() {
  // make a string for assembling the data to log:
  String accelString = "";
  String baroString = "";
  String gyroString = "";
  int loopcount = 0;
  // read three sensors and append to the string: 
//  for (int analogPin = 0; analogPin < 3; analogPin++) {
//    int sensor = analogRead(analogPin);
//    dataString += String(sensor);
//    if (analogPin < 2) {
//      dataString += ",";
//    }
//  }
  int accel = analogread(0);
  int baro = analogread(1);
  int gyro = analogread(2);

  accelString += String(accel);
  baroString += String(baro);
  gyroString += String(gyro);

  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File accelerometer = SD.open("accellog.txt", FILE_WRITE);
  
  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(accelString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  File barometer = SD.open("barolog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(baroString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  File gyroscope = SD.open("gyrolog.txt", FILE_WRITE);
  if (dataFile) {
    dataFile.println(String);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }

  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
}









