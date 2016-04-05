/*
  Rocket Code
  This is loosely based on example code in the public domain, so I guess this is too, if you ever need to make a Curvinci rocket

  It takes the myriad of sensors of the Adafruit BNO055 12 axis sensor and records the vector/quaternion values to a log file

 */

#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>


#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055();

const int chipSelect = 10;
String accelString = "";
String baroString = "";
String gyroString = "";
String magString = "";
String lin_accelString = "";
String gravString= "";

File accelerometer;
File linear_acceleration;
File gravity;
//File barometer;
File w_velocity;
File gyroscope;
File Temperature;
File Orientation;
File Log;


void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  File Log = SD.open("log.txt", FILE_WRITE);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
 
  while (!Serial) {
     // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");
  pinMode(13, OUTPUT);

   //see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  if(!bno.begin()){
    Serial.println("Inertial Sensor failed, or not present");
  }
  delay(1000);

  bno.setExtCrystalUse(true);
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
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  String accelString = getString(accel);
  imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
  String lin_accelString = getString(lin_accel);
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);
  String gravString = getString(grav);
  sensors_event_t event;
  int8_t temp = bno.getTemp();
  int baro = analogRead(1);
 

  bno.getEvent(&event);
  baroString = String(baro);


  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File accelerometer = SD.open("accellog.txt", FILE_WRITE);
  
  // if the file is available, write to it:
  if (accelerometer) {
    accelerometer.println();
    // print to the serial port too:
    Serial.println(accelString);
  }else {
    // if the file didn't open, print an error:
    Serial.println("error opening accellog.txt");
  }
  File barometer = SD.open("barolog.txt", FILE_WRITE);
  if (barometer) {
    barometer.println(baroString);
    barometer.close();
    // print to the serial port too:
    Serial.println(baroString);
  }
  File gyroscope = SD.open("gyrolog.txt", FILE_WRITE);
  if (gyroscope) {
    gyroscope.println(gyroString);
    gyroscope.close();
    // print to the serial port too:
    Serial.println(gyroString);
  }
  // if the file isn't open, pop up an error:
  Serial.println("---");



}
/* Takes vector output and makes it a nice string to look at, may change it to a row vector but whatever
 * 
 */
String getString(imu::Vector<3> vec){
  String res = "X: " + String(vec.x()) + "Y: " + String(vec.y()) + "Z: " + String(vec.z());
  return res;
}
bool Calibration(){
  uint8_t system,gyro,accel,mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system,&gyro,&accel,&mag);
}










