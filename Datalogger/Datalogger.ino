/*
  Rocket Code
  This is loosely based on example code in the public domain, so I guess this is too, if you ever need to make a Curvinci rocket

  It takes the myriad of sensors of the Adafruit BNO055 12 axis sensor and records the vector/quaternion values to a log file

 */

#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
Adafruit_BNO055 bno = Adafruit_BNO055();

const int chipSelect = 10;
String accelString = "";
String baroString = "";
String gyroString = "";
String magString = "";
String lin_accelString = "";
String gravString= "";
bool isCalibrated = false;

File Accelerometer;
File Linear_acceleration;
File Gravity;
File Magnet;
File W_velocity;
File Orientation;
File Temperature;
File Log;


void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  File Log = SD.open("log.txt", FILE_WRITE);
  Log.println("------------------------------------");
  Log.print  ("Sensor:       "); Log.println(sensor.name);
  Log.print  ("Driver Ver:   "); Log.println(sensor.version);
  Log.print  ("Unique ID:    "); Log.println(sensor.sensor_id);
  Log.print  ("Max Value:    "); Log.print(sensor.max_value); Log.println(" xxx");
  Log.print  ("Min Value:    "); Log.print(sensor.min_value); Log.println(" xxx");
  Log.print  ("Resolution:   "); Log.print(sensor.resolution); Log.println(" xxx");
  Log.println("------------------------------------");
  Log.println("");
  Log.close();
  delay(500);
}
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  File Log = SD.open("log.txt", FILE_WRITE);
  /* Display the results in the Serial Monitor */
  Log.println("");
  Log.print("System Status: 0x");
  Log.println(system_status, HEX);
  Log.print("Self Test:     0x");
  Log.println(self_test_results, HEX);
  Log.print("System Error:  0x");
  Log.println(system_error, HEX);
  Log.println("");
  Log.close();
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
  displaySensorDetails();
  displaySensorStatus();
  while(!bno.isFullyCalibrated()){
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);              // wait for a second
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delay(1000); 
  }
  bno.setExtCrystalUse(true);
}

void loop() {
  // Get the vectors that the inertial sensor provides
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  
  imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 
  imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);  
  imu::Vector<3> w_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

  sensors_event_t event;
  uint8_t temp = bno.getTemp();
  bno.getEvent(&event);

  //Turn them into strings for us to look at
  String accelString = getString(accel);
  String lin_accelString = getString(lin_accel);
  String gravString = getString(grav);
  String w_velString = getString(w_vel);
  String magString = getString(magnet);
  String tempString = String(temp);
 
 


  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File Accelerometer = SD.open("accellog.txt", FILE_WRITE);
  
  // if the file is available, write to it:
  if (Accelerometer) {
    Accelerometer.println(accelString);
    Accelerometer.close();
    // print to the serial port too:
    Serial.println(accelString);
  }else {
    // if the file didn't open, print an error:
    Serial.println("error opening accellog.txt");
  }
  
  File Linear_Acceleration = SD.open("lin_accellog.txt",FILE_WRITE);
    if (Linear_Acceleration) {
        Linear_Acceleration.println(lin_accelString);
        // print to the serial port too:
        Serial.println(lin_accelString);
      }else {
          // if the file didn't open, print an error:
          Serial.println("error opening lin_accellog.txt");
      }
  File Gravity = SD.open("gravlog.txt",FILE_WRITE);
  if (Gravity) {
    Gravity.println(gravString);
    // print to the serial port too:
    Serial.println(gravString);
    Gravity.close();
  }else {
    // if the file didn't open, print an error:
    Serial.println("error opening gravlog.txt");
  }
  File Magnet = SD.open("maglog.txt",FILE_WRITE);
  if (Magnet) {
    Magnet.println(magString);
    // print to the serial port too:
    Serial.println(magString);
    Magnet.close();
  }else {
    // if the file didn't open, print an error:
    Serial.println("error opening maglog.txt");
  }
  File W_velocity = SD.open("w_vellog.txt",FILE_WRITE);
  if (W_velocity) {
    W_velocity.println(w_velString);
    // print to the serial port too:
    Serial.println(w_velString);
    W_velocity.close();
  }else {
    // if the file didn't open, print an error:
    Serial.println("error opening w_vellog.txt");
  }
  
  File Orientation = SD.open("orlog.txt", FILE_WRITE);
  if (Orientation) {    
    Orientation.print("X: ");
    Orientation.print(event.orientation.x, 4);
    Orientation.print("\tY: ");
    Orientation.print(event.orientation.y, 4);
    Orientation.print("\tZ: ");
    Orientation.print(event.orientation.z, 4);
    Orientation.println("");
    Orientation.close(); 
  }
  File Temperature = SD.open("templog.txt",FILE_WRITE);
  if(Temperature){
    Temperature.println(tempString);
  }else{
    Serial.println("Error opening templog.txt");
  }
  Serial.println("---");
  delay(100);
}
/* Takes vector output and makes it a nice string to look at, may change it to a row vector but whatever
 * 
 */
String getString(imu::Vector<3> vec){
  String res = "X: " + String(vec.x()) + "Y: " + String(vec.y()) + "Z: " + String(vec.z());
  return res;
}












