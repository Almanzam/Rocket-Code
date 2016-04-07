#include <Arduino.h>

/*
  Rocket Code
  This is loosely based on example code in the public domain, so I guess this is too, if you ever need to make a Curvinci rocket

  It takes the myriad of sensors of the Adafruit BNO055 12 axis sensor and records the vector/quaternion values to a log file

 */

#include <SPI.h>
#include <SD.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
//#include <utility/imumaths.h>
Adafruit_BNO055 bno = Adafruit_BNO055(55);

const uint8_t chipSelect = 10;
uint8_t loopcount = 0;
void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
 
  while (!Serial) {
     // wait for serial port to connect. Needed for native USB port only
  }


  Serial.print("Initializing SD card...");
 //pin 13 means all ok
  pinMode(2, OUTPUT);
 //pin 10 means not calibrated
  pinMode(0, OUTPUT);

   //see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  if(!bno.begin()){
    Serial.println("Inertial Sensor failed, or not present");
  }else{
    Serial.println("Inertial Sensor present");
  }
  delay(1000);
//  displaySensorDetails();
//  Serial.println("Display sensor details");
//  displaySensorStatus();
//  Serial.println("display sensor status");

  Serial.println("bno calibrated");
  //bno.setExtCrystalUse(true);
}

void loop() {
 
  sensors_event_t event;
  bno.getEvent(&event);
  uint8_t temp = bno.getTemp();
   imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);  
   if(bno.isFullyCalibrated()){
    digitalWrite(2,HIGH);
    digitalWrite(0,LOW);
   }else{
    digitalWrite(0,HIGH);
    digitalWrite(2,LOW);
   }
   loopcount = loopcount + 1;
  if(loopcount%7 == 1){
   
    
    File Accelerometer = SD.open("accellog.txt", FILE_WRITE);
    if (Accelerometer) {
    Accelerometer.println(getString(accel));
    Accelerometer.close();
    // print to the serial port too:
    Serial.println("A: ["+getString(accel)+"]");
  }else {
    // if the file didn't open, print an error:
    Serial.println("error opening accellog.txt");
  }
  }else  if(loopcount%7 == 2){
    imu::Vector<3> lin_accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL); 

    File LinearAccel = SD.open("lalog.txt",FILE_WRITE);
    if (LinearAccel) {
        LinearAccel.println(getString(lin_accel));
       
        // print to the serial port too:
        Serial.println("LA: [" + getString(lin_accel) + "]");
        LinearAccel.close();
      }else {
          // if the file didn't open, print an error:
          Serial.println("error opening lin_accellog.txt");
      }
  }else if(loopcount%7 == 3){
     imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);  

     File Gravity = SD.open("gravlog.txt",FILE_WRITE);
     if (Gravity) {
         Gravity.println(getString(grav));
         // print to the serial port too:
         Serial.println("G: ["+getString(grav)+"]");
         Gravity.close();
      }else {
        // if the file didn't open, print an error:
        Serial.println("error opening gravlog.txt");
      }
      
  }else if(loopcount%7 == 4){
    imu::Vector<3> w_vel = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    File W_velocity = SD.open("w_vellog.txt",FILE_WRITE);
    if (W_velocity) {
      W_velocity.println(getString(w_vel));
      // print to the serial port too:
      Serial.println("WV: ["+getString(w_vel)+"]");
      W_velocity.close();
     }else {
      // if the file didn't open, print an error:
      Serial.println("error opening w_vellog.txt");
     }
   }else  if(loopcount%7 == 5){
    imu::Vector<3> magnet = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      File Magnet = SD.open("maglog.txt",FILE_WRITE);
      if (Magnet) {
        Magnet.println(getString(magnet));
        // print to the serial port too:
        Serial.println("M: [" + getString(magnet) + "]");
        Magnet.close();
      }else {
        // if the file didn't open, print an error:
        Serial.println("error opening maglog.txt");
      }
  }else  if(loopcount%7 == 6){
    
    File Temperature = SD.open("templog.txt",FILE_WRITE);
      if(Temperature){
        Temperature.println(String(temp));
        Serial.println("T: "+String(temp));
        Temperature.close();
      }else{
        Serial.println("Error opening templog.txt");
      }
  }else if(loopcount%7 == 0){
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
        Serial.println("O");
      }
  }
  
    
  Serial.println("---");
  
  delay(100);
}
//void displaySensorDetails(void)
//{
//  sensor_t sensor;
//  bno.getSensor(&sensor);
//  File Log = SD.open("log.txt", FILE_WRITE);
//  Log.println("------------------------------------");
//  Log.print  ("Sensor:       "); Log.println(sensor.name);
//  Log.print  ("Driver Ver:   "); Log.println(sensor.version);
//  Log.print  ("Unique ID:    "); Log.println(sensor.sensor_id);
//  Log.print  ("Max Value:    "); Log.print(sensor.max_value); Log.println(" xxx");
//  Log.print  ("Min Value:    "); Log.print(sensor.min_value); Log.println(" xxx");
//  Log.print  ("Resolution:   "); Log.print(sensor.resolution); Log.println(" xxx");
//  Log.println("------------------------------------");
//  Log.println("");
//  Log.close();
//  delay(500);
//}
//void displaySensorStatus(void)
//{
//  /* Get the system status values (mostly for debugging purposes) */
//  uint8_t system_status, self_test_results, system_error;
//  system_status = self_test_results = system_error = 0;
//  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
//  File Log = SD.open("log.txt", FILE_WRITE);
//  /* Display the results in the Serial Monitor */
//  Log.println("");
//  Log.print("System Status: 0x");
//  Log.println(system_status, HEX);
//  Log.print("Self Test:     0x");
//  Log.println(self_test_results, HEX);
//  Log.print("System Error:  0x");
//  Log.println(system_error, HEX);
//  Log.println("");
//  Log.close();
//  delay(500);
//}
/* Takes vector output and makes it a nice string to look at, may change it to a row vector but whatever
 * 
 */
String getString(imu::Vector<3> vec){
  String res = "X: " + String(vec.x()) + " Y: " + String(vec.y()) + " Z: " + String(vec.z());
  //Serial.println(res);
  return res;
}












