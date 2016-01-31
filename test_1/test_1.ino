int analogSensor = 0;
int val = 0;
int count = 0;
void setup() {
  // put your setup code here, to run once:
Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
String data = "";
val = analogRead(analogSensor);
data += ","+String(val);
count++;
if(count==100){
  Serial.println(data);
}
}
