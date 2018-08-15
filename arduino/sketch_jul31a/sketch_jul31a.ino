#include <ros.h>
#include <std_msgs/Time.h>

int count = 0;

const int camera_pin = 3;
const int imu_pin = 2;

void SensorTriggering (int pinnumber ){
  unsigned long currentTime = millis();
  Serial.write(currentTime);
  digitalWrite(pinnumber, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinnumber, LOW);
}


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(camera_pin, OUTPUT);
  pinMode(imu_pin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (count % 500 == 0){
    SensorTriggering(imu_pin);
  }
  if (count % 30 == 0){
    SensorTriggering(camera_pin);
  }
}
