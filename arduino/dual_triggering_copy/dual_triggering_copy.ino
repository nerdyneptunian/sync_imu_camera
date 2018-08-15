#include <ros.h>
#include <std_msgs/Byte.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::Byte byte_msg;
std_msgs::String msg;
ros::Publisher serialdata("serialdata", &msg);


//Defines the pins of each sensor's Sync_In pin
const int camera_pin = 3;
const int imu_pin = 2;

//Sets the rate of triggering of each sensor
const int max_camera_rate = 30; //Hz
const int max_imu_rate = 400; //Hz

unsigned long previousTimeImu = 0;
unsigned long previousTimeCamera = 0;
unsigned long currentTime;

//Time between triggers based on rates (in millis)
unsigned long camera_interval = 1000 / max_camera_rate;
unsigned long imu_interval = 1000 / max_imu_rate;

//Triggers each sensor
void SensorTriggering (int pinnumber){
  //Serial.write(currentTime);
  digitalWrite(pinnumber, HIGH);
  delayMicroseconds(100);
  digitalWrite(pinnumber, LOW);
}


void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  nh.initNode();
  nh.advertise(serialdata);
  pinMode(camera_pin, OUTPUT);
  pinMode(imu_pin, OUTPUT);
}

void loop() {

  currentTime = millis();
  if (currentTime - previousTimeCamera >= camera_interval) {
    SensorTriggering(camera_pin);
    previousTimeCamera = currentTime;
  }

  //if (currentTime - previousTimeImu >= imu_interval) {
  //  SensorTriggering(imu_pin);
  //  previousTimeImu = currentTime;
    //char c = Serial.read();
    //Serial.print(c);
    //if (c == '$'){
    // msg.data =  "it works";
    // serialdata.publish(&msg);
    //}
  //}

  nh.spinOnce();
}
