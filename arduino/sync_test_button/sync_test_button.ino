//Defines the pins of each sensor's Sync_In pin
const int camera_pin = 3;
const int imu_pin = 2;
const int buttonApin = 9;

//Sets the rate of triggering of each sensor
const int max_camera_rate = 1; //Hz
const int max_imu_rate = 1; //Hz

unsigned long previousTimeImu = 0;
unsigned long previousTimeCamera = 0;
unsigned long currentTime;

//Time between triggers based on rates (in millis)
unsigned long camera_interval = 1000 / max_camera_rate;
unsigned long imu_interval = 1000 / max_imu_rate;

//Triggers each sensor
void SensorTriggering (int pinnumber){
  digitalWrite(pinnumber, HIGH);
  delayMicroseconds(100);
  digitalWrite(pinnumber, LOW);
}


void setup() {

  pinMode(camera_pin, OUTPUT);
  pinMode(imu_pin, OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(buttonApin, INPUT_PULLUP);
}

void loop() {

  if (digitalRead(buttonApin) == LOW){
    currentTime = millis();
    digitalWrite(5, HIGH);
  
    if (currentTime - previousTimeCamera >= camera_interval) {
      SensorTriggering(camera_pin);
      previousTimeCamera = currentTime;
    }

    if (currentTime - previousTimeImu >= imu_interval) {
      SensorTriggering(imu_pin);
      previousTimeImu = currentTime;
    }
  }

  else{
    digitalWrite(5, LOW);
  }
}
