#include <IMU_VN100.h>

void setup() {
  // put your setup code here, to run once:
  IMU_Class::Init();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  IMU_Class::Read();
  IMU_Class::parse_IMU();
}

