#include <Wire.h>
#include <LiquidCrystal.h>
#include <math.h>

#define ACC_ADDR 0x53
#define GYR_ADDR 0x69
#define MAG_ADDR 0x1E
#define BAR_ADDR 0x77


void setup() {
  // put your setup code here, to run once:
Wire.begin();
Serial.begin(115200);

}




void loop() {
  // put your main code here, to run repeatedly:

}

void intIMU(){

//GYRO data
  gyroWriteI2C(0x20,0x1F); //turns on GYRO from wakeup in CTRL_REG1
  gyroWriteI2C(0x22,0x08); //enables data to be sent when ready
  gyroWriteI2C(0x23,0x80); //set gyro sensitivity to 500 deg/s
  
 
}


int gyroWriteI2C( byte regAddr, byte val){
  Wire.beginTransmission(GYR_ADDR);
  Wire.write(regAddr);
  Wire.write(val);
  Wire.endTransmission();
}
