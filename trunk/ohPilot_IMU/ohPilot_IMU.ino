#include <Wire.h>
#include <math.h>

int addrIMU = 0x68;
int GYR[3], ACC[3];
int TEMP;
double pitch,yaw,roll;

#define WAKE_REG 0x6B
#define ACC_REG 0x3B
#define IMU_ADDR 0x68

void initIMU(){
  //sets communtiotion towards IMU and sends wake up call;
  Wire.begin();
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(WAKE_REG);
  Wire.write(0x00);
  Wire.endTransmission();
}

void getIMUData(){
  //function for reading IMU 6050
  
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(ACC_REG);
  Wire.endTransmission();
  
  Wire.requestFrom(IMU_ADDR,14);

  if(Wire.available()<=14){
    ACC[0] = Wire.read()<<8|Wire.read(); //ACC_X
    ACC[1] = Wire.read()<<8|Wire.read(); //ACC_Y
    ACC[2] = Wire.read()<<8|Wire.read(); //ACC_Z
    TEMP   = Wire.read()<<8|Wire.read(); //TEMPERATURE
    GYR[0] = Wire.read()<<8|Wire.read(); //GYRO_X
    GYR[1] = Wire.read()<<8|Wire.read(); //GYRO_Y
    GYR[2] = Wire.read()<<8|Wire.read(); //GYRO_Z  
  }  

    
}


void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);

  //IMU wake up call
  initIMU();
    
}

void loop() {
  //reads data from IMU
  getIMUData();

  int xacc,yacc,zacc;

  xacc = ACC[0]*0.005493164;
  yacc = ACC[1]*0.005493164;
  zacc = ACC[2]*0.005493164; 

  roll  =  (acos(double(ACC[0])/16384)-0.5*3.14159)*(180/3.14159);
  pitch = -(acos(double(ACC[1])/16384)-0.5*3.14159)*(180/3.14159);
  //if (isnan(roll)) roll = 90;
  
  Serial.println(pitch);


  delay(50);

}
