#include <Wire.h>
#include <math.h>


// parameters which can be changed if you know what you are doing
float periodESC = 4000; //[ns] pulse period of ESC signal, thus also the main loop period
float dt = 0.004;

float timerMain;

int addrIMU = 0x68;
unsigned char GYR[3], ACC[3];
float TEMP;
int test;
unsigned char zerox, zeroy, zeroz;
float pitch_acc,yaw_acc,roll_acc,pitch_gyr=0,yaw_gyr=0,roll_gyr=0,yawDot_gyr,pitchDot_gyr,rollDot_gyr;

#define WAKE_REG 0x6B
#define ACC_REG  0x3B
#define IMU_ADDR 0x68

void initIMU(){
  //sets communtiotion towards IMU and sends wake up call;
  Wire.begin();
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(WAKE_REG);
  Wire.write(0x00);
  Wire.endTransmission();

  //set gyro configuration
  Wire.beginTransmission(IMU_ADDR); //GYRO_CONFIG address
  Wire.write(0x1b);
  Wire.write(0x08); //FS_SEL=1 -> range 65.5 deg/s
  Wire.endTransmission();

  //set acc configuration
  Wire.beginTransmission(IMU_ADDR); //GYRO_CONFIG address
  Wire.write(0x1c);
  Wire.write(0x16); //AFS_SEL=2 -> range 4096 1/g
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

void zero_gyro(){
  
  float points = 500;
  getIMUData();

  Serial.println("Calibrating gyro...");
  for (int i=1;i<=points;i++){
    getIMUData();
    zerox = zerox + GYR[0];
    zeroy = zeroy + GYR[1];
    zeroz = zeroz + GYR[2];
  }

  zerox = zerox/points/32.8;
  zeroy = zeroy/points/32.8;
  zeroz = zeroz/points/32.8;
}

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);

  //IMU wake up call
  initIMU();
  zero_gyro();
  

  
  timerMain = micros(); //start of loop timer  
}



void loop() {
  int avg = 0;
  
  //reads data from IMU
  getIMUData(); 

  if (avg == 1){
    roll_acc  = roll_acc  *0.8 + (0.2*  (acos(double(ACC[0])/4096)-0.5*3.14159)*(180/3.14159));
    pitch_acc = pitch_acc *0.8 + (0.2* -(acos(double(ACC[1])/4096)-0.5*3.14159)*(180/3.14159));  
  }
  else {
    roll_acc  =  (acos(double(ACC[0])/4096)-0.5*3.14159)*(180/3.14159);
    pitch_acc = -(acos(double(ACC[1])/4096)-0.5*3.14159)*(180/3.14159);
  }

  roll_gyr   += (GYR[0]/32.8)*dt;
  pitch_gyr  += (GYR[1]/32.8)*dt;  
  yaw_gyr    += (GYR[2]/32.8)*dt;

  //Serial.println(yaw_gyr);
  Serial.println(yaw_gyr);
  //Serial.println(ACC[0]);

  
  //Serial.print("Pitch = ");
  //Serial.print(pitch_acc);
  //Serial.print(", Roll = ");
  //Serial.println(roll_acc);

  //Serial.print("gyro x = ");
  //Serial.print(", gyro y = ");
  //Serial.print(GYR[1]-zeroy);
  //Serial.print(", gyro z = ");
  //Serial.println(GYR[2]-zeroz);

  
  

  while(micros()-timerMain < periodESC); // end of loop timer, wait for periodESC to pass
  timerMain = micros();  

  
}
