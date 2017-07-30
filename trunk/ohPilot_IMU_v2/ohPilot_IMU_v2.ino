
#include <Wire.h>
#include <math.h>

//main loop parameters
int timerMain; 
int periodESC = 100; //[ms] loop timer


//parameters for IMU
float GYR_pitch, GYR_roll, GYR_yaw, ACC_X, ACC_Y, ACC_Z, TEMP; //raw values from reading IMU modules
float PITCH=0, ROLL=0, YAW=0, PITCH_GYR=0, ROLL_GYR=0, YAW_GYR=0, PITCH_ACC = 0, ROLL_ACC = 0, Z_ACC = 0; //IMU determined angles
float zeroPitch=0, zeroRoll=0, zeroYaw=0;
int calIMU=0; //status indicator if imu is calibrated

// if you want to calibrate the accelorometer place the sensors on a level surface. Set calACC to 1 and change zeroACCx,
// zeroACCy, zeroACCz with the zero values 
int calACC = 0;
float zeroACCx = 0.0581968784; //[g]
float zeroACCy = 0.0043075246; //[g]
float zeroACCz = 1.0391490459; //[g]


//scaling of raw IMU data towards usefull information
float gyrScale = 65.5, accScale = 4096;

#define IMU_ADDR 0x68
#define WAKE_REG 0x6b
#define ACC_REG  0x3b


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //-----setup for arduino ports------
  DDRD = DDRD | B11110000;  //set ports 4,5,6,7 to inputs;    
  DDRB = DDRB | B00010000;  //set ports 8,9,10,11 to inputs and 12 to output; 

  //-----setup for IMU-----
  initIMU();      //seting up IMU i2c communication and settings
  calibrateIMU(); //calibrating IMU gyro

  //-----start of loop timer-----
  //timerMain = micros(); 
  timerMain = millis();
}

void loop() {
  // put your main code here, to run repeatedly:

  getIMUData();



  //absolute vector of Z_ACC
  Z_ACC = sqrt((ACC_X*ACC_X)+(ACC_Y*ACC_Y)+(ACC_Z*ACC_Z));



  if(abs(ACC_Y) < Z_ACC)    PITCH_ACC = asin((float)ACC_Y/Z_ACC)* 57.296; //prevents NaN values  
  if(abs(ACC_X) < Z_ACC)    ROLL_ACC  = asin((float)ACC_X/Z_ACC)*-57.296; //prevents NaN values
  
  
  //ROLL_ACC  = 90-(acos(ACC_X/zeroACCz)*(180/PI)); 
  //PITCH_ACC = 90-(acos(ACC_Y/zeroACCz)*(180/PI)); 

  // -----integration of angular rates to calculate angles-----
  YAW_GYR   += GYR_yaw  /(1000/periodESC);   //1000/periodESC if millis() is used for loop counter, 1000000/periodESC for micros()
  PITCH_GYR -= GYR_pitch/(1000/periodESC);
  ROLL_GYR  += GYR_roll /(1000/periodESC);

  YAW   = YAW_GYR;
  PITCH = 0.996*PITCH_GYR + 0.004*PITCH_ACC;
  ROLL  = 0.996*ROLL_GYR  + 0.004*ROLL_ACC;
  
  
  Serial.println(PITCH_GYR);
  
  
  //delay(100);
  //while(micros()-timerMain < periodESC); // end of loop timer, wait for periodESC to pass
  //timerMain = micros();  
  //while(millis()-timerMain < periodESC); // end of loop timer, wait for periodESC to pass
  //timerMain = millis();  

  delay(periodESC);
}


void getIMUData(){
  //function for reading IMU 6050
  
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(ACC_REG);
  Wire.endTransmission();
  
  Wire.requestFrom(IMU_ADDR,14);

  if(Wire.available()<=14){
    ACC_X     = Wire.read()<<8|Wire.read(); //ACC_X
    ACC_Y     = Wire.read()<<8|Wire.read(); //ACC_Y
    ACC_Z     = Wire.read()<<8|Wire.read(); //ACC_Z
    TEMP      = Wire.read()<<8|Wire.read(); //TEMPERATURE
    GYR_pitch = Wire.read()<<8|Wire.read(); //GYRO_X
    GYR_roll  = Wire.read()<<8|Wire.read(); //GYRO_Y
    GYR_yaw   = Wire.read()<<8|Wire.read(); //GYRO_Z  
  }  

  

  if (calIMU == 1){
    ACC_X = (ACC_X / accScale) - zeroACCx;
    ACC_Y = (ACC_Y / accScale) - zeroACCy;
    ACC_Z = (ACC_Z / accScale) - zeroACCz;
    GYR_roll  = (GYR_roll /gyrScale) - zeroRoll; //raw output of gyro in [deg/s]
    GYR_pitch = (GYR_pitch/gyrScale) - zeroPitch;
    GYR_yaw   = (GYR_yaw  /gyrScale) - zeroYaw;
  }
  else
  {
    ACC_X = ACC_X / accScale;
    ACC_Y = ACC_Y / accScale;
    ACC_Z = ACC_Z / accScale;
    GYR_roll  = GYR_roll /gyrScale; //raw output of gyro in [deg/s]
    GYR_pitch = GYR_pitch/gyrScale;
    GYR_yaw   = GYR_yaw  /gyrScale;
  }
    
}

void calibrateIMU(){
  float points = 2500;
  int   stateLED = 0;
  digitalWrite(12,HIGH);
  getIMUData(); 
  
  Serial.println("Calibrating gyro...");
  for (int i=1;i<=points;i++){
    getIMUData();
    zeroPitch = zeroPitch + GYR_pitch;
    zeroRoll  = zeroRoll  + GYR_roll;
    zeroYaw   = zeroYaw   + GYR_yaw;
    //when you want to calibrate the accelorometer
    if (calACC == 1){
      zeroACCx = zeroACCx + ACC_X;
      zeroACCy = zeroACCy + ACC_Y;
      zeroACCz = zeroACCz + ACC_Z;
    }
    // turn on/off status light every 100 points
    if (i % 100 == 0){
      digitalWrite(12, (stateLED) ? HIGH : LOW);
      stateLED = !stateLED;
    }
  }

  zeroPitch = zeroPitch/points;
  zeroRoll  = zeroRoll /points;
  zeroYaw   = zeroYaw  /points;

  //when you want to calibrate the accelorometer
  if (calACC == 1){
     zeroACCx = zeroACCx/points;
     zeroACCy = zeroACCy/points;
     zeroACCz = zeroACCz/points;
     Serial.print("zero off ACCx = ");     
     Serial.println(zeroACCx,10);
     Serial.print("zero off ACCy = ");     
     Serial.println(zeroACCy,10);
     Serial.print("zero off ACCz = ");     
     Serial.println(zeroACCz,10);             
  }




  digitalWrite(12,LOW);
  calIMU = 1;
}



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

