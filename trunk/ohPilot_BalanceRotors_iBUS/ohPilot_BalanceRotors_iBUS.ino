 // Main ohPilot script
#include <Wire.h>
//#include <LiquidCrystal.h> //for debugging purposes
#include <math.h>
#include "FlySkyIBus.h"
#include <EEPROM.h>


int motor = 0;

long motorNoise;
// parameters which can be changed if you know what you are doing
long periodESC = 4000; //pulse period of ESC signal, thus also the main loop period
float scaleRC = 15.0; //influences maximum angle of quad (RATE), angle = 500/scale RC, 15->33.33 deg 14->35.71 13->38.45 12->41.667 11->45.4545deg

// PID settings;
float P_pitch = 1.0;
float I_pitch = 0.1;
float D_pitch = 0.1;

float P_roll = P_pitch;
float I_roll = I_pitch;
float D_roll = D_pitch;

float P_yaw = 1.0;
float I_yaw = 0.0;
float D_yaw = 0.0;

long maxOutPitch = 300;
long maxOutRoll  = 300;
long maxOutYaw   = 300;

static int setPIDs = 1; //routine to config pids during flight
static int chNobInc  = 5; //channel for incrementation nob
static int chNobDec  = 6; //channel for incrementation nob
static int chSwchPID = 9; //channel for threeway switch
int K=0; 
int kInc=50; 
float dP_pitch=50.0,dI_pitch=50.0,dD_pitch=50.0;
int saveEEPROM = 1;


// parameters which should not be changed, parameters for sending and recieving motor signals
unsigned long t0, timerPin4, timerPin5, timerPin6, timerPin7, timerPin8, timerPin9, timerPin10, timerPin11, timerPins, RECIEVER[10], ESCOUT[4], timerESC;
long timerMain;
int tTest;
int tprev = 0;
int pin8 = 0,pin9 = 0,pin10 = 0,pin11 = 0;
//int pin4,pin5,pin6,pin7;
int motorStart = 0;
long THROTTLE_RC, ROLL_RC, PITCH_RC, YAW_RC; //keep as integer or long
float anglePitchRC=0,angleRollRC=0,angleYawRC=0; //input angles from RC

//PID parameters
float errorPitch, errorYaw, errorRoll;
float prevErrorPitch=0,prevErrorRoll=0,prevErrorYaw=0;
float errorPitchCum=0, errorYawCum=0, errorRollCum=0;
float errorPitchDiv, errorYawDiv, errorRollDiv;
long  PIDoutPitch, PIDoutRoll, PIDoutYaw, refPITCH, refROLL, refYAW;
float PIDoutPitchAngle, PIDoutRollAngle, PIDoutYawAngle;


//parameters for IMU
float GYR_pitch, GYR_roll, GYR_yaw, ACC_X, ACC_Y, ACC_Z, TEMP; //raw values from reading IMU modules
float PITCH, ROLL, YAW, PITCH_GYR, ROLL_GYR, YAW_GYR, PITCH_ACC, ROLL_ACC, Z_ACC; //IMU determined angles
float zeroX, zeroY, zeroZ;
int calIMU=0; //status indicator if imu is calibrated

//accelerometer zero offsets
float zeroACCx = 0.1;
float zeroACCy = -3.0;
float zeroACCz = 0.0;

#define IMU_ADDR 0x68
#define WAKE_REG 0x6B
#define ACC_REG  0x3B


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  IBus.begin(Serial);
  
  //-----setup for arduino ports------   
  //set ports 4,5,6,7 to outputs
  DDRD = DDRD | B11110000;
  //set ports 8,9,10,11 to inputs and 12 to output;  
  DDRB = DDRB | B00100000;  
  digitalWrite(13,HIGH); //turns on STATUS LED

  Wire.begin();
  
  //-----configure interrupts-----
  //PCICR  |= (1<<PCIE0); //sets Pin change interrupt control register to activate PCMSK0 register
  //PCMSK0 |= (1<<PCINT0); //config pin interrupt on pin 8
  //PCMSK0 |= (1<<PCINT1); //config pin interrupt on pin 9
  //PCMSK0 |= (1<<PCINT2); //config pin interrupt on pin 10
  //PCMSK0 |= (1<<PCINT3); //config pin interrupt on pin 11 
  
  //-----setup for IMU-----
  initIMU();      //seting up IMU i2c communication and settings
  calibrateIMU(); //calibrating IMU gyro

  
  digitalWrite(13,LOW);
  //-----start of loop timer-----
  timerMain = micros();  // start of loop timer 
}

void loop() {
  // put your main code here, to run repeatedly:  
  
  getTXInputs();  //reading iBUS inputs
  getIMUAngles();

  if (setPIDs == 1){
    tunePIDs();
  }

  //
  Serial.print(K);
  Serial.print(", ");
  Serial.print(RECIEVER[4]);
  Serial.print(", ");
  Serial.print(RECIEVER[5]);
  Serial.print(", ");
  Serial.print(P_pitch);
  Serial.print(", ");
  Serial.print(I_pitch);
  Serial.print(", ");
  Serial.println(D_pitch);
  

  //check input signals
//  Serial.print(RECIEVER[0]);
//  Serial.print(", ");
//  Serial.print(RECIEVER[1]);
//  Serial.print(", ");
//  Serial.print(RECIEVER[2]);
//  Serial.print(", ");
//  Serial.print(RECIEVER[3]);
//  Serial.print(", ");
//  Serial.print(RECIEVER[4]);
//  Serial.print(", ");
//  Serial.print(RECIEVER[5]);
//  Serial.print(", ");
//  Serial.print(RECIEVER[6]);
//  Serial.print(", ");
//  Serial.print(RECIEVER[7]);
//  Serial.print(", ");
//  Serial.print(RECIEVER[8]);
//  Serial.print(", ");
//  Serial.println(RECIEVER[9]);



  

  //=====Motor start routine, sets flag and PID errors to zero=====
  if ((RECIEVER[2] <= 1020) && (RECIEVER[3] > 1980)){
    motorStart = 1; //turns on motors
    //sets PID error integration to zero
    errorPitchCum = 0;
    errorRollCum  = 0;
    errorYawCum   = 0;
    prevErrorPitch = 0;
    prevErrorRoll  = 0;
    prevErrorYaw   = 0;
  }
  if ((RECIEVER[2] <= 1020) && (RECIEVER[3] < 1020)) motorStart = 0; //turns off motors
  if (RECIEVER[2] <= 1020) RECIEVER[3] = 1500; // disables yaw if throttle is zero

  //=====dead bands for stable zero RC input=====
  if((RECIEVER[0] < 1520) && (RECIEVER[0] > 1480)) RECIEVER[0] = 1500; //roll
  if((RECIEVER[1] < 1520) && (RECIEVER[1] > 1480)) RECIEVER[1] = 1500; //pitch
  if((RECIEVER[3] < 1520) && (RECIEVER[3] > 1480)) RECIEVER[3] = 1500; //yaw


  //=====Converts RC signals to angles=====
  anglePitchRC = -((long)RECIEVER[1]-1500)/scaleRC; //= reference for PID
  angleRollRC  =  ((long)RECIEVER[0]-1500)/scaleRC; //= reference for PID
  angleYawRC   =  ((long)RECIEVER[3]-1500)/scaleRC;
  
  //Serial.print("Roll == ");
  //Serial.print(angleRollRC);
  //Serial.print(", Pitch == ");
  //Serial.print(anglePitchRC);
  //Serial.print(", Yaw == ");
  //Serial.print(angleYawRC);   
  //Serial.print(", Throttle == ");
  //Serial.println(RECIEVER[2]);



    if (motorStart == 1){      
      //value of throttle its zero position;
      THROTTLE_RC = RECIEVER[2];      
      if (THROTTLE_RC > 1900) THROTTLE_RC = 1900;     //set maximum to throttle, otherwise PID has no room for corrections
                  
      //set inputs for PID, completentary filter can be used
      refYAW   = refYAW   * 0.0 + 1.0 * (GYR_yaw/65.5); //yaw refernce is yaw angular rate
      refPITCH = refPITCH * 0.0 + 1.0 * PITCH;          //pitch ref is pitch angle
      refROLL  = refROLL  * 0.0 + 1.0 * ROLL;           //rol ref is roll angle

      //PID();
      
      motorNoise = motorNoise *0.95 + 0.05 * (abs(GYR_yaw) );      
      Serial.println(motorNoise);
            
      //sets output to ESCs
      ESCOUT[0] = THROTTLE_RC+PIDoutPitch-PIDoutRoll+PIDoutYaw; //motor1, front right
      ESCOUT[1] = THROTTLE_RC-PIDoutPitch-PIDoutRoll-PIDoutYaw; //motor2, rear right
      ESCOUT[2] = THROTTLE_RC-PIDoutPitch+PIDoutRoll+PIDoutYaw; //motor3, rear left
      ESCOUT[3] = THROTTLE_RC+PIDoutPitch+PIDoutRoll-PIDoutYaw; //motor4, front left

      //sets minimum value to 1060ms
      if (ESCOUT[0] < 1060) ESCOUT[0] = 1000;
      if (ESCOUT[1] < 1060) ESCOUT[1] = 1000;
      if (ESCOUT[2] < 1060) ESCOUT[2] = 1000; //keeps engines running
      if (ESCOUT[3] < 1060) ESCOUT[3] = 1000;

      //sets maximum value to 2000ms
      if (ESCOUT[0] > 1140) ESCOUT[0] = 1140;
      if (ESCOUT[1] > 1140) ESCOUT[1] = 1140;
      if (ESCOUT[2] > 1140) ESCOUT[2] = 1140;
      if (ESCOUT[3] > 1140) ESCOUT[3] = 1140;

      switch (motor){
        case 1:
          ESCOUT[1] = 1000;
          ESCOUT[2] = 1000;
          ESCOUT[3] = 1000; 
          break;
        case 2:
          ESCOUT[0] = 1000;
          ESCOUT[2] = 1000;
          ESCOUT[3] = 1000; 
          break;
        case 3:
          ESCOUT[0] = 1000;
          ESCOUT[1] = 1000;
          ESCOUT[3] = 1000; 
          break; 
        case 4:
          ESCOUT[0] = 1000;
          ESCOUT[1] = 1000;
          ESCOUT[2] = 1000; 
          break;  
        
      }


    }
    else{ //if motor are not running
      ESCOUT[0] = 1000;
      ESCOUT[1] = 1000;
      ESCOUT[2] = 1000;
      ESCOUT[3] = 1000;
    }
    
    while(micros()-timerMain < periodESC); // end of loop timer, wait for periodESC to pass
    timerMain = micros();  
  
    PORTD |= B11110000; //set pins to high (motor 1,2,3,4) HIGH   
    timerPin4 = ESCOUT[0] + timerMain;
    timerPin5 = ESCOUT[1] + timerMain;
    timerPin6 = ESCOUT[2] + timerMain;
    timerPin7 = ESCOUT[3] + timerMain;


    while (PORTD >= 16){
      timerESC = micros();
      if (timerPin4 <= timerESC){      
        PORTD &= B11101111;      //set pin 4 (motor 1) LOW  
                  
      }
      if (timerPin5 <= timerESC){      
        PORTD &= B11011111;      //set pin 5 (motor 1) LOW      
            
      }
      if (timerPin6 <= timerESC){      
        PORTD &= B10111111;      //set pin 6 (motor 1) LOW     
                
      }
      if (timerPin7 <= timerESC){      
        PORTD &= B01111111;      //set pin 7 (motor 1) LOW 
              
      }
    }
}

//===== PID controller =====
void PID(){
//-----PID for pitch, with angular data-----
  //error = reference - sensor;
  errorPitch     = anglePitchRC - refPITCH;
  errorPitchCum += errorPitch;
  errorPitchDiv  = errorPitch-prevErrorPitch;

  PIDoutPitchAngle = (P_pitch*errorPitch) + (I_pitch * errorPitchCum) + (D_pitch * errorPitchDiv);

  prevErrorPitch = errorPitch;
  
  //conver to ms signal
  PIDoutPitch = PIDoutPitchAngle*scaleRC; 
  //limits output to 200ms
  if (PIDoutPitch >  maxOutPitch)       PIDoutPitch = maxOutPitch;
  if (PIDoutPitch <  (-1*maxOutPitch))  PIDoutPitch = -1*maxOutPitch;  

//-----PID for roll, with angular data-----
  //error = reference - sensor;
  errorRoll     = angleRollRC - refROLL;
  errorRollCum += errorRoll;
  errorRollDiv  = errorRoll-prevErrorRoll;

  PIDoutRollAngle = (P_roll*errorRoll) + (I_roll * errorRollCum) + (D_roll * errorRollDiv);

  prevErrorRoll = errorRoll;
  
  //conver to ms signal
  PIDoutRoll = PIDoutRollAngle*scaleRC; 
  //limits output to 200ms
  if (PIDoutRoll >  maxOutRoll)       PIDoutRoll = maxOutRoll;
  if (PIDoutRoll <  (-1*maxOutRoll))  PIDoutRoll = -1*maxOutRoll;  

//-----PID for yaw, with angular rates data-----
  //error = reference - sensor;
  errorYaw     = angleYawRC - refYAW;
  errorYawCum += errorYaw;
  errorYawDiv  = errorYaw-prevErrorYaw;

  PIDoutYawAngle = (P_yaw*errorYaw) + (I_yaw * errorYawCum) + (D_yaw * errorYawDiv);

  prevErrorYaw = errorYaw;
  
  //conver to ms signal
  PIDoutYaw = PIDoutYawAngle*scaleRC; 
  //limits output to 200ms
  if (PIDoutYaw >  maxOutYaw)       PIDoutYaw = maxOutYaw;
  if (PIDoutYaw <  (-1*maxOutYaw))  PIDoutYaw = -1*maxOutYaw;  
  
}

//===== set PIDs during void setup, reads from EEPROM ====
void initPIDs(){
    
}

//===== set PIDs during flight with turning nobs and three way switch =====
void tunePIDs(){

  //store to eemprom if nobs are off. Operation done once
  if ((RECIEVER[chNobInc-1] < 1050) && (RECIEVER[chNobDec-1] < 1050) && (saveEEPROM == 0)){
    
    saveEEPROM = 1;
  }
  else{
    saveEEPROM = 0;
  }
  
  
  //tune either Kp, Ki or Kd
  if (RECIEVER[chSwchPID-1] > 1950){
    K = 2; //tune D
  }
  else if ((RECIEVER[chSwchPID-1] > 1450) && (RECIEVER[chSwchPID-1] < 1550)) {
    K = 1; //tune I
  }
  else if (RECIEVER[chSwchPID-1] < 1050) {
    K = 0; //tune P
  }

  //
  if (RECIEVER[chNobInc-1] > 1050){
    kInc = (2010-RECIEVER[chNobInc-1])*100;


    switch (K){
      case 0:{
      
        if (kInc != 0){
          dP_pitch = P_pitch/kInc;
        }
        P_pitch += dP_pitch;  
        break; 
      }  
      case 1:{
        if (kInc != 0){
          dI_pitch = I_pitch/kInc;
        }
        I_pitch += dI_pitch;  
        break; 
      }
      case 2:{
        if (kInc != 0){
          dD_pitch = D_pitch/kInc;
        }
        D_pitch += dD_pitch;  
        break; 
      }         
    }     
  }

  
  if (RECIEVER[chNobDec-1] > 1050){
    kInc = (2010-RECIEVER[chNobDec-1])*100;

    switch (K){
      case 0:{
        if (kInc != 0){
          dP_pitch = P_pitch/kInc;
        }
        P_pitch -= dP_pitch; 
        break;   
      }  
      case 1:{
        if (kInc != 0){
          dI_pitch = I_pitch/kInc;
        }
        I_pitch -= dI_pitch;  
        break; 
      }
      case 2:{
        if (kInc != 0){
          dD_pitch = D_pitch/kInc;
        }
        D_pitch -= dD_pitch; 
        break;    
      }       
    }   
  }
  
  //  
  

  
}


//===== Fuction for calculating IMU angles from raw data=====
void getIMUAngles(){
  // get raw data
  getIMUData();
  
  //correcting for zero offset of gyro. Offset is determined during calibration  
  GYR_pitch -= zeroX; //pitch
  GYR_roll  -= zeroY; //roll
  GYR_yaw   -= zeroZ; //yaw

  //gyro angles (integration)
  PITCH += GYR_pitch* 0.000061069; 
  ROLL  += GYR_roll * 0.000061069; //(1/250)/65.5  

  //gyro yaw angular rate
  YAW   -= GYR_yaw  * 0.000061069;

  //couple gyro axes
  PITCH += ROLL  * sin(GYR_yaw * 0.0000010658); //((1/250)/65.5) * (3.14159/180)
  ROLL  -= PITCH * sin(GYR_yaw * 0.0000010658);

  //Angles from accelerometer
  Z_ACC     = sqrt((ACC_X*ACC_X)+(ACC_Y*ACC_Y)+(ACC_Z*ACC_Z));
  PITCH_ACC = asin((float)ACC_Y/Z_ACC)*  57.296;
  ROLL_ACC  = asin((float)ACC_X/Z_ACC)* -57.296;

  //zero offset correction for accelerometer
  PITCH_ACC -= zeroACCx; 
  ROLL_ACC  -= zeroACCy;
  
  if (calIMU==1){        
    //complementary filter
    PITCH = PITCH * 0.9996 + PITCH_ACC * 0.0004;
    ROLL  = ROLL  * 0.9996 + ROLL_ACC  * 0.0004;
  } 
  else{
    //for first run, angles equal to angles of accelerometer
    PITCH = PITCH_ACC;
    ROLL  = ROLL_ACC;
    calIMU = 1;
  }
}

//===== Fuction for reading MPU6050 raw data=====
void getIMUData(){  
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
}

//===== Fuction for calibrating the IMU at the start (void setup)=====
void calibrateIMU(){
  
  int points = 2500;
  int stateLED = 0;
  digitalWrite(12,HIGH);
  getIMUData(); 
  
  //Serial.println("Calibrating gyro...");
  for (int i=1;i<=points;i++){
    getIMUData();
    zeroX += GYR_pitch;
    zeroY += GYR_roll;
    zeroZ += GYR_yaw;
    // turn on/off status light every 100 points
    if (i % 100 == 0){
      digitalWrite(12, (stateLED) ? HIGH : LOW);
      stateLED = !stateLED;
    }
  }

  zeroX /= points;
  zeroY /= points;
  zeroZ /= points;

  digitalWrite(12,LOW);  
}

//===== Fuction for initializing MPU6050 comms=====
void initIMU(){  
  //sets communtiotion towards IMU and sends wake up call;
  Wire.begin();
  Wire.beginTransmission(IMU_ADDR);
  Wire.write(WAKE_REG);
  Wire.write(0x00);
  Wire.endTransmission();

  //set gyro configuration
  Wire.beginTransmission(IMU_ADDR); //GYRO_CONFIG address
  Wire.write(0x1B);
  Wire.write(0x08); //FS_SEL=1 -> range 65.5 deg/s
  Wire.endTransmission();

  //set acc configuration
  Wire.beginTransmission(IMU_ADDR); //GYRO_CONFIG address
  Wire.write(0x1C);
  Wire.write(0x16); //AFS_SEL=2 -> range 4096 1/g
  Wire.endTransmission();  
  
}

//
void getTXInputs(){
  IBus.loop();
  RECIEVER [0] = IBus.readChannel(0); //channel 1
  RECIEVER [1] = IBus.readChannel(1); //channel 2
  RECIEVER [2] = IBus.readChannel(2); //channel 3
  RECIEVER [3] = IBus.readChannel(3); //channel 4
  RECIEVER [4] = IBus.readChannel(4); //channel 5
  RECIEVER [5] = IBus.readChannel(5); //channel 6
  RECIEVER [6] = IBus.readChannel(6); //channel 7
  RECIEVER [7] = IBus.readChannel(7); //channel 8
  RECIEVER [8] = IBus.readChannel(8); //channel 9
  RECIEVER [9] = IBus.readChannel(9); //channel 10
}


//===== Interrupt sub routine. Reads the reciever signals=====
// currently set for four inputs.
//ISR(PCINT0_vect){
//  timerPins = micros();
//
//  // CHANNEL 0
//  if (PINB & B00000001){
//    pin8  = 1;
//    timerPin8 = timerPins; //signal 1
//  }  
//  else if (pin8 == 1 && !(PINB & B00000001)){ 
//    pin8 = 0;
//    RECIEVER[0] = timerPins - timerPin8; //signal 0
//  }
//
//  // CHANNEL 1
//  if (PINB & B00000010){
//    pin9  = 1;
//    timerPin9 = timerPins; //signal 1
//  }  
//  else if (pin9 == 1 && !(PINB & B00000010)){ 
//    pin9 = 0;
//    RECIEVER[1] = timerPins - timerPin9; //signal 0
//  }
//
//  // CHANNEL 3
//  if (PINB & B00000100){
//    pin10 = 1;
//    timerPin10 = timerPins; //signal 1
//  }  
//  else if (pin10 == 1 && !(PINB & B00000100)){ 
//    pin10 = 0;
//    RECIEVER[2] = timerPins - timerPin10; //signal 0
//  }
//
//  // CHANNEL 4
//  if (PINB & B00001000){
//    pin11 = 1;
//    timerPin11 = timerPins; //signal 1
//  }  
//  else if (pin11 == 1 && !(PINB & B00001000)){ 
//    pin11 = 0;
//    RECIEVER[3] = timerPins - timerPin11; //signal 0
//  }
//}

void blinkStatusLED(int interval, int blinks){
  //LED blink routine. Do not run in void main! Only in void setup.  
  int i;
  for (i=1;i<=blinks;i++){
    digitalWrite(12,HIGH);
    delay(interval);
    digitalWrite(12,LOW);
    delay(interval);
  }  
}





