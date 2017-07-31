
#include <Wire.h>
#include <LiquidCrystal.h>
#include <math.h>

//main loop parameters
long timerMain; 
int periodESC = 4000; //[us] loop timer

// for lcd (copied from brokking)
int lcd_loop_counter;
int angle_pitch_buffer, angle_roll_buffer;
float angle_pitch_output,angle_roll_output;

//parameters for IMU
float GYR_pitch, GYR_roll, GYR_yaw, ACC_X, ACC_Y, ACC_Z, TEMP; //raw values from reading IMU modules
float PITCH, ROLL, YAW, PITCH_GYR, ROLL_GYR, YAW_GYR, PITCH_ACC, ROLL_ACC, Z_ACC; //IMU determined angles
float zeroX, zeroY, zeroZ;
int calIMU=0; //status indicator if imu is calibrated

//accelerometer
float zeroACCx = 0.1;
float zeroACCy = -3.0;
float zeroACCz = 0.0;

//scaling of raw IMU data towards usefull information
float gyrScale = 65.5, accScale = 4096;

#define IMU_ADDR 0x68
#define WAKE_REG 0x6B
#define ACC_REG  0x3B

LiquidCrystal lcd(1,2,4,5,6,7);

void setup() {  
  //-----setup for arduino ports------
  pinMode(12,OUTPUT);
  lcd.begin(16,2);
  Wire.begin();
  
  //-----setup for IMU-----
  initIMU();      //seting up IMU i2c communication and settings
  
  //LCD settings, coppied from brokking
  lcd.clear();
  lcd.setCursor(0,0);                                                  //Set the LCD cursor to position to position 0,0
  lcd.print("Pitch:");                                                 //Print text to screen
  lcd.setCursor(0,1);                                                  //Set the LCD cursor to position to position 0,1
  lcd.print("Roll :");                                                 //Print text to screen
  
  

  //-----setup for IMU-----
  calibrateIMU(); //calibrating IMU gyro


  digitalWrite(12,LOW);
  //-----start of loop timer-----
  timerMain = micros();  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  getIMUAngles();

  
  //dampen output for LCD
  angle_pitch_output = angle_pitch_output * 0.9 + PITCH * 0.1; 
  angle_roll_output  = angle_roll_output  * 0.9 + ROLL  * 0.1;
  
  write_LCD();
  
  while(micros() - timerMain < periodESC);                 
  timerMain = micros(); 
}

void getIMUAngles(){
//===== Fuction for calculating IMU angles from raw data=====
  // get raw data
  getIMUData();
  
  //correcting for zero offset of gyro. Offset is determined during calibration  
  GYR_pitch -= zeroX; //pitch
  GYR_roll  -= zeroY; //roll
  GYR_yaw   -= zeroZ; //yaw

  //gyro angles (integration)
  PITCH += GYR_pitch* 0.000061069; 
  ROLL  += GYR_roll * 0.000061069; //(1/250)/65.5  
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


void getIMUData(){
//===== Fuction for reading MPU6050 raw data=====
  
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

void calibrateIMU(){
//===== Fuction for calibrating the IMU at the start (void setup)=====
  
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



void initIMU(){
//===== Fuction for initializing MPU6050 comms=====
  
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





void write_LCD(){                                                      //Subroutine for writing the LCD
  //To get a 250Hz program loop (4us) it's only possible to write one character per loop
  //Writing multiple characters is taking to much time
  if(lcd_loop_counter == 14)lcd_loop_counter = 0;                      //Reset the counter after 14 characters
  lcd_loop_counter ++;                                                 //Increase the counter
  if(lcd_loop_counter == 1){
    angle_pitch_buffer = angle_pitch_output * 10;                      //Buffer the pitch angle because it will change
    lcd.setCursor(6,0);                                                //Set the LCD cursor to position to position 0,0
  }
  if(lcd_loop_counter == 2){
    if(angle_pitch_buffer < 0)lcd.print("-");                          //Print - if value is negative
    else lcd.print("+");                                               //Print + if value is negative
  }
  if(lcd_loop_counter == 3)lcd.print(abs(angle_pitch_buffer)/1000);    //Print first number
  if(lcd_loop_counter == 4)lcd.print((abs(angle_pitch_buffer)/100)%10);//Print second number
  if(lcd_loop_counter == 5)lcd.print((abs(angle_pitch_buffer)/10)%10); //Print third number
  if(lcd_loop_counter == 6)lcd.print(".");                             //Print decimal point
  if(lcd_loop_counter == 7)lcd.print(abs(angle_pitch_buffer)%10);      //Print decimal number

  if(lcd_loop_counter == 8){
    angle_roll_buffer = angle_roll_output * 10;
    lcd.setCursor(6,1);
  }
  if(lcd_loop_counter == 9){
    if(angle_roll_buffer < 0)lcd.print("-");                           //Print - if value is negative
    else lcd.print("+");                                               //Print + if value is negative
  }
  if(lcd_loop_counter == 10)lcd.print(abs(angle_roll_buffer)/1000);    //Print first number
  if(lcd_loop_counter == 11)lcd.print((abs(angle_roll_buffer)/100)%10);//Print second number
  if(lcd_loop_counter == 12)lcd.print((abs(angle_roll_buffer)/10)%10); //Print third number
  if(lcd_loop_counter == 13)lcd.print(".");                            //Print decimal point
  if(lcd_loop_counter == 14)lcd.print(abs(angle_roll_buffer)%10);      //Print decimal number
}

