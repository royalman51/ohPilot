#include <Wire.h>
#include <GY80.h>
#include "rgb_lcd.h"

GY80 IMU = GY80(); //create GY80 instance
rgb_lcd lcd;

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
float zeroACCx = 6.4;
float zeroACCy = 0.3;

//float zeroACCz = 0.0;

float PITCH_ACC_OUT, ROLL_ACC_OUT;


void setup()
{
    // initialize serial communication at 115200 bits per second:
    Serial.begin(115200);

    //-----for debugging with groove LCD-----
    lcd.begin(16, 2);
    lcd.setRGB(255,255,255);
    
    lcd.clear();
    lcd.setCursor(0,0);        //Set the LCD cursor to position to position 0,0
    lcd.print("Pitch:");       //Print text to screen
    lcd.setCursor(0,1);        //Set the LCD cursor to position to position 0,1
    lcd.print("Roll :");       //Print text to screen 


    //-----setup for IMU-----
    IMU.begin();    //initialize sensors
    calibrateIMU(); //calibrating IMU gyro
        

    //-----start of loop timer-----
    timerMain = micros();  // start of loop timer 
}


void loop()
{
    getIMUAngles();

    //PITCH_ACC_OUT = PITCH_ACC_OUT * 0.90 + 0.1 * PITCH_ACC; 
    //ROLL_ACC_OUT  = ROLL_ACC_OUT  * 0.90 + 0.1 * ROLL_ACC; 

    Serial.print("GYR_pitch = ");
    Serial.print(GYR_pitch);
    Serial.print(", GYR_roll = ");    
    Serial.print(GYR_roll);
    Serial.print(", GYR_yaw = "); 
    Serial.println(GYR_yaw);

    //dampen output for LCD
    angle_pitch_output = angle_pitch_output * 0.9 + PITCH * 0.1; 
    angle_roll_output  = angle_roll_output  * 0.9 + ROLL  * 0.1;

    write_LCD();

    while(micros()-timerMain < 4000); // end of loop timer, wait for periodESC to pass
    timerMain = micros();  
    delay(250);        // delay in between reads for stability

    
}

void getIMUAngles(){
  // get raw data
  getIMUData();
  
  //correcting for zero offset of gyro. Offset is determined during calibration  
  //GYR_pitch -= zeroX; //pitch
  //GYR_roll  -= zeroY; //roll
  //GYR_yaw   -= zeroZ; //yaw

  //gyro angles (integration)
  PITCH += GYR_pitch* 0.004; 
  ROLL  += GYR_roll * 0.004; //(1/250) 

  //gyro yaw angular rate
  YAW   -= GYR_yaw  * 0.004;

  //couple gyro axes
  PITCH += ROLL  * sin(GYR_yaw * 0.0000698131); //(1/250) * (3.14159/180)
  ROLL  -= PITCH * sin(GYR_yaw * 0.0000698131);

  //Angles from accelerometer
  Z_ACC     = sqrt((ACC_X*ACC_X)+(ACC_Y*ACC_Y)+(ACC_Z*ACC_Z));
  PITCH_ACC = asin((float)ACC_Y/Z_ACC)*  57.296; //(180/3.14159)
  ROLL_ACC  = asin((float)ACC_X/Z_ACC)* -57.296;

  //zero offset correction for accelerometer
  PITCH_ACC -= zeroACCy; 
  ROLL_ACC  -= zeroACCx;
  
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
  
  GY80_scaled val = IMU.read_scaled();  
  
  ACC_X = val.a_x;
  ACC_Y = val.a_y;
  ACC_Z = val.a_z;
  TEMP  = val.t;
  GYR_pitch = val.g_x;
  GYR_roll  = val.g_y;  
  GYR_yaw   = val.g_z;

  //ACC_X += zeroACCx;
  //ACC_Y += zeroACCy;
  //ACC_Z += zeroACCz;
  
}


//===== Fuction for calibrating the IMU at the start (void setup)=====
void calibrateIMU(){
  
  int points = 2000;
  int stateLED = 0;
  digitalWrite(13,HIGH);
  getIMUData(); 
  
  Serial.println("Calibrating gyro...");
  for (int i=1;i<=points;i++){
    getIMUData();
    zeroX += GYR_pitch;
    zeroY += GYR_roll;
    zeroZ += GYR_yaw;
    // turn on/off status light every 100 points
    if (i % 100 == 0){
      digitalWrite(13, (stateLED) ? HIGH : LOW);
      stateLED = !stateLED;
    }
  }

  zeroX /= points;
  zeroY /= points;
  zeroZ /= points;

  //
  Serial.print("ZeroX = ");
  Serial.print(zeroX);
  Serial.print(", ZeroY = ");
  Serial.print(zeroY);
  Serial.print(", ZeroZ = ");
  Serial.println(zeroZ);
  

  calIMU = 1;

  digitalWrite(13,LOW);  
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
