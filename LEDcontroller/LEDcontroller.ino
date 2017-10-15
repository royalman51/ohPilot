#include <Wire.h>

//battery voltage
float Vbat=13.0, readV=1023.0, Vbatavg=13.0;
int LEDsOnOff = 0;

#define LED_CONTR_ADDR 5

void setup() {
  // put your setup code here, to run once:
  // configure i2c comms
  Wire.begin(LED_CONTR_ADDR); // setup slave device for comms with FC
  Wire.onReceive(recieveBytes);
  
  //Serial.begin(115200);

  
  //initialize output pins
  pinMode(A2,INPUT); //input for battery voltage
  pinMode(7,OUTPUT); //LED RightForward
  pinMode(8,OUTPUT); //LED RightAft
  pinMode(9,OUTPUT); //LED LeftAft
  pinMode(10,OUTPUT); //LED LeftForward

  //turn off LEDs for start
  toggleLED(0);
 
}

void loop() {
  // put your main code here, to run repeatedly:


  if (LEDsOnOff == 1){
    getVbat();
    if (Vbatavg < 10.0){
      toggleLED(1);
      delay(200);
      toggleLED(0);
      delay(200);
    }
  }
  
  
  
}



void toggleLED(int stat){
  if (stat == 1){
    digitalWrite(7,HIGH);
    digitalWrite(8,HIGH);
    digitalWrite(9,HIGH);
    digitalWrite(10,HIGH);
  }
  if (stat == 0){
    digitalWrite(7,LOW);
    digitalWrite(8,LOW);
    digitalWrite(9,LOW);
    digitalWrite(10,LOW);
  }

}


void getVbat(){
  readV = readV *0.7 + 0.3 * analogRead(A2);  
  Vbat = 0.012244897959184 * readV -0.085102040816353;
  Vbatavg = Vbatavg*0.95+Vbat*0.05;
}


void recieveBytes(int nBytes){
  while(Wire.available())
  {
    char c = Wire.read();
    if (c == '1'){
      LEDsOnOff = 1; 
      digitalWrite(7,HIGH);
      digitalWrite(8,HIGH);
      digitalWrite(9,HIGH);
      digitalWrite(10,HIGH);
    }
    else if (c == '0'){
      LEDsOnOff = 0;
      digitalWrite(7,LOW);
      digitalWrite(8,LOW);
      digitalWrite(9,LOW);
      digitalWrite(10,LOW);
    }
  }
}

