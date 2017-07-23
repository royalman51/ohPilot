 // This script is used to calibrate your quad copter ESC. 
 // It reads the reciever inputs and directly sends them to the ESCs.


// parameters which can be changed if you know what you are doing
int periodESC = 4000; //pulse period of ESC signal, thus also the main loop period


// parameters which should not be changed
unsigned long t0, timerPin4, timerPin5, timerPin6, timerPin7, timerPin8, timerPin9, timerPin10, timerPin11, timerPins, RECIEVER[4], ESCOUT[4], timerESC, timerMain;
int tTest;
int tprev = 0;
int pin8 = 0,pin9 = 0,pin10 = 0,pin11 = 0;
int pin4, pin5, pin6, pin7;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  //set ports 4,5,6,7 to outputs
  DDRD = DDRD | B11110000;
  //set ports 8,9,10,11 to inputs and 12 to output;  
  DDRB = DDRB | B00010000;  

  //configure interrupts
  PCICR  |= (1<<PCIE0); //sets Pin change interrupt control register to activate PCMSK0 register
  PCMSK0 |= (1<<PCINT0); //config pin interrupt on pin 8
  PCMSK0 |= (1<<PCINT1); //config pin interrupt on pin 9
  PCMSK0 |= (1<<PCINT2); //config pin interrupt on pin 10
  PCMSK0 |= (1<<PCINT3); //config pin interrupt on pin 11 
  
}

//void sendPulseESC(int p0, int p1, int p2, int p3){
  
//}

void loop() {
  // put your main code here, to run repeatedly:  
  timerMain = micros();  // start of loop timer 

  //sets minimum value to 1000ms
  if (RECIEVER[0] < 1000) RECIEVER[0] = 1000;
  if (RECIEVER[1] < 1000) RECIEVER[1] = 1000;
  if (RECIEVER[2] < 1000) RECIEVER[2] = 1000;
  if (RECIEVER[3] < 1000) RECIEVER[3] = 1000;

  //sets maximum value to 2000ms
  if (RECIEVER[0] > 2000) RECIEVER[0] = 2000;
  if (RECIEVER[1] > 2000) RECIEVER[1] = 2000;
  if (RECIEVER[2] > 2000) RECIEVER[2] = 2000;
  if (RECIEVER[3] > 2000) RECIEVER[3] = 2000;

  //sets output to ESCs
  ESCOUT[0] = RECIEVER[2];
  ESCOUT[1] = RECIEVER[2];
  ESCOUT[2] = RECIEVER[2];
  ESCOUT[3] = RECIEVER[2];
  
  while(micros()-timerMain < periodESC); // end of loop timer, wait for periodESC to pass
  timerMain = micros();  
  
  PORTD |= B11110000; //set pins to high (motor 1,2,3,4) HIGH   
  pin4  = 1;
  pin5  = 1;
  pin6  = 1;
  pin7  = 1;
  timerPin4 = ESCOUT[0] + timerMain;
  timerPin5 = ESCOUT[0] + timerMain;
  timerPin6 = ESCOUT[0] + timerMain;
  timerPin7 = ESCOUT[0] + timerMain;

  while (pin4 == 1 & pin5 == 1 & pin6 == 1 & pin7 == 1){
    timerESC = micros();
    if (timerPin4 < timerESC){      
      PORTD &= B11101111;      //set pin 4 (motor 1) LOW
      pin4  = 0;       
    }
    if (timerPin5 < timerESC){      
      PORTD &= B11011111;      //set pin 4 (motor 1) LOW
      pin5  = 0;       
    }
    if (timerPin6 < timerESC){      
      PORTD &= B10111111;      //set pin 4 (motor 1) LOW
      pin6  = 0;       
    }
    if (timerPin7 < timerESC){      
      PORTD &= B01111111;      //set pin 4 (motor 1) LOW
      pin7  = 0;       
    }

    
    
  }



  //timerESC = micros();
  //PORTD = PORTD | B00010000; //set pin 4 HIGH    
  //while(micros()-RECIEVER[2]<timerESC);
  //PORTD = PORTD & B11101111; //set pin 4 LOW
}


// Interrupt sub routine. Reads the reciever signals.
// currently set for four inputs.
ISR(PCINT0_vect){
  timerPins = micros();

  // CHANHEL 0
  if (PINB & B00000001){
    pin8  = 1;
    timerPin8 = timerPins; //signal 1
  }  
  else if (pin8 == 1 && !(PINB & B00000001)){ 
    pin8 = 0;
    RECIEVER[0] = timerPins - timerPin8; //signal 0
  }

  // CHANHEL 1
  if (PINB & B00000010){
    pin9  = 1;
    timerPin9 = timerPins; //signal 1
  }  
  else if (pin9 == 1 && !(PINB & B00000010)){ 
    pin9 = 0;
    RECIEVER[1] = timerPins - timerPin9; //signal 0
  }

  // CHANHEL 3
  if (PINB & B00000100){
    pin10 = 1;
    timerPin10 = timerPins; //signal 1
  }  
  else if (pin10 == 1 && !(PINB & B00000100)){ 
    pin10 = 0;
    RECIEVER[2] = timerPins - timerPin10; //signal 0
  }

  // CHANHEL 4
  if (PINB & B00001000){
    pin11 = 1;
    timerPin11 = timerPins; //signal 1
  }  
  else if (pin11 == 1 && !(PINB & B00001000)){ 
    pin11 = 0;
    RECIEVER[3] = timerPins - timerPin11; //signal 0
  }
}

