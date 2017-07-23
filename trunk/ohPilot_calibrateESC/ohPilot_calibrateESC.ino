 // This script is used to calibrate your quad copter ESC. 
 // It reads the reciever inputs and directly sends them to the ESCs.


// parameters which can be changed if you know what you are doing
int periodESC = 4000; //pulse period of ESC signal, thus also the main loop period


// parameters which should not be changed
unsigned long t0, timerPin8, timerPin9, timerPin10, timerPin11, timerPins, RECIEVER[4], timerESC, timerMain;
int tTest;
int tprev = 0;
int pin8 = 0,pin9 = 0,pin10 = 0,pin11 = 0;
 


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

  if (RECIEVER[2] < 1000) RECIEVER[2] = 1000;
  
  while(micros()-timerMain < periodESC); // end of loop timer, wait for periodESC to pass



  
  timerESC = micros();
  PORTD = PORTD | B11110000; //set pin 4 HIGH    
  while(micros()-RECIEVER[2]<timerESC);
  PORTD = PORTD & B00001111; //set pin 4 LOW



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

