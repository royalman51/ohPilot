  unsigned long t0, timerPin8, timerPin9, timerPin10, timerPin11, timerPins, RECIEVER[4], timerESC, timerMain;
  int tTest;
  int tprev = 0;
  int pin8 = 0,pin9 = 0,pin10 = 0,pin11 = 0;
  int periodESC = 4000;


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

void sendPulseESC(int p0, int p1, int p2, int p3){
  

  //int Period
  timerESC = micros()+p0;
  PORTD = PORTD | B00010000; //set pin 4 HIGH    
  while(micros()<timerESC);
  PORTD = PORTD & B11101111; //set pin 4 LOW

  
}

void loop() {
  // put your main code here, to run repeatedly:
  //delayMicroseconds(periodESC);
  //timerMain = micros();  // start of loop
    
  //sendPulseESC(RECIEVER[2],1000,1000,1000);

  
  //delayMicroseconds(period);
  



  Serial.print("Roll == ");
  Serial.print(RECIEVER[0]);
  Serial.print(", Pitch == ");
  Serial.print(RECIEVER[1]);
  Serial.print(", Throttle == ");
  Serial.print(RECIEVER[2]);
  Serial.print(", Yaw == ");
  Serial.println(RECIEVER[3]);
  delay(100);

  //end of loop
  //while(micros()-timerMain < periodESC);
}

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

