int Pulse = 1000;
int Period = 4000;

void setup() {
  // put your setup code here, to run once:
  //set ports 4,5,6,7 to outputs
  DDRD |= DDRD | B11110000;
  //set ports 8,9,10,11 to inputs and 12 to output;  
  DDRB |= DDRB | B00010000;  
}

void loop() {
  // put your main code here, to run repeatedly:


  PORTD = PORTD | B00010000; //set pin 4 HIGH    
  delayMicroseconds(Pulse);
  PORTD = PORTD & B11101111; //set pin 4 HIGH
  delayMicroseconds(Period - Pulse);

  //digitalWrite(4,HIGH);
  //digitalWrite(4,LOW);
  //delay(4);

}
