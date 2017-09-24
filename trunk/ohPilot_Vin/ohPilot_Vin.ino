float Vbat;
float readV=0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //port 12 to output (status LED)
  DDRB = DDRB | B00010000;  
}

void loop() {
  // put your main code here, to run repeatedly:
  // U_bat = U_in*((R1+R2)/R1), -0.2 for safety

  readV = readV *0.7 + 0.3 * analogRead(A0);    
  Vbat = 0.012244897959184 * readV -0.085102040816353;
  
  if (Vbat < 12.0){
    PORTB &= B00010000;
  }


  Serial.print("Analog read = ");
  Serial.print(readV);
  Serial.print(", V battery = ");
  Serial.println(Vbat);
  delay(100);
}
