float Vbat;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  // U_bat = U_in*((R1+R2)/R1), -0.2 for safety
  Vbat = ((analogRead(A0) * (5.0/1023.0)) * ((994.0+1495.0)/994.0)) - 0.2;
  if (Vbat < 0.0) Vbat = 0.0;

  Serial.println(Vbat);
  delay(100);
}
