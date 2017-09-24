//battery voltage
float Vbat=13.0, readV=1023.0, Vbatavg=13.0;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(115200);
  //initialize output pins
  pinMode(A0,INPUT);
  pinMode(A1,OUTPUT); //LED RightForward
  pinMode(A2,OUTPUT); //LED RightAft
  pinMode(A3,OUTPUT); //LED LeftAft
  pinMode(A5,OUTPUT); //LED LeftForward
    
}

void loop() {
  // put your main code here, to run repeatedly:

  getVbat();
  if (Vbatavg < 10.0){
    toggleLED(1);
    delay(200);
    toggleLED(0);
    delay(200);
  }
  else{
    toggleLED(1);  
  }

  //Serial.println(Vbatavg);
  
  
}



void toggleLED(int stat){
  if (stat == 1){
    analogWrite(A1,255);
    analogWrite(A2,255);
    analogWrite(A3,255);
    analogWrite(A5,255);
  }
  if (stat == 0){
    analogWrite(A1,0);
    analogWrite(A2,0);
    analogWrite(A3,0);
    analogWrite(A5,0);
  }

}


void getVbat(){
  readV = readV *0.7 + 0.3 * analogRead(A0);  
  Vbat = 0.012244897959184 * readV -0.085102040816353;
  Vbatavg = Vbatavg*0.95+Vbat*0.05;
}
