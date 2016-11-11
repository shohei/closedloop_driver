#define STEP 11 
#define DIR 12 

void setup(){
  Serial.begin(9600);
  delay(1000);
  Serial.println("test stepper initialize");
  pinMode(STEP,OUTPUT);
  pinMode(DIR,OUTPUT);

  digitalWrite(DIR,HIGH);
}


void testRun(){
	for (int i=0;i<16*200;i++){
		writePulse();
		delayMicroseconds(500);
    // delay(10);
	}
}

void writePulse(){
		digitalWrite(STEP,HIGH);
		delayMicroseconds(5);
		digitalWrite(STEP,LOW);
		delayMicroseconds(5);
}

void loop(){
  Serial.println("testRun");
  testRun();
  delay(3000);
}

