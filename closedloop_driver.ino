#define INA1_PIN 2
#define INA2_PIN 3
#define INB1_PIN 4
#define INB2_PIN 5
#define AP_PIN 8
#define AN_PIN 9
#define BP_PIN 10 
#define BN_PIN 11 

#define STAT_A (1<<3|0<<2|0<<1|1<<0)
#define STAT_B (1<<3|0<<2|1<<1|0<<0)
#define STAT_C (0<<3|1<<2|1<<1|0<<0)
#define STAT_D (0<<3|1<<2|0<<1|1<<0)

int pin_ovf_led = 13;
uint8_t ap,an,bp,bn;
uint8_t current;
uint8_t previous;
long counter;

void setup(){
	counter=0;
	pinMode(pin_ovf_led, OUTPUT);
	digitalWrite(pin_ovf_led, LOW);
	setupTimer();

	pinMode(INA1_PIN,INPUT);
	pinMode(INA2_PIN,INPUT);
	pinMode(INB1_PIN,INPUT);
	pinMode(INB2_PIN,INPUT);
	pinMode(AP_PIN,INPUT);
	pinMode(AN_PIN,INPUT);
	pinMode(BP_PIN,INPUT);
	pinMode(BN_PIN,INPUT);


	SerialUSB.begin(115200);
	while (!SerialUSB) {};  
	SerialUSB.println("init done.");
	updateEncoder();
	previous = ap<<3|an<<2|bp<<1|bn<<0; 	
}

void loop(){
	delay(1000);
	printCounter();
}

void printCounter(){
	SerialUSB.print("counter: ");
	SerialUSB.println(counter);
}

void updateEncoder(){
	ap = digitalRead(AP_PIN);
	an = digitalRead(AN_PIN);
	bp = digitalRead(BP_PIN);
	bn = digitalRead(BN_PIN);
}

void decodeEncoder(){
	updateEncoder();

	current = ap<<3|an<<2|bp<<1|bn<<0; 	

	if(current == STAT_A){
    // 1001
    if(previous == STAT_D){
    	counter++;
    	} else if(previous == STAT_B){
    		counter--;
    	}
    	} else if(current == STAT_B){
    // 1010
    if(previous == STAT_A){
    	counter++;
    	} else if(current == STAT_C){
    		counter--;
    	}
    	} else if(current == STAT_C) {
    // 0110
    if(previous == STAT_B){
    	counter++;
    	} else if(previous == STAT_D){
    		counter--;
    	}
    	} else if(current == STAT_D) {
    // 0101
    if(previous == STAT_C){
    	counter++;
    	} else if(previous == STAT_A){
    		counter--;
    	}
    }
    previous = current;
}


void setupTimer(){
	REG_GCLK_CLKCTRL = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_TCC0_TCC1)) ;
	while ( GCLK->STATUS.bit.SYNCBUSY == 1 );

	Tcc* TC = (Tcc*) TCC0;

	TC->CTRLA.reg &= ~TCC_CTRLA_ENABLE;
	while (TC->SYNCBUSY.bit.ENABLE == 1);

  // TC->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV1024;  // Set perscaler
  TC->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV256;  // Set perscaler

  TC->WAVE.reg |= TCC_WAVE_WAVEGEN_NFRQ;
  while (TC->SYNCBUSY.bit.WAVE == 1);

  //In case pre-scaler is 1024
  // TC->PER.reg = 0xB71B;  // Set counter Top using the PER register : 1s
  // TC->PER.reg = 0x124F;  // Set counter Top using the PER register : 10ms
  // TC->PER.reg = 0x1D4;  // Set counter Top using the PER register : 1ms

  //In case pre-scaler is 256
  // TC->PER.reg = 0x1D4;  // Set counter Top using the PER register : 250us
  // TC->PER.reg = 0x2E;  // Set counter Top using the PER register : 25us
  TC->PER.reg = 0x4;  // Set counter Top using the PER register : 2.5us

  while (TC->SYNCBUSY.bit.PER == 1)

  TC->CC[0].reg = 0xFFF;
  while (TC->SYNCBUSY.bit.CC0 == 1);

  TC->INTENSET.reg = 0;
  TC->INTENSET.bit.OVF = 1;
  TC->INTENSET.bit.MC0 = 1;

  NVIC_EnableIRQ(TCC0_IRQn);

  TC->CTRLA.reg |= TCC_CTRLA_ENABLE ;
  while (TC->SYNCBUSY.bit.ENABLE == 1);
}


void TCC0_Handler()
{
	Tcc* TC = (Tcc*) TCC0;

	if (TC->INTFLAG.bit.OVF == 1) {
    	// periodical processing
    	decodeEncoder();
    	TC->INTFLAG.bit.OVF = 1;
    }

    if (TC->INTFLAG.bit.MC0 == 1) {
	TC->INTFLAG.bit.MC0 = 1;
    }
}

