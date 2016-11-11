#include "lookuptable.h"
#include <math.h>

//firmware output
#define STP_PIN 6
#define DIR_PIN 7
//output to full bridge driver
#define INA1_PIN 8
#define INA2_PIN 9
#define INB1_PIN 10
#define INB2_PIN 11 
//encoder input
#define AP_PIN A0 
#define AN_PIN A1 
#define BP_PIN A2
#define BN_PIN A3
//define state
#define STAT_A (1<<3|0<<2|0<<1|1<<0)
#define STAT_B (1<<3|0<<2|1<<1|0<<0)
#define STAT_C (0<<3|1<<2|1<<1|0<<0)
#define STAT_D (0<<3|1<<2|0<<1|1<<0)

int pin_ovf_led = 13;
uint8_t ap,an,bp,bn;
uint8_t current;
uint8_t previous;
long counter;
uint8_t dir;
uint8_t stepper_state;

void setup(){
	counter=0;
	pinMode(pin_ovf_led, OUTPUT);
	digitalWrite(pin_ovf_led, LOW);
	setupTimer();

	pinMode(INA1_PIN,OUTPUT);
	pinMode(INA2_PIN,OUTPUT);
	pinMode(INB1_PIN,OUTPUT);
	pinMode(INB2_PIN,OUTPUT);
  pinMode(STP_PIN,INPUT);
  pinMode(DIR_PIN,INPUT);
	pinMode(AP_PIN,INPUT);
	pinMode(AN_PIN,INPUT);
	pinMode(BP_PIN,INPUT);
	pinMode(BN_PIN,INPUT);

  attachInterrupt(STP_PIN, trigStep, RISING);

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

void trigStep(){
  updateState();
  if (sin_a[stepper_state]>0){
    analogWrite(INA1_PIN,sin_a[stepper_state]);
    analogWrite(INA2_PIN,LOW);
    if(sin_b[stepper_state]>0){
      analogWrite(INB1_PIN,sin_b[stepper_state]);
      analogWrite(INB2_PIN,LOW);
    }else if(sin_b[stepper_state]<0){
      analogWrite(INB1_PIN,LOW);
      analogWrite(INB2_PIN,fabs(sin_b[stepper_state]));
    }
  } else if(sin_a[stepper_state]<0){
    analogWrite(INA1_PIN,LOW);
    analogWrite(INA2_PIN,fabs(sin_a[stepper_state]));
    if(sin_b[stepper_state]>0){
      analogWrite(INB1_PIN,sin_b[stepper_state]);
      analogWrite(INB2_PIN,LOW);
    }else if(sin_b[stepper_state]<0){
      analogWrite(INB1_PIN,LOW);
      analogWrite(INB2_PIN,fabs(sin_b[stepper_state]));
    }
  }
}

void updateState(){
  dir = digitalRead(DIR_PIN);
  if(dir==HIGH){
    stepper_state++;
  } else if(dir==LOW){
    stepper_state--;
  }
  cycleState();
}

void cycleState(){
    if(stepper_state>15){
      stepper_state=0;
    }else if(stepper_state<0){
      stepper_state=15;
    }
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

