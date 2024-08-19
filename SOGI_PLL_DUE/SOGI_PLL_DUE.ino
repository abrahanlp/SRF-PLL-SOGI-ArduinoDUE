/*
* More info on:
* https://faultyproject.es/category/control-medida/rms-y-frecuencia-con-arduino
*/
#include "sogi_pll.h"

volatile bool timerFlag = false;
volatile unsigned int timerCount;

float k_0 = 0.6;
float wn_0 = 2*M_PI*50;
float kp_0 = 10.0;
float ki_0 = 200.0;
float ts_0 = 0.0002;
volatile unsigned int u_in_0 = 0.0;
float phase = 0.0;

float ADC_Vref = 3.3;
float ADC_factor = ADC_Vref/4096;
float ADC_offset = 2.35;

volatile unsigned long i0 = 0;
volatile unsigned long time_elapsed = 0;
unsigned long max_time_elapsed = 0;

void setup() {
  analogReadResolution(12);
  analogWriteResolution(12);
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Arduino DUE AC Monitor");
  sogi_pll_start(k_0, wn_0, kp_0, ki_0, ts_0);
  config_timer2();
  timerCount = 0;
}

void loop() {
  if (timerFlag) {
    analogWrite(DAC0, u_in_0);
    phase = sogi_pll_update((u_in_0 * ADC_factor) - ADC_offset);
    analogWrite(DAC1, map((int)(phase*1000), 0, (int)(M_TWOPI*1000), 0, 4096));
    time_elapsed = micros() - i0;
    
    if(time_elapsed > max_time_elapsed){
      max_time_elapsed = time_elapsed;
    }

    if(timerCount >= 2500){
      Serial.print(max_time_elapsed);
      Serial.print(',');
      Serial.print(sogi_pll_get_ampli());
      Serial.print(',');
      Serial.println(sogi_pll_get_freq());
      timerCount = 0;
    }
    time_elapsed = micros() - i0;

    timerFlag = false;
  }
  
}

void TC5_Handler() {
  i0 = micros();
  u_in_0 = analogRead(A0);
  timerFlag = true;
  timerCount++;
  TC_GetStatus(TC1, 2); //Clear flag interruption
}

void config_timer2(void){
  pmc_set_writeprotect(false);
  pmc_enable_periph_clk((uint32_t) TC5_IRQn);
  TC_Configure(TC1, 2, TC_CMR_TCCLKS_TIMER_CLOCK1 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC);
  TC_SetRC(TC1, 2, 8400); // 1":84000000/2; 1ms:84000/2; 1us: 84/2;
  TC_Start(TC1, 2);
  TC1->TC_CHANNEL[2].TC_IER=TC_IER_CPCS;
  TC1->TC_CHANNEL[2].TC_IDR=~TC_IER_CPCS;
  NVIC_EnableIRQ(TC5_IRQn);
}