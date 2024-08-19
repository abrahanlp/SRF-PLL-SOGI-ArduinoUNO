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
float ki_0 = 1000.0;
float ts_0 = 0.001;
volatile unsigned int u_in_0 = 0.0;
float phase = 0.0;

float ADC_Vref = 4.68;
float ADC_factor = ADC_Vref/1024;
float ADC_offset = 2.33;

volatile unsigned long i0 = 0;
volatile unsigned long time_elapsed = 0;
unsigned long max_time_elapsed = 0;

void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("Arduino UNO AC Monitor");
  sogi_pll_start(k_0, wn_0, kp_0, ki_0, ts_0);
  config_timer2();
  timerCount = 0;
}

void loop() {
  if (timerFlag) {
    phase = sogi_pll_update((u_in_0 * ADC_factor) - ADC_offset);
    time_elapsed = micros() - i0;
    
    if(time_elapsed > max_time_elapsed){
      max_time_elapsed = time_elapsed;
    }

    if(timerCount >= 500){
      Serial.print(max_time_elapsed);
      Serial.print(',');
      Serial.print(sogi_pll_get_ampli());
      Serial.print(',');
      Serial.println(sogi_pll_get_freq());
      timerCount = 0;
    }
    
    timerFlag = false;
  }
}

ISR(TIMER2_COMPA_vect){
  i0 = micros();
  u_in_0 = analogRead(A0);
  timerFlag = true;
  timerCount++;
}

void config_timer2(void){
  TCCR2A = 3;  // Configurar registro TCCR2A a 0 para modo normal
  TCCR2B = 3;  // Configurar registro TCCR2B a 0 para prescaler 250kH
  TCNT2  = 0;  // Establecer el contador en 0
  OCR2A = 250; // Establecer el valor de comparación para 1ms (F_CPU/1000/64 - 1)
  TIMSK2 |= (1 << OCIE2A); // Habilitar la interrupción de comparación de Timer2
}