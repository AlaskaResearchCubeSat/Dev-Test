#include <msp430.h>
#include <ctl.h>

//read TA while timer is running
short readTA(void){
  int v=TA0R;
  //read until two consecutive readings match
  while(v!=TA0R);
  return v;
}

//setup timer A to run off 32.768kHz xtal
void init_timerA(void){
  //setup timer A 
  TA0CTL=TASSEL_1|ID_0|TACLR;
  //init CCR0 for tick interrupt
  TA0CCR0=32;
  TA0CCTL0=CCIE;
}

//start timer A in continuous mode
void start_timerA(void){
//start timer A
  TA0CTL|=MC_2;
}

//================[Time Tick interrupt]=========================
void task_tick(void) __ctl_interrupt[TIMER0_A0_VECTOR]{
  //set rate to 1024Hz
  TA0CCR0+=32;
  //increment timer
  ctl_increment_tick_from_isr();
}

