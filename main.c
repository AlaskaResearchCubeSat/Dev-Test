#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include <ctl.h>
#include <UCA1_uart.h>
#include <terminal.h>
#include "timer.h"
#include "pins.h"
#include "vcore.h"

//task structure for idle task
CTL_TASK_t idle_task;

CTL_TASK_t terminal_task;

//stack for task
unsigned stack1[1+256+1];


void initCLK(void){
  //set XT1 load caps, do this first so XT1 starts up sooner
  UCSCTL6=XCAP_0|XT2OFF|XT1DRIVE_3;
  //stop watchdog
  WDTCTL = WDTPW|WDTHOLD;

  //change core voltage and DCO frequency
  PMM_setVCore(PMM_CORE_LEVEL_3);

  //setup clocks
  //set frequency range
  UCSCTL1=DCORSEL_5;
  //setup FLL for 19.99 MHz operation
  UCSCTL2=FLLD__4|(609);
  UCSCTL3=SELREF__XT1CLK|FLLREFDIV__4;

  //use XT1 for ACLK and DCO for MCLK and SMCLK
  UCSCTL4=SELA_0|SELS_3|SELM_3;
}

//prototype for clkCmd so it can be called from start_term
int clkCmd(char *argv[],unsigned short argc);

void start_term(void *p){
  //run clock test
  clkCmd(NULL,0);
  //start terminal task when done
  terminal(p);
}

void main(void){
  //turn on LED's this will flash the LED's during startup
  P7OUT=0xFF;
  P7DIR=0xFF;
  //initialize clocks
  initCLK();
  //disable AUX supplies
  AUXCTL0_H=AUXKEY_H;
  AUXCTL1=AUX2MD|AUX1MD|AUX0MD|AUX0OK;
  //disable backup supply
  //BACKCTL=BAKDIS;
  //setup timerA
  init_timerA();
  //initialize UART
  UCA1_init_UART(UART_PORT,UART_TX_PIN_NUM,UART_RX_PIN_NUM);

 //initialize tasking
  ctl_task_init(&idle_task, 255, "idle");  

  //start timerA
  start_timerA();

  //initialize stack
  memset(stack1,0xcd,sizeof(stack1));  // write known values into the stack
  stack1[0]=stack1[sizeof(stack1)/sizeof(stack1[0])-1]=0xfeed; // put marker values at the words before/after the stack

  //turn off LED's
  P7OUT=0;

  //create tasks
  ctl_task_run(&terminal_task,2,start_term,"ARC Development Test Program Ready","terminal",sizeof(stack1)/sizeof(stack1[0])-2,stack1+1,0);
  
  // drop to lowest priority to start created tasks running.
  ctl_task_set_priority(&idle_task,0);

  for(;;){
    LPM0;
  }
}



//==============[task library error function]==============

//something went seriously wrong
//perhaps should try to recover/log error
void ctl_handle_error(CTL_ERROR_CODE_t e){
  switch(e){
    case CTL_ERROR_NO_TASKS_TO_RUN: 
      __no_operation();
      //puts("Error: No Tasks to Run\r");
    break;
    case CTL_UNSUPPORTED_CALL_FROM_ISR: 
      __no_operation();
      //puts("Error: Wait called from ISR\r");
    break;
    case CTL_UNSPECIFIED_ERROR:
      __no_operation();
      //puts("Error: Unspesified Error\r");
    break;
    default:
      __no_operation();
      //printf("Error: Unknown error code %i\r\n",e);
  }
  //something went wrong, reset
  WDTCTL=0;
}
