#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <ctl.h>
#include <UCA1_uart.h>
#include <terminal.h>
#include "pins.h"

#define ARRAY_SIZE(a) (sizeof(a)/sizeof(a[0]))


//make printf send over UCA1
int __putchar(int ch){
  return UCA1_TxChar(ch);
}

int __getchar(void){
  return UCA1_Getc();
}

//make clocks available for measuring by outputting them to pins
int clkCmd(char *argv[],unsigned short argc){
    unsigned short bgnd=0,stop=0;
    if(argc!=0){
        //check for background clock test
        if(!strcmp("bgnd",argv[1])){
            //notify user
            printf("Starting Background clock test\r\n");
            bgnd=1;
        //check for end of background clock test
        }else if(!strcmp("stop",argv[1])){
            //notify user
            printf("Stopping Background clock Test\r\n");
            stop=1;
        }
    }
    if(!stop){
        //output clocks on clock pins
        CLOCK_SEL0=CLOCK_PINS;
        CLOCK_DIR=CLOCK_PINS;
    }
    //if not stopping or background wait for input
    //from user so that measurements can be taken
    if(!stop && !bgnd){
        //flush keys
        //TODO: is this needed?
        //debugRxFlush();
        //notify user
        printf("Preforming Clock Test\r\nPress any key to terminate.");
        //wait for key press
        //UCA1_Getc();                    //this way sends CPU to low power mode causing incorrect MCLK readings
        while(UCA1_CheckKey()==EOF);      //this way blokcs lower priority tasks from running. not a big deal cause there is only one useful task
        //clock test done notify user
        printf("\r\nClock Test terminated.\r\n");
    }
    //if not background then clear select bits
    if(!bgnd){
        CLOCK_SEL0&=~CLOCK_PINS;
        CLOCK_SEL1&=~CLOCK_PINS;
        CLOCK_DIR&=~CLOCK_PINS;
    }
    //if in background mode clocks still being outputted now
    //otherwise clock pins are now inputs
}


//define max and min port numbers for usage in port test
#define MIN_PORT_NUM    1
#define MAX_PORT_NUM    8

int patternCmd(char *argv[],unsigned short argc){
    unsigned short tmp,num=0;
    int i,j;
    //port offsets from P1
    unsigned char P_idx[MAX_PORT_NUM];
    //port output registers
    volatile unsigned char *(p_out[MAX_PORT_NUM]),*addr;
    unsigned char m;
    int state;
    //check for at least one argument
    if(argc==0){
        //user must supply at least one port
        printf("error: at least one port name must be supplied.\r\n");
        return -1;
    }
    //check for maximum number of arguments
    if(argc>MAX_PORT_NUM){
      printf("Error : too many ports. maximum number of ports is %i\r\n",MAX_PORT_NUM);
      return -2;
    }
    //read in arguments, parse and check
    for(i=1;i<=argc;i++){
        //must be in the form of Px where x is 1-8 and P is case insensitive
        if(tolower(argv[i][0])!='p'){
            //did not get P
            printf("Error: %s is not a valid port name. Port names must be supplied in the form of Px.\r\n",MIN_PORT_NUM,MAX_PORT_NUM,argv[i]);
            return -3;
        }
        //parse port number
        tmp=argv[i][1]-'0';
        //check port number and string length
        if(tmp<MIN_PORT_NUM || tmp>MAX_PORT_NUM || argv[i][2]!='\0'){
            //report error
            printf("Error: %s is not a valid port name. Port names range from %i to %i.\r\n",argv[i]);
            return -4;
        }
        //store port index
        P_idx[i-1]=(tmp-1);
        num++;
    }
    //clear output bits and set pins to output
    for(i=0;i<argc;i++){
        //a giant switch is necessary here because
        //port addresses are not uniformly distributed
        switch(P_idx[i]){
            case 0:
                //P1
                p_out[i]=&P1OUT;
                P1OUT=0;
                P1DIR=0xFF;
                P1SEL0=0;
                P1SEL1=0;
                P1REN=0;
            break;
            case 1:
                //P2
                p_out[i]=&P2OUT;
                P2OUT=0;
                P2DIR=0xFF;
                P2SEL0=0;
                P2SEL1=0;
                P2REN=0;
            break;
            case 2:
                //P3
                p_out[i]=&P3OUT;
                P3OUT=0;
                P3DIR=0xFF;
                P3REN=0;
                //clear select pins except for P3.6 and P3.7
                //these are required for UART usage
                P3SEL0&=UART_PINS;
                //inform the user that P3.6 and P3.7 will not work
                printf("Warning: ");
                for(i=0;i<8;i++){
                  if(UART_PINS&(1<<i)){
                    printf("P%i.%i",UART_PORT,(1<<i));
                  }
                }
                printf(" not used in pattern test.\r\n");
            break;
            case 3:
                //P4
                p_out[i]=&P4OUT;
                P4OUT=0;
                P4DIR=0xFF;
                P4SEL0=0;
                P4REN=0;
            break;
            case 4:
                //P5
                p_out[i]=&P5OUT;
                P5OUT=0;
                P5DIR=0xFF;
                P5SEL0=0;
                P5SEL1=0;
                P5REN=0;
            break;
            case 5:
                //P6
                p_out[i]=&P6OUT;
                P6OUT=0;
                P6DIR=0xFF;
                P6SEL0=0;
                P6SEL1=0;
                P6REN=0;
            break;
            case 6:
                //P7
                p_out[i]=&P7OUT;
                P7OUT=0;
                P7DIR=0xFF;
                P7SEL0=0;
                P7REN=0;
            break;
            case 7:
                //P8
                p_out[i]=&P8OUT;
                P8OUT=0;
                P8DIR=0xFF;
                P8SEL0=0;
                P8REN=0;
            break;
            default :
                printf("Unknown Error\r\n");
        }
    }
    printf("Running pattern test.\r\nPress any key to terminate.");
    //disable interrupts so pulses are of consistent length
    state=ctl_global_interrupts_set(0);
    for(;;){
        //pulse each port
        for(i=0;i<num;i++){
          //get port address
          addr=p_out[i];
          //pulse each bit on port
          for(j=0,m=1;j<8;j++,m<<=1){
            //set high
            *(addr)|=m;
            //set low
            *(addr)&=~m;
          }
        }
        //enable interrupts so that keys are received
        ctl_global_interrupts_set(1);
        //check for keypress
        if(UCA1_CheckKey()!=EOF){
          //exit loop
          break;
        }
        //disable interrupts again
        ctl_global_interrupts_set(0);
    }
    //restore interrupts to there previous state
    ctl_global_interrupts_set(state);
    //print ending message
    printf("\r\nPattern test terminated.\r\n");
    //clear output bits and set pins to input
    for(i=0;i<argc;i++){
        //clear output bits
        *(p_out[i])=0;
        //set direction to input
        switch(P_idx[i]){
            case 0:
                P1DIR=0;
            break;
            case 1:
                P2DIR=0;
            break;
            case 2:
                P3DIR=0;
            break;
            case 3:
                P4DIR=0;
            break;
            case 4:
                P5DIR=0;
            break;
            case 5:
                P6DIR=0;
            break;
            case 6:
                P7DIR=0;
            break;
            case 7:
                P8DIR=0;
            break;
        }
    }
    return 0;
}

//flash the LED's on P7
int LED_Cmd(char *argv[],unsigned short argc){
  unsigned char mask=0,dir=0;
  //initialize LED pins
  P7OUT=BIT0;
  P7DIR=0xFF;
  P7REN=0;
  P7SEL0=0;
  //setup LED variables
  mask=BIT0;
  dir=1;
  printf("Preforming LED Test\r\nPress any key to terminate.");
  //wait for key press
  while(UCA1_CheckKey()==EOF){
    //blink at 0.1Hz
    ctl_timeout_wait(ctl_get_current_time()+102);
    //shift light
    if(dir){
      mask<<=1;
    }else{
      mask>>=1;
    }
    //check for ends
    if(mask==0){
      if(dir){
        mask=BIT6;
      }else{
        mask=BIT1;
      }
      //invert direction
      dir=!dir;
    }
    //apply mask
    P7OUT=mask;
  }
  printf("\r\nLED Test terminated.\r\n");
  P7DIR=0;
}

//drive waveforms on P1 0-7 and P3 0-2 and 4-5
int busCmd(char *argv[],unsigned short argc){
    //mask array, contains values to write to port
    const unsigned char m[]={(1<<0),(1<<1),(1<<2),(1<<3),(1<<4),(1<<5),(1<<6),(1<<7) ,BUS_PIN_SDA,BUS_PIN_SCL,BUS_PIN_SCK,BUS_PIN_SOMI,BUS_PIN_SIMO};
    //port array contains address of port register to write to
    volatile unsigned char *(port[])={&P2OUT,&P2OUT,&P2OUT,&P2OUT,&P2OUT,&P2OUT,&P2OUT,&P2OUT, &P3OUT,&P3OUT,&P3OUT,&P3OUT,&P3OUT};
    //last port pointer used for cleanup
    volatile unsigned char *pLast=&P1OUT;
    int i=0;
    //check to see that both arrays are equally sized
    //this should be optimized out by most compilers
    if(ARRAY_SIZE(m)!=ARRAY_SIZE(port)){
        printf("Internal Error\r\n");
        return 1;
    }
    //inform the user that the test is starting
    printf("Preforming Bus Test\r\nPress any key to terminate.\r\n");
    //setup P1 for output
    P2OUT=0;
    P2REN=0;
    P2SEL0=0;
    P1DIR=0xFF;
    //setup P3
    P3OUT=0;
    P3REN=0;
    P3SEL0&=~(BUS_PINS_SER);
    P3DIR|= BUS_PINS_SER;
    //wait for key press
    while(UCA1_CheckKey()==EOF){
        *port[i]=m[i];
        //if last port different from current port
        //clear pin values
        if(pLast!=port[i]){
            *pLast=0;
        }
        //remember last port
        pLast=port[i];
        i++;
        //wrap arround
        if(i>=ARRAY_SIZE(port)){
            i=0;
        }
    }
    //notify user of completion
    printf("Bus Test complete.\r\n");
    //set ports to input
    P1DIR=0;
    P3DIR=0;
    return 0;
}

//period/2 for I2C waveform
unsigned short I2C_T=0;

//TODO: This test does not test 

//switch I2C pins between input and low states
void i2c_toggle(void) __interrupt[TIMER1_A0_VECTOR] {
    TA1CCR0+=I2C_T;
    P3DIR^=(BUS_PINS_I2C);
    P1OUT^=BIT0;
}

//repeatedly pull-down the I2C pins
int I2C_Cmd(char *argv[],unsigned short argc){
    const char *(freqNames[])={"10kHz","200kHz"};
    const unsigned short freqVals[]={1600/2,80/2};
    int i;
    //check that both frequency arrays are equally sized
    //this should be optimized out at compile time
    if(ARRAY_SIZE(freqVals)!=ARRAY_SIZE(freqNames)){
        printf("Internal Error.\r\n");
        return 1;
    }
    //use P1.0 for a trigger
    P1SEL0&=~BIT0;
    P1OUT&=~BIT0;
    P1DIR|=BIT0;
    //setup I2C pins
    P3OUT&=~(BUS_PINS_I2C);
    P3DIR&=~(BUS_PINS_I2C);
    P3REN&=~(BUS_PINS_I2C);
    //start SDA high so that waveforms are out of phase
    P3DIR|=BUS_PIN_SDA;
    //setup TBCCR0 for I2C testing
    TA1CTL=TASSEL_2|TACLR;
    I2C_T=freqVals[0];
    TA1CCTL0=CCIE;
    TA1CCR0=40;
    TA1CTL|=MC_2;
    //Do all tests in array
    for(i=0;i<ARRAY_SIZE(freqNames);i++){
        printf("Preforming %s I2C Test\r\nPress any key for next test.\r\n",freqNames[i]);
        //set frequency
        I2C_T=freqVals[i];
        //wait for key press
        UCA1_Getc(); 
    }
    //test complete, notify user and exit
    printf("I2C Test complete.\r\n");
    //stop timer
    TA1CCTL0=0;
    TA1CTL=0;
    //pins back to inputs
    P3DIR&=~(BIT4|BIT5);
    P2DIR&=~BIT0;
    return 0;
}


//reset a MSP430 on command
int restCmd(char **argv,unsigned short argc){
  //force user to pass no arguments to prevent unwanted resets
  if(argc!=0){
    printf("Error : %s takes no arguments\r\n",argv[0]);
    return -1;
  }
  //print reset message
  puts("Initiating reset\r\n");
  //wait for UART buffer to empty
  while(UCA1_CheckBusy());
  //write to WDTCTL without password causes PUC
  WDTCTL=0;
  //Never reached due to reset
  puts("Error : Reset Failed!\r");
  return 0;
}

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                    {"reset","\r\n\t""Reset the MSP430",restCmd},
                    {"clk","[bgnd|stop]\r\n\t""Output the clock signals on P5 pins",clkCmd},
                    {"pattern","port [port ...]\r\n\t""Output test pattern to given ports",patternCmd},
                    {"LED","\r\n\t""Blink LEDS in sequence",LED_Cmd},
                    {"bus","\r\n\t""Output pattern on BUS pins",busCmd},
                    {"I2C","\r\n\t""Toggle I2C pins",I2C_Cmd},
                   //end of list
                   {NULL,NULL,NULL}};
