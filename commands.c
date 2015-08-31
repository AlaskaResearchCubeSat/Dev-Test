#include <msp430.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <ctl.h>
#include <UCA1_uart.h>
#include <terminal.h>
#include <limits.h>
#include "pins.h"
#include "timer.h"

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
    unsigned short num=0;
    unsigned long tmp;
    int i,j,first;
    char *end;
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
            printf("Error: %s is not a valid port name. Port names must be supplied in the form of Px.\r\n",argv[i]);
            return -3;
        }
        //parse port number
        tmp=strtoul(argv[i]+1,&end,10);
        //check for errors
        if(argv[i]+1==end){
          printf("Error: can not parse port number \"%s\".\r\n",argv[i]+1);
          return -5;
        }
        //check for trailing chars
        if(*end){
          printf("Error: unknown suffix \"%s\" for port \"%s\".\r\n",end,argv[i]);
          return -6;
        }
        //check port number and string length
        if(tmp<MIN_PORT_NUM || tmp>MAX_PORT_NUM || argv[i][2]!='\0'){
            //report error
            printf("Error: %s is not a valid port name. Port names range from %i to %i.\r\n",argv[i],MIN_PORT_NUM,MAX_PORT_NUM);
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
                //clear select pins except for P3.5 and P3.6
                //these are required for UART usage
                P3SEL0&=UART_PINS;
                //inform the user that P3.5 and P3.6 will not work
                printf("Warning: ");
                for(first=1,i=0;i<8;i++){
                  if(UART_PINS&(1<<i)){
                    printf("%sP%i.%i",(first)?"":", ",UART_PORT,i);
                    first=0;
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
  //cause a software Brown Out Reset to occur
  PMMCTL0=PMMPW|PMMSWBOR;
  //Never reached due to reset
  puts("Error : Reset Failed!\r");
  return 0;
}

int infoCmd(char **argv,unsigned short argc){
  enum{ DDT_INFO_FLAG=0x0001,DDT_DIE_FLAG=0x0002,DDT_ADC10_FLAG=0x0004};
  int fl=0,i,len;
  unsigned char *ptr;
  for(i=0;i<argc;i++){
    if(!strcmp(argv[i+1],"Info")){
      fl|=DDT_INFO_FLAG;
    }else if(!strcmp(argv[i+1],"Die")){
      fl|=DDT_DIE_FLAG;
    }else if(!strcmp(argv[i+1],"ADC10")){
      fl|=DDT_ADC10_FLAG;
    }else{
      printf("Error : unknown argument \"%s\"\r\n",argv[i+1]);
      return 1;
    }
  }
  //check CRC for TLV section
  //initialize CRC module
  CRCINIRES=0xFFFF;
  //get CRC length
  for(ptr=(unsigned char*)TLV_START-4;ptr<=(unsigned char*)TLV_END;ptr++){
    //read in each byte
    CRCDIRB_L=*ptr;
  }
  //check CRC
  if(CRCINIRES!=*(unsigned short*)0x1A02){
    printf("Error : Invalid CRC\r\n");
    return 2;
  }
  //print info block
  if(fl&DDT_INFO_FLAG || !fl){
    //check info block length
    if(0x6!=*(unsigned char*)0x1A00){
      printf("Error : invalid info block length\r\n");
      return 3;
    }
    printf("Info Block :\r\n");
    printf("\t""Device ID    : 0x%04X\r\n",*(unsigned short*)0x1A04);
    printf("\t""Hardware Rev : %u\r\n",*(unsigned char*)0x1A06);
    printf("\t""Firmware Rev : %u\r\n",*(unsigned char*)0x1A07);
  }
  //print die reccord
  if(fl&DDT_DIE_FLAG){
    if(TLV_DIERECORD!=*(unsigned char*)0x1A08){
      printf("Error : invalid Die Record tag\r\n");
      return 4;
    }
    if(0x0A!=*(unsigned char*)0x1A09){
      printf("Error : invalid Die Record length\r\n");
      return 5;
    }
    printf("Die Record :\r\n");
    printf("\t""Lot ID         : %lu\r\n",*(unsigned long*)0x1A0A);
    printf("\t""X pos          : %u\r\n",*(unsigned short*)0x1A0E);
    printf("\t""X pos          : %u\r\n",*(unsigned short*)0x1A10);
    printf("\t""Test Record CP : %u\r\n",*(unsigned char*)0x1A12);
    printf("\t""Test Record FT : %u\r\n",*(unsigned char*)0x1A13);
  }
  //print ADC10 calibration data
  if(fl&DDT_ADC10_FLAG){
    if(TLV_ADC10CAL!=*(unsigned char*)0x1A14){
      printf("Error : invalid ADC10 Calibration tag\r\n");
      return 6;
    }
    if(0x10!=*(unsigned char*)0x1A15){
      printf("Error : invalid ADC10 Calibration length\r\n");
      return 7;
    }
    printf("ADC10 Calibration :\r\n");
    printf("\t""ADC Gain Factor : %u\r\n",*(unsigned short*)0x1A16);
    printf("\t""ADC Offset      : %u\r\n",*(unsigned short*)0x1A18);
    printf("\t""ADC 15T30       : %u\r\n",*(unsigned short*)0x1A1A);
    printf("\t""ADC 15T85       : %u\r\n",*(unsigned short*)0x1A1C);
    printf("\t""ADC 20T30       : %u\r\n",*(unsigned short*)0x1A1E);
    printf("\t""ADC 20T85       : %u\r\n",*(unsigned short*)0x1A20);
    printf("\t""ADC 25T30       : %u\r\n",*(unsigned short*)0x1A22);
    printf("\t""ADC 25T85       : %u\r\n",*(unsigned short*)0x1A24);
  }
  return 0;
}

//ADC events
CTL_EVENT_SET_t SD24_events;

//events for SD24
enum {SD24_OVERFLOW_EVT=SD24OVIFG0};

long SD24_results[7];

#define SD24_read(ch)     {unsigned long val;val =(unsigned long)SD24BMEML##ch;val|=((unsigned long)SD24BMEMH##ch)<<16;SD24_results[ch]=val;}

void SD24_ISR(void) __ctl_interrupt[SD24B_VECTOR]{
  switch(SD24BIV){
    case SD24BIV_SD24OVIFG:
      ctl_events_set_clear(&SD24_events,SD24_OVERFLOW_EVT,0);
      //clear interrupts
      SD24BIFG&=~(SD24OVIFG0|SD24OVIFG1|SD24OVIFG2|SD24OVIFG3|SD24OVIFG4|SD24OVIFG5|SD24OVIFG6);
    return;
    case SD24BIV_SD24TRGIFG:
    break;
    case SD24BIV_SD24IFG0:
      ctl_events_set_clear(&SD24_events,SD24IFG0,0);
      //read and store value
      SD24_read(0);
    return;
    case SD24BIV_SD24IFG1:
      ctl_events_set_clear(&SD24_events,SD24IFG1,0);
      //read and store value
      SD24_read(1);
    return;
    case SD24BIV_SD24IFG2:
      ctl_events_set_clear(&SD24_events,SD24IFG2,0);
      //read and store value
      SD24_read(2);
    return;
    case SD24BIV_SD24IFG3:
      ctl_events_set_clear(&SD24_events,SD24IFG3,0);
      //read and store value
      SD24_read(3);
    return;
    case SD24BIV_SD24IFG4:
      ctl_events_set_clear(&SD24_events,SD24IFG4,0);
      //read and store value
      SD24_read(4);
    return;
    case SD24BIV_SD24IFG5:
      ctl_events_set_clear(&SD24_events,SD24IFG5,0);
      //read and store value
      SD24_read(5);
    return;
    case SD24BIV_SD24IFG6:
      ctl_events_set_clear(&SD24_events,SD24IFG6,0);
      //read and store value
      SD24_read(6);
    return;
    //unknown interrupt
    default:
    break;
  }
}

int analogCmd(char **argv,unsigned short argc){
  unsigned short e;
  int i,j,error=0;
  long val;
  #define TEST_OSR      0xFF
  #define TEST_SCALE    (2.5/(float)(((long)1)<<(3*8)))
  struct{
    long min,max;
    unsigned char mask;
  }test_dat[4]={{-30000,30000,0x2A},{16500000,LONG_MAX,0x15},{-30000,30000,0x2A},{LONG_MIN,-16500000,0x15}};
  printf("Use shorting jumpers to short P5 to analog pins and press any key to continue\r\n");
  UCA1_Getc();
  //setup P5 pins for test
  P5OUT &=~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5);
  P5SEL0&=~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5);
  P5SEL1&=~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5);
  P5DIR |= (BIT0|BIT1|BIT2|BIT3|BIT4|BIT5);
  //setup reference
  //check if reference is busy
  if(!(REFCTL0&REFGENBUSY)){
    REFCTL0=REFMSTR|REFVSEL_3|REFON;
  }else{
    printf("REF busy! continuing anyway\r\n");
  }
  //set ADC settings
  SD24BCTL0=SD24PDIV_1|SD24DIV1|SD24DIV2|SD24SSEL__SMCLK|SD24REFS|SD24OV32;
  SD24BCTL1=0;
  //setup ADCs to test
  SD24BCCTL0 =SD24SNGL|SD24DF_1|SD24SCS__GROUP0;
  SD24BINCTL0=0;
  SD24BOSR0  =TEST_OSR;
  SD24BPRE0  =0x3FF;          //maximum preload to allow inputs to change
  
  SD24BCCTL1 =SD24SNGL|SD24DF_1|SD24SCS__GROUP0;
  SD24BINCTL1=0;
  SD24BOSR1  =TEST_OSR;
  SD24BPRE1  =0x3FF;          //maximum preload to allow inputs to change
  
  SD24BCCTL2 =SD24SNGL|SD24DF_1|SD24SCS__GROUP0;
  SD24BINCTL2=0;
  SD24BOSR2  =TEST_OSR;
  SD24BPRE2  =0x3FF;          //maximum preload to allow inputs to change

  //setup event
  ctl_events_init(&SD24_events,0);
  //clear interrupt flags
  SD24BIFG=0;
  //enable interrupts 
  SD24BIE=SD24OVIE0|SD24IE0|SD24OVIE1|SD24IE1|SD24OVIE2|SD24IE2;

  for(i=0;i<4;i++){
    //trigger conversion
    SD24BCTL1|=SD24GRP0SC;

    //wait for conversion to complete
    e=ctl_events_wait(CTL_EVENT_WAIT_ALL_EVENTS_WITH_AUTO_CLEAR,&SD24_events,SD24IFG0|SD24IFG1|SD24IFG2,CTL_TIMEOUT_DELAY,1024*2);
    //clear trigger bit
    SD24BCTL1&=~SD24GRP0SC;
    //check for overflow
    if(e&SD24_OVERFLOW_EVT){
      printf("Error : overflow detected\r\n");
      error=0xFF;
      break;
    }
    //check if conversion completed
    if(e!=(SD24IFG0|SD24IFG1|SD24IFG2)){
      printf("Error : Timeout\r\n");
      error=0x80;
      break;
    }
    //print values
    for(j=0;j<3;j++){
      val=SD24_results[j];
      //check results
      if(val<test_dat[i].min || val>test_dat[i].max){      
        printf("\r\nError : AN%i is out of range\r\n\tmeasured value = %f V\r\nMin = %f V\r\nMax = %f V\r\n",j,val*TEST_SCALE,test_dat[i].min*TEST_SCALE,test_dat[i].max*TEST_SCALE);
        error|=1<<j;
      }
    }
    //setup new output
    P5OUT^=test_dat[i].mask;
  }
  if(error){
    if(error!=0xFF && error!=0x80){
      printf("\r\nThe following channels showed errors:\r\n");
      for(i=0;i<3;i++){
        if(error&(1<<i)){
          printf("\tAN%i\r\n",i);
        }
      }
    }
  }else{
    printf("All tests passed!!!\r\n");
  }
  //disable interrupts 
  SD24BIE=0;
  //pins back to inputs
  P5DIR &=~(BIT0|BIT1|BIT2|BIT3|BIT4|BIT5);

  return 0;
}

#define SD24BCCTL_IDX     0
#define SD24BINCTL_IDX    1
#define SD24BOSR_IDX      2
#define SD24BPRE_IDX      3

int SD24_Cmd(char **argv,unsigned short argc){
  unsigned short e;
  int chan=0;
  volatile unsigned int *ptr=&SD24BCCTL0;
  //check for arguments
  if(argc==1){
    //get value
    chan=atoi(argv[1]);
    //check if value is out of range
    if(chan<0 || chan>6){
      printf("Error : value out of range\r\n");
      return 1;
    }
    //set register pointer
    ptr=(&SD24BCCTL0)+4*chan;
  }else if(argc!=0){
    printf("Error : too many arguments\r\n");
  }
  //setup reference
  //check if reference is busy
  if(!(REFCTL0&REFGENBUSY)){
    REFCTL0=REFMSTR|REFVSEL_3|REFON;
  }else{
    printf("REF busy! continuing anyway\r\n");
  }
  //set ADC settings
  SD24BCTL0=SD24PDIV_4|SD24DIV1|SD24DIV2|SD24SSEL__SMCLK|SD24REFS|SD24OV32;
  SD24BCTL1=0;
  //setup ADCs to test
  ptr[SD24BCCTL_IDX] =SD24DF_1|SD24SCS__SD24SC;  
  ptr[SD24BINCTL_IDX]=0;
  ptr[SD24BOSR_IDX]  =TEST_OSR;
  ptr[SD24BPRE_IDX]  =0;    

  //setup event
  ctl_events_init(&SD24_events,0);
  //clear interrupt flags
  SD24BIFG=0;
  //enable interrupts 
  SD24BIE=(SD24OVIE0|SD24IE0)<<chan;
  //start conversion
  ptr[SD24BCCTL_IDX]|=SD24SC;
  //loop until a key is pressed
  while(EOF==UCA1_CheckKey()){
    //wait for conversion to complete
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&SD24_events,SD24IFG0<<chan,CTL_TIMEOUT_DELAY,512);
    if(e==0){
      printf("\r\nTimeout!\r\n");
      break;
    }
    //print result and erase line
    printf("\r%f       ",SD24_results[chan]*TEST_SCALE);
  }
  //stop conversion
  ptr[SD24BCCTL_IDX]&=~SD24SC;
  //disable interrupts 
  SD24BIE=0;
  //print message
  printf("\r\nComplete\r\n");
}

//================[TA0 Interrupt]=========================
void TA0_int(void) __ctl_interrupt[TIMER0_A1_VECTOR]{
  switch(TA0IV){
    //set next ADC trigger
    case TA0IV_TA0CCR1:
      TA0CCR1+=16384;
      //start next conversion
      ADC10CTL0|=ADC10SC;
    break;
  }
}

unsigned short auxv_meas[4];
int aux_idx;
CTL_EVENT_SET_t aux_ev;

//================[ADC10 Interrupt]=========================
void ADC_int(void) __ctl_interrupt[ADC10_VECTOR]{
  switch(ADC10IV){
    case ADC10IV_ADC10IFG:
      //get sample
      auxv_meas[aux_idx++]=ADC10MEM0;
      //check for wraparound
      if(aux_idx>=4){
        aux_idx=0;
        //set event
        ctl_events_set_clear(&aux_ev,BIT0,0);
      } 
      //select next voltage
      AUXADCCTL&=~AUXADCSEL_3;
      AUXADCCTL|=aux_idx<<1;
      if(aux_idx!=0){      
        //start next conversion
        ADC10CTL0|=ADC10SC;
      }
    break;
  }
}

int AUX_Cmd(char **argv,unsigned short argc){
  unsigned short e;
  int i,found;
  unsigned short j;
  char *end;
  unsigned char aux_sw;
  unsigned short aux_th,aux_stat;
  const char *(aux_names[4])={"DVCC","AUXVCC1","AUXVCC2","AUXVCC3"};
  const char *(aux_th_names[8])={"1.74","1.94","2.14","2.26","2.40","2.70","3.00","3.00"};
  const char *(pmm_SVM_names[8])={"1.7","1.9","2.1","2.2","2.35","2.65","3.0","3.0"};
  const float aux_th_vals[8]={1.74,1.94,2.14,2.26,2.40,2.70,3.00,3.00};
  const int aux_chg_res[4]={-1,5,10,20};
  //if arguments given then setup AUX
  if(argc>0){
    //check for presets
    if(!strcmp(argv[1],"tst")){
      //unlock AUX
      AUXCTL0_H=AUXKEY_H;
      //all supplies under hardware control
      AUXCTL1=0;
      //setup thresholds 
      AUXCTL2=AUXMR_0|AUX2LVL_5|AUX1LVL_5|AUX0LVL_7;
      //disable chargers
      AUX2CHCTL=AUXCHKEY;
      AUX3CHCTL=AUXCHKEY;
      //lock AUX
      AUXCTL0_H=0;
      //done return
      return 0;
    }else if(!strcmp(argv[1],"ctst")){
      //enable chargers
      AUX2CHCTL=AUXCHKEY|AUXCHV_1|AUXCHC_1|AUXCHEN;
      AUX3CHCTL=AUXCHKEY|AUXCHV_1|AUXCHC_1|AUXCHEN;
      return 0;
    }
    //first argument supply
    for(i=0,found=0;i<4;i++){
      if(!strcmp(argv[1],aux_names[i])){
        found=1;
        break;
      }
    }
    //check if name matched
    if(!found){
      printf("Error : Unknown supply \"%s\"\r\n",argv[1]);
      return 1;
    }
    //check if a command is given
    if(argc>1){
      //read command
      if(!strcmp(argv[2],"CHG")){
        unsigned long res;
        unsigned short chg_val;
        //AUXVCC1 and DVCC don't support charging
        if(i<2){
          printf("Error : %s does not support charging\r\n",argv[1]);
          return 3;
        }
        //check if charging resistor was given
        if(argc<3){
          printf("Error : charge resistance not given\r\n");
          return 4;
        }
        //check if too many arguments given
        if(argc>3){
          printf("Error : too many arguments\r\n");
          return 5;
        }
        if(!strcmp(argv[3],"OFF")){
          //turn charger off
          chg_val=AUXCHKEY;
        }else{
          //parse charge resistor
          res=strtoul(argv[3],&end,10);
          //check if there was an error
          if(argv[3]==end){
            printf("Error : failed to parse charge resistor \"%s\"\r\n",argv[3]);
            return 6;
          }
          //check for suffix
          if(*end!='\0'){
            if(!strcmp(end,"k")){
              res*=1000;
            }else{
              printf("Error : unknown suffix \"%s\" for charge resistor\r\n",end);
              return 7;
            }
          }
          //compute value for register
          res=(res+2500)/5000;
          //maximum value is 4
          res=res>3?3:res;
          //compose value
          chg_val=AUXCHKEY|AUXCHV_1|(res<<1)|AUXCHEN;
        }
        //write to charging register
        if(i==2){
          AUX2CHCTL=chg_val;
        }else{
          AUX3CHCTL=chg_val;
        }
      }else if(!strcmp(argv[2],"ON")){
        //check for AUXVCC3
        if(i==3){
          printf("Error : AUXVCC3 does not support switching\r\n");
          return 8;
        }
        //unlock AUX
        AUXCTL0_H=AUXKEY_H;
        //set AXUxMD and AUXxOK
        AUXCTL1|=(AUX0OK|AUX0MD)<<i;
        //lock AUX
        AUXCTL0_H=0;
      }else if(!strcmp(argv[2],"OFF")){
        //check for AUXVCC3
        if(i==3){
          printf("Error : AUXVCC3 does not support switching\r\n");
          return 8;
        }
        //unlock AUX
        AUXCTL0_H=AUXKEY_H;
        //set AXUxMD
        AUXCTL1|=(AUX0OK)<<i;
        //clear AUXxOK
        AUXCTL1&=~(AUX0OK<<i);
        //lock AUX
        AUXCTL0_H=0;
      }else if(!strcmp(argv[2],"HW")){
        //check for AUXVCC3
        if(i==3){
          printf("Error : AUXVCC3 does not support switching\r\n");
          return 8;
        }
        //unlock AUX
        AUXCTL0_H=AUXKEY_H;
        //clear AUXxMD
        AUXCTL1&=~(AUX0MD<<i);
        //lock AUX
        AUXCTL0_H=0;
      }else if(!strcmp(argv[2],"TH")){
        unsigned short tmp;
        float th;
        //check for AUXVCC3
        if(i==3){
          printf("Error : AUXVCC3 does not support switching threshold\r\n");
          return 8;
        }
        //check if switching threshold was given
        if(argc<3){
          printf("Error : switching threshold not given\r\n");
          return 4;
        }
        //check if too many arguments given
        if(argc>3){
          printf("Error : too many arguments\r\n");
          return 5;
        }
        //parse switching threshold
        th=strtof(argv[3],&end);
        //check if there was an error
        if(argv[3]==end){
          printf("Error : failed to parse switching threshold \"%s\"\r\n",argv[3]);
          return 6;
        }
        //check for suffix
        if(*end!='\0'){
          if(!strcmp(end,"mV")){
            th*=1000;
          }else if(!strcmp(end,"V")){
            //Nothing to do
          }else{
            printf("Error : unknown suffix \"%s\" for switching threshold\r\n",end);
            return 7;
          }
        }
        //pick a threshold
        for(j=0,found=0;j<8;j++){
          //find the largest threshold that is not greater than desired value
          if(th>=aux_th_vals[j] && (j==7 || th<aux_th_vals[j+1])){
            //value found
            found=1;
            //exit loop
            break;
          }
        }
        //check if threshold found
        if(!found){
          //threshold not found, return error
          printf("Error : could not find threshold close to %.2f V\r\n",th);
          return 10;
        }
        //read register
        tmp=AUXCTL2;
        //clear old level bits
        tmp&=~(AUX0LVL_7<<(4*i));
        //set new level bits
        tmp|= j<<(4*i);
        //unlock AUX
        AUXCTL0_H=AUXKEY_H;
        //set new value
        AUXCTL2=tmp;
        //lock AUX
        AUXCTL0_H=0;
      }else if(!strcmp(argv[2],"PRI")){
        if(i==0){
          printf("DVCC always has priority\r\n");
          return 0;
        }
        if(i==3){
          printf("AUXVCC3 is not selectable and has no priority\r\n");
          return 0;
        }
        //unlock AUX
        AUXCTL0_H=AUXKEY_H;
        if(i==1){
          //AUXVCC1 has priority
          AUXCTL1&=~AUX2PRIO;
        }else{
          //AUXVCC2 has priority
          AUXCTL1|=AUX2PRIO;
        }
        //lock AUX
        AUXCTL0_H=0;

      }else{
        printf("Error : unknown command \"%s\"\r\n",argv[2]);
        return 2;
      }
    }
    
    //print info about supply
    //read status
    aux_sw=AUXCTL0_L;
    aux_th=AUXCTL2;
    aux_stat=AUXCTL1;
    //check if AUXVCC1 or AUXVCC2
    if(i==1 ||i==2){
      //print supply priority
      printf("%s has priority\r\n",(aux_stat&AUX2PRIO)?"AUXVCC2":"AUXVCC1");
    }
    //print information about selectable supplies
    if(i<3){
      printf("%s switch %s\r\n",aux_names[i],(aux_sw&(BIT1<<i))?"closed":"open");
      j=(aux_th>>(4*i))&0x07;
      printf("threshold = %s V (%i)\r\n""Status = %s\r\n""Control = %s\r\n",aux_th_names[j],j,(aux_stat&(BIT0<<i))?"OK":"Not OK",(aux_stat&(BIT8<<i))?"Software":"Hardware");
    }
    //check for DVCC
    if(i==0){
      unsigned short pmm_th=SVSMHCTL&SVSMHRRL_7;
      //print SVM threshold
      printf("SVSMHRRL = %s V (%i)\r\n",pmm_SVM_names[pmm_th],pmm_th);
    }
    //check if there is a charger
    if(i>1){
      unsigned char aux_chg=(i==2)?AUX2CHCTL_L:AUX3CHCTL_L,v,r;
      v=(aux_chg>>4)&0x03;
      r=(aux_chg>>1)&0x03;
      //check if charger is enabled
      if(r==0 || v==0 || !(aux_chg&AUXCHEN)){
        printf("Charger Disabled\r\n");
      }else{
        printf("Charger:\r\n\t""End voltage = %s\r\n\t""Charge Resistor = %ik ohm\r\n",(v==0x01)?"VCC":"Invalid",aux_chg_res[r]);
      }
    }
    return 0;
  }
  //setup voltage reference
  REFCTL0=REFMSTR|REFVSEL_0|REFON;
  //Disable conversion so that we can setup the ADC
  ADC10CTL0&=~ADC10ENC;
  ADC10CTL0=ADC10SHT_10|ADC10MSC|ADC10ON;
  //ADC10CTL1=ADC10SHS_1|ADC10SHP|ADC10DIV_0|ADC10SSEL_3|ADC10CONSEQ_2;
  ADC10CTL1=ADC10SHS_0|ADC10SHP|ADC10DIV_0|ADC10SSEL_3|ADC10CONSEQ_0;
  ADC10CTL2=ADC10PDIV__4|ADC10RES;
  //setup conversion memory, read 
  ADC10MCTL0=ADC10SREF_1|ADC10INCH_12;
  //clear ADC interrupt flags
  ADC10IFG=0;
  //enable ADC interrupts
  ADC10IE=ADC10IE0;
   // set port 1 digital buffer off
   P1SEL0|=BIT0;
   P1SEL1|=BIT0;
  //enable conversion
  ADC10CTL0|=ADC10ENC;
  //setup AUX monitor
  AUXADCCTL=AUXADCR_0|AUXADCSEL_0|AUXADC;
  //setup events
  ctl_events_init(&aux_ev,0);
  //clear index
  aux_idx=0;
  //initialize aux memory
  auxv_meas[0]=auxv_meas[1]=auxv_meas[2]=auxv_meas[3]=0xFFFF;
  //setup timer
  TA0CCR1=readTA()+16*3;
  TA0CCTL1=CCIE|OUTMOD_5;
  //set ADC10SC
  ADC10CTL0|=ADC10SC;
  //print blank lines
  for(i=0;i<4;i++){
    printf(" %7s = --.-- V\r\n",aux_names[i]);
  }
  //Read aux supplies with ADC
  while(UCA1_CheckKey()==EOF){
    e=ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS_WITH_AUTO_CLEAR,&aux_ev,BIT0,CTL_TIMEOUT_DELAY,1024);
    if(e&BIT0){
      //get aux supplies
      aux_sw=AUXCTL0_L;
      aux_th=AUXCTL2;
      aux_stat=AUXCTL1;
      //move the cursor up
      printf("\x1B[4A\r");
      for(i=0;i<4;i++){
        //check if printing AUXVCC3
        if(i<3){
          //print wit threshold and indicate if selected
          printf("\x1B[2K""%c%7s = %.3f V\t%s V\t%c\t%cW\r\n",(aux_sw&(BIT1<<i))?'>':' ',aux_names[i],auxv_meas[i]*(1.5*3)/(1023.0),
                                                             aux_th_names[(aux_th>>(4*i))&0x07],(aux_stat&(BIT0<<i))?'O':'X',(aux_stat&(BIT8<<i))?'S':'H');
        }else{
          //AUXVCC3 is never used for supply and has no thresholds
          printf("\x1B[2K"" %7s = %.3f V\r\n",aux_names[i],auxv_meas[i]*(1.5*3)/(1023.0));
          
        }
      }
    }
  }
  //stop timer interrupts
  TA0CCTL1&=~CCIE;
  //disable ADC
  ADC10CTL0&=ADC10ENC;
  //turn off ADC
  ADC10CTL0&=ADC10ENC;
  //disable ref
  REFCTL0&=~REFON;
  //disable AUX sampling
  AUXADCCTL=0;
  //user exited
  return 0;
}


int back_Cmd(char **argv,unsigned short argc){
  unsigned short *ptr;
  char *end;
  unsigned short vals[4];
  int i;
  if(argc==0){
    //print backup memory
    for(i=0,ptr=(unsigned short*)&BAKMEM0;i<4;i++){
      printf("BAKMEM%i = 0x%04X\r\n",i,*ptr++);
    }
    return 0;
  }else if(argc==4){
    //parse arguments and write to backup memory
    for(i=0,ptr=(unsigned short*)&BAKMEM0;i<4;i++){
      //parse value
      vals[i]=strtoul(argv[i+1],&end,0);
      //check for errors
      if(argv[i+1]==end){
        printf("Error: can not parse \"%s\".\r\n",argv[i+1]);
        return 2;
      }
      //check for trailing chars
      if(*end){
        printf("Error: unknown suffix \"%s\" for \"%s\".\r\n",end,argv[i+1]);
        return 3;
      }
    }
    printf("Storing Values to backup memory:\r\n");
    //store values
    for(i=0,ptr=(unsigned short*)&BAKMEM0;i<4;i++){
      *ptr++=vals[i];
      printf("BAKMEM%i = 0x%04X\r\n",i,vals[i]);
    }
    return 0;
  }
  //wrong number of arguments given
  printf("Error : %s requires 0 or 4 arguments but %i given\r\n",argv[0],argc);
  return 1;
}

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                    {"reset","\r\n\t""Reset the MSP430",restCmd},
                    {"clk","[bgnd|stop]\r\n\t""Output the clock signals on P5 pins",clkCmd},
                    {"pattern","port [port ...]\r\n\t""Output test pattern to given ports",patternCmd},
                    {"LED","\r\n\t""Blink LEDS in sequence",LED_Cmd},
                    {"bus","\r\n\t""Output pattern on BUS pins",busCmd},
                    {"I2C","\r\n\t""Toggle I2C pins",I2C_Cmd},
                    {"info","[Info|Die|ADC10]\r\n\t""Print Device Information",infoCmd},
                    {"analog","\r\n\t""Test Analog Pins",analogCmd},
                    {"SD24","[chan]\r\n\t""Read from SD24",SD24_Cmd},
                    {"aux","[tst|ctst|[DVCC|AUXVCC1|AUXVCC2|AUXVCC3 [CHG|ON|OFF|HW|TH|PRI [arg]]]]\r\n\t""Enable AUX power supplies",AUX_Cmd},
                    {"back","[m0 m1 m2 m3]\r\n\t""Read/Write to the backup memory",back_Cmd},
                   //end of list
                   {NULL,NULL,NULL}};
