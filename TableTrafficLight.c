/******************************************************************************
This program is an implementation of Lab 10 of the course UTAustinX: UT.6.03x.

It makes use of the microcontroller LM4F120/TM4C123 to interface with a6
external and 1 internal LED along with 3 push-button switches. 
The code in this project represents a traffic controller for managing vehicualr
traffic at a multi-lane intersection.It implements a Moore Finite State Machine
(FSM) to capture different possible traffic situations.

Following details outline the pinout configuration of the LEDS with different
port interfaces of the board:
east/west red light connected to PB5
east/west yellow light connected to PB4
east/west green light connected to PB3
north/south facing red light connected to PB2
north/south facing yellow light connected to PB1
north/south facing green light connected to PB0
pedestrian detector connected to PE2 (1=pedestrian present)
north/south car detector connected to PE1 (1=car present)
east/west car detector connected to PE0 (1=car present)
"walk" light connected to PF3 (built-in green LED)
"don't walk" light connected to PF1 (built-in red LED)
******************************************************************************/
/***** 1. Pre-processor Directives Section *****/
#include "TExaS.h"
#include "tm4c123gh6pm.h"
#include "PLL.h"

/***** 2. Global Declarations Section *****/

/* FUNCTION PROTOTYPES: Each subroutine defined */
void DisableInterrupts(void); /* Disable interrupts */
void EnableInterrupts(void);  /* Enable interrupts */

/***** 3. Subroutines Section *****/
void SysTick_Init(void); /* Routine for initializing SysTick timer */
void SysTick_Wait(unsigned long delay); /* Routine for producing time of delay
                                           ticks */
void SysTick_Wait10ms(unsigned long delay);/* Routine for waiting 10ms */
void PortF_Init(void); /* Routine initializing PortF */
void PortE_Init(void); /* Routine initializing PortE */
void PortB_Init(void); /* Routine initializing PortB */

/* Define addresses for NVIC control, reload and current count registers */
#define NVIC_ST_CTRL_R      (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R    (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R   (*((volatile unsigned long *)0xE000E018))

/* Define addresses for data registers of PORTE, PORTB, PORTF */
#define SENSOR  GPIO_PORTE_DATA_R /* Used for input. Port E(APB) */
#define LIGHT   GPIO_PORTB_DATA_R /* Used for traffic light output. Port B(APB) */
#define WALK    GPIO_PORTF_DATA_R /* Used for walk light output. Port F(APB) */


/* Structure representing State of traffic controller */
struct State {
  unsigned long Out;
  unsigned long walkOut;
  unsigned long Time;
  unsigned long Next[8];
}; 


typedef const struct State STyp;

/* Define different FSM inputs */
#define goW      0
#define waitW    1
#define goS      2
#define waitS    3
#define goP      4
#define waitA    5
#define flashP1  6
#define flashP2  7
#define flashP3  8 


/* Define FSM for traffic controller */
STyp FSM[9]=
{
{0x0C, 0x02, 10, {goW,goW,waitW,waitW,waitW,waitW,waitW,waitW}},
{0x14, 0x02, 10, {waitA,waitA,waitA,waitA,waitA,waitA,waitA,goS}},
{0x21, 0x02, 10, {goS,waitS,goS,waitS,waitS,waitS,waitS,waitS}},
{0x22, 0x02, 10, {waitA,waitA,waitA,waitA,waitA,waitA,waitA,goP}},
{0x24, 0x08, 10, {goP,flashP1,flashP1,flashP1,goP,flashP1,flashP1,waitA}},
{0x24, 0x02, 10, {waitA,goW,goS,goW,goP,goP,goP,flashP1}},
{0x24, 0x00, 10, {flashP2,flashP2,flashP2,flashP2,flashP2,flashP2,flashP2,flashP2}},
{0x24, 0x02, 10, {flashP3,flashP3,flashP3,flashP3,flashP3,flashP3,flashP3,flashP3}},
{0x24, 0x00, 10, {waitA,waitA,waitA,waitA,waitA,waitA,waitA,goW}}
};
 
/* Index for current state */
unsigned long S;
unsigned long Input; 


/* main function. Hold all logic of traffic controller */
int main(void){ 
  volatile unsigned long delay;

  PLL_Init();       /* Activate PLL to get a clock rate of 80 MHz. This is down
											 from the original system clock of 400 MHz*/
  
  /* Call all initialization functions */
  SysTick_Init();
  PortF_Init();
  PortE_Init();
  PortB_Init();

  /* Activate grader and set system clock to 80 MHz */
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210,ScopeOff);
 
  
  EnableInterrupts();

  S = goW;

  /* Run the FSM in an infinite loop */
  while(1){
     LIGHT = FSM[S].Out;     // set lights
     WALK  = FSM[S].walkOut; // set walk lights
     SysTick_Wait10ms(FSM[S].Time);
     Input = SENSOR;         //read sensors
     S = FSM[S].Next[Input];  
  }
}

/* The SysTick_Init() function sets the SysTick controller as mentioned in the 
   data sheet */
void SysTick_Init(void){
  /* First off, disable SysTick for setup */
  NVIC_ST_CTRL_R = 0;

  /* Enable SysTick by feeding it the system's core clock */
  NVIC_ST_CTRL_R = 0x00000005;
}


/* This function takes as input an integer value. The function produces a delay
   equivalent to the time it takes to reduce this integer value to 0. A single
   unit's reduction in this value takes 12.5ns given we are running on a 80 MHz
   clock. So the delay in seconds produced by this function can be calculated
   as follows: 12.5e-09 * delay */
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}

/* This function produces time delays of multiples of 10ms. Its input is the
   multiplying factor */
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    /* A count value of 800000 produced a time delay of 10ms */
    SysTick_Wait(800000);
  }
}

/* PortF Initialization function. All the details here come from the cryptic
   instructions in the data sheet */
void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) activate clock for Port F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 in, PF3-1 out
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_PUR_R = 0x11;          // enable pull-up on PF0 and PF4
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital I/O on PF4-0
}

/* PortE Initialization function. All the details here come from the cryptic
   instructions in the data sheet */
void PortE_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x10;      // 1) E
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTE_AMSEL_R &= ~0x07; // 3) disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   // 5) inputs on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 6) regular function on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE2-0
}

/* PortB Initialization function. All the details here come from the cryptic
   instructions in the data sheet */
void PortB_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x02;      // 1) B
	delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0
}