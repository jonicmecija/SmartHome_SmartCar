#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define NVIC_EN0_R              (*((volatile unsigned long *)0xE000E100))  // IRQ 0 to 31 Set Enable Register
#define NVIC_PRI7_R             (*((volatile unsigned long *)0xE000E41C))  // IRQ 28 to 31 Priority Register
#define GPIO_PORTF_DIR_R        (*((volatile unsigned long *)0x40025400))
#define GPIO_PORTF_IS_R         (*((volatile unsigned long *)0x40025404))
#define GPIO_PORTF_IBE_R        (*((volatile unsigned long *)0x40025408))
#define GPIO_PORTF_IEV_R        (*((volatile unsigned long *)0x4002540C))
#define GPIO_PORTF_IM_R         (*((volatile unsigned long *)0x40025410))
#define GPIO_PORTF_RIS_R        (*((volatile unsigned long *)0x40025414))
#define GPIO_PORTF_ICR_R        (*((volatile unsigned long *)0x4002541C))
#define GPIO_PORTF_AFSEL_R      (*((volatile unsigned long *)0x40025420))
#define GPIO_PORTF_PUR_R        (*((volatile unsigned long *)0x40025510))
#define GPIO_PORTF_PDR_R        (*((volatile unsigned long *)0x40025514))
#define GPIO_PORTF_DEN_R        (*((volatile unsigned long *)0x4002551C))
#define GPIO_PORTF_AMSEL_R      (*((volatile unsigned long *)0x40025528))
#define GPIO_PORTF_PCTL_R       (*((volatile unsigned long *)0x4002552C))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
#define SYSCTL_RCGC2_GPIOF      0x00000020  // port F Clock Gating Control
#define GPIO_PORTF_LOCK_R       (*((volatile unsigned long *)0x40025520))
#define GPIO_PORTF_CR_R         (*((volatile unsigned long *)0x40025524))
	
#define GPIO_PORTA_DATA_R				(*((volatile unsigned long *)0x400043FC))
#define NVIC_PRI0_R 						(*((volatile unsigned long *)0xE000E400))
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_IS_R					(*((volatile unsigned long *)0x40004404))
#define GPIO_PORTA_IBE_R				(*((volatile unsigned long *)0x40004408))
#define GPIO_PORTA_IEV_R				(*((volatile unsigned long *)0x4000440C))
#define GPIO_PORTA_IM_R					(*((volatile unsigned long *)0x40004410))
#define GPIO_PORTA_RIS_R				(*((volatile unsigned long *)0x40004414))
#define GPIO_PORTA_ICR_R				(*((volatile unsigned long *)0x4000441C))
#define GPIO_PORTA_AFSEL_R			(*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_AMSEL_R			(*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R				(*((volatile unsigned long *)0x4000452C))
#define GPIO_PORTA_DEN_R				(*((volatile unsigned long *)0x4000451C))
	
#define NVIC_SYS_PRI3_R         (*((volatile unsigned long *)0xE000ED20))  // Sys. Handlers 12 to 15 Priority
#define NVIC_ST_CTRL_R          (*((volatile unsigned long *)0xE000E010))
#define NVIC_ST_RELOAD_R        (*((volatile unsigned long *)0xE000E014))
#define NVIC_ST_CURRENT_R       (*((volatile unsigned long *)0xE000E018))
	
#include "SysTick.h"
#include <stdint.h>
#include "stepper.h"

// FUNCTION DECLARATIONS
void PortF_Init(void);
void PortA_Init(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);
#define T1ms 3000 

// VARIABLE DECLARATIONS
unsigned int flag = 0;
unsigned int last = 0;
unsigned int stop = 0;
unsigned int sw1 = 0;
unsigned int sw2 = 0;
unsigned int go = 1;
unsigned int i = 0;

// MAIN
int main( void )
{  
	// PORT INITILIZATIONS
	PortF_Init();
	PortA_Init();
	EnableInterrupts();
  Stepper_Init();
	
	// SUPER LOOP
	while ( 1 ) 
	{
		// ********* SWITCH ONE PROCESSES ********* //
		if ( sw1 == 1 )
			{
				if( flag == 1 )
					{
						// GO FORWARD 720 DEGREES
						for ( i = 0; i < 4000; i++ ) 
							{ 
								Stepper_Forward(10*T1ms);
							}
							SysTick_Wait10ms(5);
						
						// TURN RIGHT 90 DEGREES
						for ( i = 0; i < 900; i++ )
							{
								Stepper_Left(10*T1ms);
							}
							SysTick_Wait10ms(5);
						
						flag = 0;
					}
				
				// GO FORWARD 
				if ( go == 1 )
					{
						Stepper_Forward(10*T1ms);
						
						// STOP IF OBJECT DETECTED
						if( stop == 1 )
							{
								SysTick_Wait10ms(5);
								go = 0;
							}
					} 
				// IF OBJECT IS REMOVED
				if ( last == 1 )
					{
						SysTick_Wait10ms(100*15);
						// MOVE FORWARD 360 DEGREES
						for ( i = 0; i < 2500; i++ )
							{
								Stepper_Forward(10*T1ms);
							}
							
						// STOP
						// RESET FLAGS
						SysTick_Wait10ms(5);
						last = 0;
						sw1 = 0;
					}
			}
			
		// ********  SWITCH TWO PROCESSES ********* //
		if ( sw2 == 1 )
			{
				// REVERSE 360 DEGREES
				for( i = 0; i < 2500; i++ )
					{
						Stepper_Back(10*T1ms);
					}
				SysTick_Wait10ms(5);
				
				// TURN RIGHT 90 DEGREES
				for( i = 0; i < 900; i++ )
					{
						Stepper_Right(10*T1ms);
					}
				SysTick_Wait10ms(5);
				
				// MOVE FORWARD 720 DEGREES
				for( i = 0; i < 4000; i++ )
					{
						Stepper_Forward(10*T1ms);
					}
				
				// STOP
				SysTick_Wait10ms(5);
				sw2 = 0;
			}
	}	
}
	
// SWITCH INTERRUPT 
void GPIOPortF_Handler(void)
{
	// IF SW 1 IS PRESSED TRIGGER FLAGS
	if( (GPIO_PORTF_RIS_R & 0x10) ) 
		{ 
			sw1 = 1;
			flag = 1;
			sw2 = 0;			
		  GPIO_PORTF_ICR_R = 0x10;
		}
	
	// IF SW 2 IS PRESSED TRIGGER FLAGS
	if( (GPIO_PORTF_RIS_R & 0x01) )
		{ 
			sw2 = 1;
			sw1 = 0;
			flag = 0;
	    GPIO_PORTF_ICR_R = 0x01;
		}
}

// IR SENSOR INTERRUPT
void GPIOPortA_Handler(void)
{
	// PIN 7 PORT A INPUT
	GPIO_PORTA_ICR_R = 0x80;
	
	// OBJECT IS APPROACHING
	if ( go == 1 )
		{ 
			stop = 1;
			last = 0;
		}
	// OBJECT IS DEPARTING
	if ( go == 0 )
		{ 
			go = 0;
			last = 1;
			stop = 0;
		}
}		

// PORT INITILIZATIONS
void PortF_Init(void)
{ 
	volatile unsigned long d;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
	d= SYSCTL_RCGC2_R;
	GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  // Unlock at beginning, broke code 
	GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0 
  //GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4 and PF0 in (built-in button)
	GPIO_PORTF_DIR_R = 0x0E;          // 5)PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R &= ~0x1F;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4  
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; // configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R = 0;       //     disable analog functionality on PF
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x11;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x11;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x11;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x11;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x11;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R |= 0x40000000;      // (h) enable interrupt 30 in NVIC
}

void PortA_Init(void)
{
	volatile unsigned long d;
	SYSCTL_RCGC2_R |= 0x00000001;
	d = SYSCTL_RCGC2_R;
	GPIO_PORTA_DIR_R &= ~0x80; // Input, PA7
	GPIO_PORTA_AFSEL_R &= ~0xFF;
	GPIO_PORTA_DEN_R |= 0x80;
	GPIO_PORTA_PCTL_R &= ~0xF0000000;
	GPIO_PORTA_AMSEL_R = 0;
	GPIO_PORTA_IS_R &= ~0x80;
	GPIO_PORTA_IBE_R |= 0x80; // Both edges
	GPIO_PORTA_ICR_R = 0x80;  
	GPIO_PORTA_IM_R |= 0x80;
	NVIC_PRI0_R = (NVIC_PRI0_R&0xFFFFFF00) | 0x00000080; //priority 4
	NVIC_EN0_R |= 0x00000001;
}