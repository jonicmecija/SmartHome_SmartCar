#define GPIO_PORTF_DATA_R       (*((volatile unsigned long *)0x400253FC))
#define LIGHT										(*((volatile unsigned long *)0x40025038))
	
#define RED 0x02;
#define BLUE 	0x04;
#define GREEN 0x08;
#include <stdint.h>
#include "stepper.h"
#include "SysTick.h"
#include "tm4c123gh6pm.h"
#define T1ms 16000 

/*
WANT:
TIMER INTERRUPT - INIT @ 2Hz FOR 3 SECONDS
EXTERNAL INTERRUPT
*/

// 2. Declarations Section
//   Function Prototypes
void PortF_Init(void);
void PortA_Init(void);
void EnableInterrupts(void);
void WaitForInterrupt(void);
//void WaitForInterrupt(void);  // low power mode

// 3. Subroutines Section
unsigned int detectApproach = 0;
unsigned int detectDepart = 0;
unsigned int trigger = 0;
unsigned int flag = 0;
unsigned int Counter = 0;
unsigned int Count =0;

// MAIN: Mandatory for a C Program to be executable
int main( void )
{   unsigned int i=0;
	//TExaS_Init(SW_PIN_PF40,LED_PIN_PF321); // this initializes the TExaS grader lab 2

	PortF_Init();
	PortA_Init();
  SysTick_Init(); // WANT PERIOD OF 1/2 FOR 3 SEC, x12, HALF PERIOD HIGH/LOW
	EnableInterrupts();
  Stepper_Init();
	
	LIGHT = GREEN;
	
  while(1)
		{
			if(detectApproach == 0xFF | trigger == 1)
				{
					detectApproach = 0x00;
					
					if (flag == 0) 
						{
							trigger=0;
							
							for(i = 0; i < 5000; i++)
								{
									Stepper_CW(40000); 
								}				
								Counter =0;			
								flag = 1;
						}					
						LIGHT = 0x04;		
				}
			
		   if(detectDepart == 0xFF | trigger == 2)
				{	
					detectDepart = 0x00;
					if (flag == 1) 
						{
							trigger = 0;
							SysTick_Wait10ms(100*3);
							for(i = 0; i < 4300; i++)
								{
									Stepper_CCW(60000);
								}				 
							Counter = 0;
							flag = 0;
						}					
						LIGHT = 0x08;		
				}			
	}			
}


//*****************************
// 		PORT INITIALIZATIONS
//*****************************

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


//*****************************
// 		INTERRUPT HANDLERS
//*****************************

void GPIOPortF_Handler(void)
{
	if( (GPIO_PORTF_RIS_R & 0x10) ) // Check SW1 is pressed
		{
			GPIO_PORTF_ICR_R = 0x10;
			
		 if ( LIGHT== 0x08 )
			{
				trigger = 1;
				Counter =1;
				Count =0;
				LIGHT = GREEN;
				detectApproach = 0x00;

			}
			
		 if ( LIGHT == 0x04 )
			{
				trigger = 2;
				Counter =1;
				Count=0;
				LIGHT = BLUE;
				detectDepart = 0x00;
			}
		}
}

void GPIOPortA_Handler(void)
{
	GPIO_PORTA_ICR_R = 0x80; // acknowledge
	// Determine previous value of LED
	
	if( LIGHT == 0x08 )
	{
		detectApproach = 0xFF;
		Count = 0;
		Counter = 1;
	}
	
 if ( LIGHT == 0x04 )
	{
		detectDepart = 0xFF;
		Count = 0;

		Counter =1 ;
	}
	
	if (trigger == 1){ detectApproach = 0x00;}

	if (trigger == 2) {	detectDepart = 0x00;}
}
void SysTick_Handler(void)
{
		Count = Count +1;
		if (Counter  == 1)
		{			
			if (Count == 50) // half a sec 
		{
			GPIO_PORTF_DATA_R ^= 0x02;
			GPIO_PORTF_DATA_R &= ~0x0D;//clear
			
			Count = 0;
		}
	}
}	