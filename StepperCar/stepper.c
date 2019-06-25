// Stepper.c
// Runs on LM4F120/TM4C123
// Provide functions that step the motor once clockwise, step
// once counterclockwise, and initialize the stepper motor
// interface.
// Daniel Valvano
// September 12, 2013
// Modified by Dr. Min He April 28, 2017

/* This example accompanies the book
   "Embedded Systems: Real Time Interfacing to ARM Cortex M Microcontrollers",
   ISBN: 978-1463590154, Jonathan Valvano, copyright (c) 2015
   Example 4.1, Programs 4.4, 4.5, and 4.6
   Hardware circuit diagram Figure 4.27

 Copyright 2015 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// PD3 connected to driver for stepper motor coil A
// PD2 connected to driver for stepper motor coil A'
// PD1 connected to driver for stepper motor coil B
// PD0 connected to driver for stepper motor coil B'

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "systick.h"

struct State
{
  uint8_t Out;     // Output
  uint8_t Next[4]; // CW/CCW
};

typedef const struct State StateType;

#define forward 0        // Next index clockwise
#define backwards 1      // Next index counter clock wise
#define left 3
#define right 2

StateType fsm[7] =
{
  { 0xCC, {1,3,4,6}},			// 12
  { 0x61, {2,0,0,0}},			// 6
  { 0x33, {3,1,0,0}},     // 3
  { 0x16, {0,2,0,0}},	    // 1
	{ 0x66, {0,0,5,0}},
	{ 0x33, {0,0,6,4}},
	{ 0x11, {0,0,0,5}}, 
};

unsigned char s; // current state

#define STEPPER   (*((volatile uint32_t *)0x400053FC))
	
// Move 1.8 degrees clockwise, delay is the time to wait after each step
void Stepper_Forward(uint32_t delay)
{
  s = fsm[s].Next[forward]; // clock wise circular
  STEPPER = fsm[s].Out; // step motor
  SysTick_Wait(delay);
}

// Move 1.8 degrees counterclockwise, delay is wait after each step
void Stepper_Back(uint32_t delay)
{
  s = fsm[s].Next[backwards]; // counter clock wise circular
  STEPPER = fsm[s].Out; // step motor
  SysTick_Wait(delay); // blind-cycle wait
}

void Stepper_Right(uint32_t delay)
{
  s = fsm[s].Next[right]; // counter clock wise circular
  STEPPER = fsm[s].Out; // step motor
  SysTick_Wait(delay); // blind-cycle wait
}

void Stepper_Left(uint32_t delay)
{
  s = fsm[s].Next[left]; // counter clock wise circular
  STEPPER = fsm[s].Out; // step motor
  SysTick_Wait(delay); // blind-cycle wait
}

// Initialize Stepper interface
void Stepper_Init(void)
{
  SYSCTL_RCGCGPIO_R |= 0x02; // 1) activate port B
  SysTick_Init();
  s = 0; 
                                    // 2) no need to unlock PD3-0
  GPIO_PORTB_AMSEL_R &= ~0xFF;      // 3) disable analog functionality on PD3-0
  GPIO_PORTB_PCTL_R &= ~0x0000FFFF; // 4) GPIO configure PD3-0 as GPIO
  GPIO_PORTB_DIR_R |= 0xFF;   // 5) make PD3-0 out
  GPIO_PORTB_AFSEL_R &= ~0xFF;// 6) disable alt funct on PD3-0
  GPIO_PORTB_DR8R_R |= 0xFF;  // enable 8 mA drive
  GPIO_PORTB_DEN_R |= 0xFF;   // 7) enable digital I/O on PD3-0 
}
