// TableTrafficLight.c solution to EE319K Lab 5, spring 2021
// Runs on TM4C123
// Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// January 17, 2021

/* 

 Copyright 2021 by Jonathan W. Valvano, valvano@mail.utexas.edu
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

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3-1 (built-in white LED)
// "don't walk" light connected to PF1 (built-in red LED)
#include <stdint.h>
#include "SysTick.h"
#include "TExaS.h"
#include "../inc/tm4c123gh6pm.h"



void DisableInterrupts(void);
void EnableInterrupts(void);

#define PB543210                (*((volatile uint32_t *)0x400050FC)) // bits 5-0
#define PE210                   (*((volatile uint32_t *)0x4002401C)) // bits 2-0
#define PF321                   (*((volatile uint32_t *)0x40025038)) // bits 3-1


void LogicAnalyzerTask(void){
  UART0_DR_R = 0x80|GPIO_PORTB_DATA_R;
}
struct State{
	uint8_t outPB;
	uint8_t outPF;
	uint32_t delay;
	const struct State *Next[8];
};
typedef const struct State State_t;

State_t *Pt;
uint32_t in;
#define Delay1s 100
#define Delay2s 200

#define goS &FSM[0]
#define waitS &FSM[1]
#define goW &FSM[2]
#define waitW &FSM[3]
#define walk &FSM[4]
#define walkR1 &FSM[5]
#define walkO1 &FSM[6]
#define walkR2 &FSM[7]
#define walkO2 &FSM[8]
#define walkR3 &FSM[9]
#define walkO3 &FSM[10]
#define NWtoS &FSM[11]
#define RStoW &FSM[12]
#define RStoPed &FSM[13]
#define RWtoS &FSM[14]
#define RWtoPed &FSM[15]
#define NWtoW &FSM[16]
State_t FSM[17]={
//															Next If
// OutputPB		OutputPF 	Delay		(PE)	In=000	In=001	In=010	In=011	In=100		In=101		In=110		In=111 
		{33,				2, 			Delay1s,				{goS,	waitS,	goS,		waitS,	waitS, 		waitS,		waitS,		waitS}},
		{34,				2, 			Delay1s,			{RStoW,	RStoW,	RStoW,	RStoW,	RStoPed, 	RStoPed,	RStoPed,	RStoPed}},
		{12,				2, 			Delay1s,				{goW,	goW,		waitW,	waitW,	waitW, 		waitW,		waitW,		waitW}},
		{20,				2, 			Delay1s,			{RWtoS,	RWtoS,	RWtoS,	RWtoS,	RWtoPed, 	RWtoPed,	RWtoPed,	RWtoS}},
		{36,				14, 		Delay1s,			{walk,		walkR1,	walkR1,	walkR1,	walk, 		walkR1,		walkR1,		walkR1}},
		{36,				2, 			Delay1s,			{walkO1,	walkO1,	walkO1,	walkO1,	walkO1,		walkO1,		walkO1,		walkO1}},
		{36,				0, 			Delay1s,			{walkR2,	walkR2,	walkR2,	walkR2,	walkR2,		walkR2,		walkR2,		walkR2}},
		{36,				2, 			Delay1s,			{walkO2,	walkO2,	walkO2,	walkO2,	walkO2,		walkO2,		walkO2,		walkO2}},
		{36,				0, 			Delay1s,			{walkR3,	walkR3,	walkR3,	walkR3,	walkR3,		walkR3,		walkR3,		walkR3}},
		{36,				2, 			Delay1s,			{walkO3,	walkO3,	walkO3,	walkO3,	walkO3,		walkO3,		walkO3,		walkO3}},
		{36,				0, 			Delay1s,			{NWtoS,	NWtoW,	NWtoS,	NWtoS,	NWtoS,		NWtoW,		NWtoS,		NWtoW}},
		{36,				2, 			Delay1s,			{goS,		goS,		goS,		goS,		goS,			goS,			goS,			goS}},
		{36,				2, 			Delay1s,			{goW,		goW,		goW,		goW,		goW,			goW,			goW,			goW}},
		{36,				2, 			Delay1s,			{walk,		walk,		walk,		walk,		walk,			walk,			walk,			walk}},
		{36,				2, 			Delay1s,			{goS,		goS,		goS,		goS,		goS,			goS,			goS,			goS}},
		{36,				2, 			Delay1s,			{walk,		walk,		walk,		walk,		walk,			walk,			walk,			walk}},
		{36,				2, 			Delay1s,			{goW,		goW,		goW,		goW,		goW,			goW,			goW,			goW}},
};

//run on sim
/*
int main(void){ volatile uint32_t delay;
  DisableInterrupts();
  //TExaS_Init(&LogicAnalyzerTask);
  PLL_Init();     // PLL on at 80 MHz
	SYSCTL_RCGC2_R |= 0x32;  // LM3S legacy clock register
  delay = SYSCTL_RCGC2_R;
*/

// run this on real board

int main(void){ volatile uint32_t delay;
	DisableInterrupts();
	TExaS_Init(&LogicAnalyzerTask);
	SYSCTL_RCGC2_R |= 0x32;  // LM3S legacy clock register
	delay = SYSCTL_RCGC2_R;

// **************************************************
// weird old bug in the traffic simulator
// run next two lines on real board to turn on F E B clocks
//  SYSCTL_RCGCGPIO_R |= 0x32;  // real clock register 
//  while((SYSCTL_PRGPIO_R&0x32)!=0x32){};
// run next two lines on simulator to turn on F E B clocks
	EnableInterrupts();
  SysTick_Init();   // Initialize SysTick for software waits
// **************************************************
	SYSCTL_RCGCGPIO_R |= 0x32; 
	while((SYSCTL_RCGCGPIO_R&0x32) != 0x32){};
	GPIO_PORTE_DIR_R &= ~0x07;
	GPIO_PORTE_DEN_R |= 0x07;
	GPIO_PORTB_DIR_R |= 0x3F; 
	GPIO_PORTB_DEN_R |= 0x3F;
	GPIO_PORTF_DIR_R |= 0x0E;
	GPIO_PORTF_DEN_R |= 0x0E;
	
  Pt = goS;
  while(1){
		GPIO_PORTB_DATA_R = Pt->outPB;// output to port B
		GPIO_PORTF_DATA_R = Pt->outPF;// output to port F
		SysTick_Wait10ms(Pt->delay);// wait
		in = (GPIO_PORTE_DATA_R&0x07); //PE0-2
		Pt = Pt->Next[in]; // input

  }
}



