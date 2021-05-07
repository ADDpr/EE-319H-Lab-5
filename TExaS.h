// TExaS.h
// Runs on TM4C123
// Periodic timer Timer5A which will interact with debugger and implement logic analyzer 
// It initializes on reset and runs whenever interrupts are enabled
// Jonathan Valvano
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

void PLL_Init(void);

// ************TExaS_Init*****************
// Initialize 7-bit logic analyzer on timer 5A 100us
// sets PLL to 80 MHz
// This needs to be called once
// Inputs: function to send data
// This will only activate clock, user sets direction and other modes
// Outputs: none
void TExaS_Init(void(*task)(void));

// ************TExaS_Stop*****************
// Stop the transfer 
// Inputs:  none
// Outputs: none
void TExaS_Stop(void);
