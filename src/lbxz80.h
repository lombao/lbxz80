/* Copyright (c) 2023 Cesar Lombao
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
 

#ifndef _LBXZ80_H
#define _LBXZ80_H

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>


/* type for the I/O functions */
typedef	void( *t_callback_readio)  ( const uint16_t addr, uint8_t * data );
typedef	void( *t_callback_writeio) ( const uint16_t addr, uint8_t data );
typedef	void( *t_callback_readmem)  ( const uint16_t addr, uint8_t * data );
typedef	void( *t_callback_writemem) ( const uint16_t addr, uint8_t data );


/* struct of signals */
/* 0 is OFF ( FALSE ), 1 is ON ( TRUE or ACTIVE ) */
struct z80signals {
	uint8_t NMI;  /* non maskeable interrupt */
	uint8_t INT;  /* maskeablre interrupt */
	uint8_t HALT; /* signal HALT */
	uint8_t RST;  /* signal RESET */ 
	
	uint8_t inst[10];  /* pipe with opcodes to fill it up with instructions for INT mode 0 */
	uint8_t ptrinst;   /* ptr to the next opcode to read, remember to set this to 0 when you call INT in mode 0 */	  
};
				
/****************************************************************/
/* Declaration Functions 					*/

/****************************************************************/
/* Z80 INIT 
	clockmhz --> Z80 speed in Mhz .i.e: 1.0, 2.0 ,3.5 ,3.7, 4.0 and so on.
	s        --> struct z80signals pointer. Please see the struct z80signals
				* INT  Interruption maskearable
				* NMI  Non maskearable
				* HALT Signal to stop/halt
				* RST  Signal to Reset
			In each instruciton cycle these signals will be checked and
			act accordingly
			
	rio ---> Callback. Pointer to a function to be called when a IN instruction happen
	wio ---> Callback. Pointer to the function to be called when an OUT instruciton happen
	rmem --> Callback for read mem operations
	wmem --> Callback for write mem operations
*/

void z80_init( float clockmhz, struct z80signals * s , t_callback_readio rio , t_callback_writeio wio , t_callback_readmem rmem , t_callback_writemem wmem );
/****************************************************************/


/****************************************************************/
/* Z80 RESET 
	Reset the CPU to the initial Values 
*/
void z80_reset();
/****************************************************************/



/****************************************************************/
/* Z80 Set Signals
	s -> Pointer to the struct z80signals 
*/
void z80_setsignals ( struct z80signals * s );
/****************************************************************/

/****************************************************************/
/* Z80 RUN
	Runs the CPU until end of execution or detect an INT or
	IN or OUT instructions
*/
int z80_run();
/****************************************************************/

#endif
