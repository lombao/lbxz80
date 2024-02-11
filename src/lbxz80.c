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
 

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <time.h>

#include "lbxz80.h"

/**********************************************************************/
/* Global storage                                                     */


t_callback_readio read_io;
t_callback_writeio write_io;
t_callback_readmem read_mem;
t_callback_writemem write_mem;

float z80_clock; /* in Mhz: 1, 2, 3.5, 4 */

struct z80signals * signals;
int flag_int_signal = 0;


  int iff1;
  int iff2;
  int imode;  
  uint8_t inst;
  int ts;

 
struct { 
	uint16_t PC; 
	uint16_t SP;
	uint16_t BC;
	uint16_t DE;
	uint16_t HL;
	uint16_t IR;
	uint16_t AF;
	uint16_t IX;
	uint16_t IY;
	uint16_t BCPLUS;
	uint16_t DEPLUS;
	uint16_t HLPLUS;
	uint16_t AFPLUS;	
	uint16_t WZ;
} regs;


/**********************************************************************/
#define REG_A (*(uint8_t *)(&regs.AF))
#define REG_B (*(uint8_t *)(&regs.BC))
#define REG_C (*((uint8_t *)(&regs.BC)+1))
#define REG_D (*(uint8_t *)(&regs.DE))
#define REG_E (*((uint8_t *)(&regs.DE)+1))
#define REG_H (*(uint8_t *)(&regs.HL))
#define REG_L (*((uint8_t *)(&regs.HL)+1))
#define REG_I (*(uint8_t *)(&regs.IR))
#define REG_R (*((uint8_t *)(&regs.IR)+1))
#define REG_W (*(uint8_t *)(&regs.WZ))
#define REG_Z (*((uint8_t *)(&regs.WZ)+1))

#define RAM_WRITE_BYTE(ADDR,V) write_mem(ADDR,V)
#define RAM_READ_BYTE(ADDR,V)  read_mem(ADDR,&V)
#define RAM_WRITE_WORD(ADDR,V) RAM_WRITE_BYTE(ADDR,V&0x00FF); RAM_WRITE_BYTE(ADDR+1,V>>8)  
#define RAM_READ_WORD(ADDR,V)  RAM_READ_BYTE(ADDR,(*(uint8_t *)&V)); RAM_READ_BYTE(ADDR+1,(*(((uint8_t *)&V)+1))) 

#define RAM_FETCH_WORD(V)	if (flag_int_signal) { V=(signals->inst[signals->ptrinst] | (signals->inst[signals->ptrinst+1]<<8)); signals->ptrinst += 2;} else { RAM_READ_WORD(regs.PC,V); regs.PC += 2; }
#define RAM_FETCH_BYTE(V)   if (flag_int_signal) { V=signals->inst[signals->ptrinst++]; } else { RAM_READ_BYTE(regs.PC,V); regs.PC++; }

#define PUSH_PC PUSH_REG16(regs.PC)
#define POP_PC POP_REG16(regs.PC)
#define PUSH_REG16(v)		regs.WZ = v; RAM_WRITE_BYTE(--regs.SP,REG_Z); RAM_WRITE_BYTE(--regs.SP,REG_W); 
#define POP_REG16(v)		RAM_READ_BYTE(++regs.SP,REG_W); RAM_READ_BYTE(++regs.SP,REG_Z); v = regs.WZ;

#define ADDTS(V)  ts += V;

#define CX(V) regs.AF = V? regs.AF | 0x0000000000000001 : regs.AF & 0x1111111111111110
#define NX(V) regs.AF = V? regs.AF | 0x0000000000000010 : regs.AF & 0x1111111111111101  
#define XX(V) regs.AF = V? regs.AF | 0x0000000000001000 : regs.AF & 0x1111111111110111
#define HX(V) regs.AF = V? regs.AF | 0x0000000000010000 : regs.AF & 0x1111111111101111  
#define PX(V) regs.AF = V? regs.AF | 0x0000000000000100 : regs.AF & 0x1111111111111011  
#define YX(V) regs.AF = V? regs.AF | 0x0000000000100000 : regs.AF & 0x1111111111011111  
#define ZX(V) regs.AF = V? regs.AF | 0x0000000001000000 : regs.AF & 0x1111111110111111  
#define SX(V) regs.AF = V? regs.AF | 0x0000000010000000 : regs.AF & 0x1111111101111111  

#define F_C (regs.AF & 0x0000000000000001)
#define F_N ((regs.AF & 0x0000000000000010) >> 1)
#define F_P ((regs.AF & 0x0000000000000100) >> 2)
#define F_X ((regs.AF & 0x0000000000001000) >> 3)
#define F_H ((regs.AF & 0x0000000000010000) >> 4)
#define F_Y ((regs.AF & 0x0000000000100000) >> 5)
#define F_Z ((regs.AF & 0x0000000010000000) >> 6)
#define F_S ((regs.AF & 0x0000000100000000) >> 7)
	
#define ALU_ADC(r)    NX(0); PX((! ((REG_A >> 7)^(F_C)) )  && (((REG_A+F_C+r)>>7)!=(REG_A>>7))); HX(((REG_A&0x0F)+(r&0x0F) + F_C ) > 0x0F ); YX((uint16_t)(REG_A + r + F_C) > 0x00FF); REG_A = REG_A + r + F_C; CX(F_Y);ZX(REG_A==0);SX(REG_A>>7);YX(REG_A>>5);XX(REG_A>>3);
#define ALU_ADC_HL(r) NX(0); PX( (!((regs.HL >> 15)  ^ ((uint16_t)(r+F_C) >> 15))) && ( ( (regs.HL + r + F_C) >> 15) != (regs.HL >> 15) ) ); CX((uint32_t)(regs.HL + r + F_C) > 0x0000FFFF); regs.HL = regs.HL + r + F_C ; ZX(regs.HL==0); SX(regs.HL >> 15);
#define ALU_AND(r)    NX(0); HX(1); REG_A &=r; CX(0); PX(z80_alu_check_parity(REG_A)); ZX(REG_A==0); SX(REG_A>>7); YX(REG_A>>5); XX(REG_A>>3);
#define ALU_ADD_16(r1,r2) NX(0); CX((uint32_t)(r1 + r2) > 0x0000FFFF); r1 += r2;
#define ALU_ADD(r)	  NX(0); HX(((REG_A&0x0F) + (r & 0x0F)) > 0x0F); CX((uint16_t)(REG_A + r) > 0x00FF); PX((!((REG_A >> 7) ^ (r >> 7))) && (((REG_A+r) >> 7) != (REG_A >> 7))); REG_A += r; ZX(REG_A==0); SX(REG_A>>7); YX(REG_A>>5); XX(REG_A>>3);
#define ALU_BIT(r,b)  NX(0); HX(1); ZX( (r & (1 << b) ) == 0); SX( b == 7 ) && ( r >> 7 ); YX(( b == 5)  && ( b & 0x20 )); XX(( b == 3 ) && ( r & 0x04 )); PX((r & (1 << b) ) == 0);
#define ALU_CP(r)     NX(1); HX(( REG_A & 0x0F) < (r & 0x0F));CX(REG_A < r);PX(((REG_A >> 7)  ^ (r >> 7)) && ( REG_A < r ));ZX(REG_A == r); SX((REG_A - r)>>7 );  YX( (REG_A - r ) >> 5); XX( (REG_A - r )>> 3 );		
#define ALU_DEC(r)    NX(1);HX((r & 0x0F)==0);PX(r==0x80);r--; ZX(r==0);SX(r>>7);YX(r>>5);XX(r>>3);
#define ALU_INC(r)    NX(0);HX((r & 0x0F)==0x0F);PX(r==0x7F);r++;ZX(r==0);SX(r>>7);YX(r>>5);XX(r>>3);
#define ALU_OR(r)     NX(0);HX(1); REG_A|=r;;CX(0);PX(z80_alu_check_parity(REG_A));ZX(REG_A==0);SX(REG_A>>7);YX(REG_A>>5);XX(REG_A>>3);
#define ALU_RES(r,b)   r = r & (~(1 << b))
#define ALU_RLC(r)    NX(0);HX(0);CX(r >> 7);r = (r<<1)|(r >> 7); PX(z80_alu_check_parity(r)); ZX(r==0);SX(r>>7);YX(r>>5);XX(r>>3); 
#define ALU_RL(r)     NX(0);HX(0);CX(r >> 7);r = (r<<1) | F_C;  PX(z80_alu_check_parity(r)); ZX(r==0);SX(r>>7);YX(r>>5);XX(r>>3);
#define ALU_RR(r)     NX(0);HX(0);CX(r & 0x01);r = (r >> 1)|(F_C << 7); PX(z80_alu_check_parity(r)); ZX(r==0);SX(r>>7);YX(r>>5);XX(r>>3);		   					
#define ALU_RRC(r)    NX(0);HX(0);CX(r & 0x01);r = (r >> 1)|(r << 7); PX(z80_alu_check_parity(r)); ZX(r==0);SX(r>>7);YX(r>>5);XX(r>>3);
#define ALU_SBC(r)    NX(1);HX( (REG_A&0x0F) < (r & 0x0F) ) || ( (REG_A&0x0F) < ( (r+F_C) & 0x0F) ) ;PX( ((REG_A >> 7)  ^ ((uint8_t)(r+F_C) >> 7)) && (REG_A < (uint8_t)(r+F_C) ));YX ( (uint16_t)REG_A < (uint16_t)(r + F_C) );REG_A = REG_A - r - F_C; CX(F_Y);ZX(REG_A==0); SX(REG_A>>7);	YX(REG_A>>5); XX(REG_A>>3);
#define ALU_SBC_HL(r) NX(1);PX( ((regs.HL >> 15)  ^ ((uint16_t)(r+F_C) >> 15)) && ( regs.HL < (uint16_t)(r+F_C ) ) ); YX( (uint32_t)regs.HL < (uint32_t)(r + F_C) ); regs.HL = regs.HL - r - F_C; CX(F_Y);ZX(regs.HL==0);SX(regs.HL >> 15);
#define ALU_SET(r,b) r = (r | (1 << b))
#define ALU_SLA(r)    NX(0);HX(0);CX(r >> 7); r=(r<<1); PX(z80_alu_check_parity(r)); ZX(r==0);SX(r>>7);YX(r>>5);XX(r>>3);
#define ALU_SRA(r)    NX(0);HX(0);CX( r&0x01 ); r=(r >> 1) | ( r & 0x80); PX(z80_alu_check_parity(r));ZX(r==0);SX(r>>7);YX(r>>5);XX(r>>3);
#define ALU_SRL(r)    NX(0);HX(0);CX( r&0x01 ); r=(r >> 1); PX(z80_alu_check_parity(r));ZX(r==0);SX(r>>7);YX(r>>5);XX(r>>3);
#define ALU_SUB(r)    NX(1);PX(((REG_A>>7)^(r>>7))&&(REG_A<r));HX((REG_A&0x0F)<(r&0x0F));CX(REG_A < r); REG_A -= r;ZX(REG_A==0);SX(REG_A>>7);YX(REG_A>>5);XX(REG_A>>3);
#define ALU_XOR(r)    NX(0);HX(0);CX(0); REG_A ^= r; PX(z80_alu_check_parity(REG_A));ZX(r==0);SX(r>>7);YX(r>>5);XX(r>>3);

				
/**********************************************************************/
/* Declaration Functions */
void z80_decode();
void z80_decode_ddfd(uint16_t * hlidx);
void z80_decode_cb();
void z80_decode_ed();
int z80_alu_check_parity( uint8_t input );
void z80_alu_daa();


//----------------------------------------------------------------------

void z80_init( float clockmhz, struct z80signals * s, t_callback_readio rio , t_callback_writeio wio, t_callback_readmem rmem , t_callback_writemem wmem ) {

	
	z80_clock = clockmhz;
	z80_setsignals(s); 
	read_io = rio;
	write_io = wio;
	read_mem = rmem;
	write_mem = wmem;
	z80_reset();
	
} 

void z80_setsignals( struct z80signals * s ) 	{ signals = s; }

void z80_reset () {

	regs.PC	= 0;
	regs.IR	= 0;
	
	 iff1	= 0;
	 iff2	= 0;
		
	regs.AF = 0xFFFF;
	regs.SP	= 0xFFFF; 
	
	regs.BC = 0x0;
	regs.DE = 0x0;
	regs.HL = 0x0;
	regs.IX = 0x0;
	regs.IY = 0x0;
	
	regs.AFPLUS = 0x0;
	regs.BCPLUS = 0x0;
	regs.DEPLUS = 0x0;
	regs.HLPLUS = 0x0;
	
	 imode = 0;
	
	signals->RST=0;
	signals->NMI=0;
	signals->HALT=0;
	signals->INT=0;

}


 

//---------------------------------------------------------------------
int z80_run() {

 struct timespec tpstart;
 struct timespec tpend;
 struct timespec sleep;
 int realclock=0; 	/* in nanoseconds real time passed */
 int virtclock=0; 	/* in nanoseconds the calculated time spent based on TS*/

	sleep.tv_sec = 0; /* we will ignore the seconds part, always set to 0 */
	clock_gettime(CLOCK_REALTIME,&tpstart);

	int counter=0;
	#define CHECK_TIME_COUNTER 100
	while( !signals->HALT ) {
		
		if ( signals->RST ) { z80_reset(); continue; /* reset */ }
		if ( signals->NMI ) { iff2 = iff1; iff1 = 0; PUSH_PC; regs.PC=0x0066; signals->NMI = 0; continue; } /* NMI */
		if ( signals->INT && iff1 && imode == 0 ) { iff2 =  iff1 = 0; flag_int_signal=1; z80_decode(); signals->INT = 0; flag_int_signal=0; continue; } /* int mode 0 */  
		if ( signals->INT && iff1 && imode == 1 ) { PUSH_PC; regs.PC = 0x0038; signals->INT = 0; continue; } /* int mode 1 */ 
		if ( signals->INT && iff1 && imode == 2 ) { PUSH_PC; flag_int_signal = 1; RAM_FETCH_BYTE(REG_Z); REG_Z &= 0xFE; REG_W = REG_I; RAM_READ_WORD(regs.WZ,regs.PC); flag_int_signal  = 0; signals->INT = 0; continue; } /* int mode 2 */
		
		z80_decode();
		
		/* check if we should sync every CHECK_TIME_COUNTER instructions */
		if ( counter++ > CHECK_TIME_COUNTER ) {
			counter=0;
			clock_gettime(CLOCK_REALTIME,&tpend);
			realclock += tpend.tv_nsec - tpstart.tv_nsec;
			virtclock += ts *  (1000 / z80_clock ); /* normalized to nanoseconds */
			
			if ( virtclock > realclock ) {
				tpstart = tpend; ts = 0;
				sleep.tv_nsec = virtclock - realclock;
				nanosleep(&sleep,NULL); /* slow down a bit */
			}	
		}
	}
	
	return(EXIT_SUCCESS);
}


//---------------------------------------------------------------------
void z80_decode() {

	RAM_FETCH_BYTE( inst);
	switch( inst) {
			case 0x00: ADDTS(4); break; /* NOP */
			case 0x27: z80_alu_daa(); ADDTS(4); break; /* DAA */
			
			case 0x01: RAM_FETCH_WORD(regs.BC); ADDTS(10); break; /* LD BC,nn */ 
			case 0x11: RAM_FETCH_WORD(regs.DE); ADDTS(10); break; /* LD DE,nn */
			case 0x21: RAM_FETCH_WORD(regs.HL); ADDTS(10); break; /* LD HL,nn */
			case 0x31: RAM_FETCH_WORD(regs.SP); ADDTS(10); break; /* LD SP,nn */

			case 0x02: 	RAM_WRITE_BYTE(regs.BC,REG_A); ADDTS(7); break; /* LD (BC),A */
			case 0x12: 	RAM_WRITE_BYTE(regs.DE,REG_A); ADDTS(7); break; /* LD (DE),A */
			case 0x36:	RAM_FETCH_BYTE(REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(10); break; /* LD (HL),n */
	
			case 0x77:	RAM_WRITE_BYTE(regs.HL,REG_A); ADDTS(7); break; /* LD (HL),A */
			case 0x70:	RAM_WRITE_BYTE(regs.HL,REG_B); ADDTS(7); break; /* LD (HL),B */
			case 0x71:	RAM_WRITE_BYTE(regs.HL,REG_C); ADDTS(7); break; /* LD (HL),C */
			case 0x72:	RAM_WRITE_BYTE(regs.HL,REG_D); ADDTS(7); break; /* LD (HL),D */
			case 0x73:	RAM_WRITE_BYTE(regs.HL,REG_E); ADDTS(7); break; /* LD (HL),E */
			case 0x74:	RAM_WRITE_BYTE(regs.HL,REG_H); ADDTS(7); break; /* LD (HL),H */
			case 0x75:	RAM_WRITE_BYTE(regs.HL,REG_L); ADDTS(7); break; /* LD (HL),L */
	
			case 0x3E:	RAM_FETCH_BYTE(REG_A); ADDTS(7); break; /* LD A,n */
			case 0x06:	RAM_FETCH_BYTE(REG_B); ADDTS(7); break; /* LD B,n */
			case 0x0E:	RAM_FETCH_BYTE(REG_C); ADDTS(7); break; /* LD C,n */
			case 0x16:	RAM_FETCH_BYTE(REG_D); ADDTS(7); break; /* LD D,n */
			case 0x1E:	RAM_FETCH_BYTE(REG_E); ADDTS(7); break; /* LD E,n */
			case 0x26:	RAM_FETCH_BYTE(REG_H); ADDTS(7); break; /* LD H,n */
			case 0x2E:	RAM_FETCH_BYTE(REG_L); ADDTS(7); break; /* LD L,n */

			case 0x7F:	ADDTS(4); break; /* LD A,A */
			case 0x78:	REG_B = REG_A; ADDTS(4); break; /* LD A, B */
			case 0x79:	REG_C = REG_A; ADDTS(4); break; /* LD A, C */
			case 0x7A:	REG_D = REG_A; ADDTS(4); break; /* LD A, D */
			case 0x7B:	REG_E = REG_A; ADDTS(4); break; /* LD A, E */
			case 0x7C:	REG_H = REG_A; ADDTS(4); break; /* LD A, H */
			case 0x7D:	REG_L = REG_A; ADDTS(4); break; /* LD A, L */
			
			case 0x47:	REG_B = REG_A; ADDTS(4); break; /* LD B, A */
			case 0x40:	ADDTS(4); break; /* LD B, B */
			case 0x41:	REG_C = REG_A; ADDTS(4); break; /* LD C, A */
			case 0x42:	REG_D = REG_A; ADDTS(4); break; /* LD D, A */
			case 0x43:	REG_E = REG_A; ADDTS(4); break; /* LD E, A */
			case 0x44:	REG_H = REG_A; ADDTS(4); break; /* LD H, A */
			case 0x45:	REG_L = REG_A; ADDTS(4); break; /* LD L, A */
			
			case 0x4F:	REG_C = REG_A; ADDTS(4); break; /* LD C, A */
			case 0x48:	REG_C = REG_B; ADDTS(4); break; /* LD C, B */
			case 0x49:	ADDTS(4); break; /* LD C,C */
			case 0x4A:	REG_C = REG_D; ADDTS(4); break; /* LD C, D */
			case 0x4B:	REG_C = REG_E; ADDTS(4); break; /* LD C, E */
			case 0x4C:	REG_C = REG_H; ADDTS(4); break; /* LD C, H */
			case 0x4D:	REG_C = REG_L; ADDTS(4); break; /* LD C, L */
			
			case 0x57:	REG_D = REG_A; ADDTS(4); break; /* LD D, A */
			case 0x50:	REG_D = REG_B; ADDTS(4); break; /* LD D, B */
			case 0x51:	REG_D = REG_C; ADDTS(4); break; /* LD D, C */
			case 0x52:	ADDTS(4); break; /* LD D, D */
			case 0x53:	REG_D = REG_E; ADDTS(4); break; /* LD D, E */
			case 0x54:	REG_D = REG_H; ADDTS(4); break; /* LD D, H */
			case 0x55:	REG_D = REG_L; ADDTS(4); break; /* LD D, L */
			
			case 0x5F:	REG_E = REG_A; ADDTS(4); break; /* LD E, A */
			case 0x58:	REG_E = REG_B; ADDTS(4); break; /* LD E, B */
			case 0x59:	REG_E = REG_C; ADDTS(4); break; /* LD E, C */
			case 0x5A:	REG_E = REG_D; ADDTS(4); break; /* LD E, D */
			case 0x5B:	ADDTS(4); break; /* LDD E, E */
			case 0x5C:	REG_E = REG_H; ADDTS(4); break; /* LD E, H */
			case 0x5D:	REG_E = REG_L; ADDTS(4); break; /* LD E, L */
			
			case 0x67:	REG_H = REG_A; ADDTS(4); break; /* LD H, A */
			case 0x60:	REG_H = REG_B; ADDTS(4); break; /* LD H, B */
			case 0x61:	REG_H = REG_C; ADDTS(4); break; /* LD H, C */
			case 0x62:	REG_H = REG_D; ADDTS(4); break; /* LD H, D */
			case 0x63:	REG_H = REG_E; ADDTS(4); break; /* LD H, E */
			case 0x64:	ADDTS(4); break; /* LD H,H */
			case 0x65:	REG_H = REG_L; ADDTS(4); break; /* LD H, L */
			
			case 0x6F:	REG_L = REG_A; ADDTS(4); break; /* LD L, A */
			case 0x68:	REG_L = REG_B; ADDTS(4); break; /* LD L, B */
			case 0x69:	REG_L = REG_C; ADDTS(4); break; /* LD L, C */
			case 0x6A:	REG_L = REG_D; ADDTS(4); break; /* LD L, D */
			case 0x6B:	REG_L = REG_E; ADDTS(4); break; /* LD L, E */
			case 0x6C:	REG_L = REG_H; ADDTS(4); break; /* LD L, H */
			case 0x6D:	ADDTS(4); break; /* LDD H,H */
		
			case 0x88:  ALU_ADC(REG_B); ADDTS(4); break; /* ADC A,B */
			case 0x89:  ALU_ADC(REG_C); ADDTS(4); break; /* ADC A,C */
			case 0x8A:  ALU_ADC(REG_D); ADDTS(4); break; /* ADC A,D */
			case 0x8B:  ALU_ADC(REG_E); ADDTS(4); break; /* ADC A,E */
			case 0x8C:  ALU_ADC(REG_H); ADDTS(4); break; /* ADC A,H */
			case 0x8D:  ALU_ADC(REG_L); ADDTS(4); break; /* ADC A,L */
			case 0x8F:  ALU_ADC(REG_A); ADDTS(4); break; /* ADC A,A */
			case 0xCE:  RAM_FETCH_BYTE(REG_Z); ALU_ADC(REG_Z); ADDTS(7); break; 	/* ADC A,n */
			case 0x8E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_ADC(REG_Z);ADDTS(7); break; 					/* ADC A,(HL) */
		
			case 0x86:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_ADD(REG_Z);  ADDTS(7); break; /* ADD A,(HL) */
			case 0xC6:  RAM_FETCH_BYTE(REG_Z); ALU_ADD(REG_Z); ADDTS(7); break; /* ADD A,n */
			case 0x80:  ALU_ADD(REG_B); ADDTS(4); break; /* ADD A,B */
			case 0x81:  ALU_ADD(REG_C); ADDTS(4); break; /* ADD A,C */
			case 0x82:  ALU_ADD(REG_D); ADDTS(4); break; /* ADD A,D */
			case 0x83:  ALU_ADD(REG_E); ADDTS(4); break; /* ADD A,E */
			case 0x84:  ALU_ADD(REG_H); ADDTS(4); break; /* ADD A,H */
			case 0x85:  ALU_ADD(REG_L); ADDTS(4); break; /* ADD A,L */
			case 0x87:  ALU_ADD(REG_A); ADDTS(4); break; /* ADD A,A */

			case 0x09:  ALU_ADD_16(regs.HL,regs.BC); ADDTS(11); break; /*  ADD HL,BC */
			case 0x19:  ALU_ADD_16(regs.HL,regs.DE); ADDTS(11); break; /*  ADD HL,DE */
			case 0x29:  ALU_ADD_16(regs.HL,regs.HL); ADDTS(11); break; /*  ADD HL,HL */
			case 0x39:  ALU_ADD_16(regs.HL,regs.SP); ADDTS(11); break; /*  ADD HL,SP */

			case 0xA6:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_AND(REG_Z); ADDTS(7); break; /* AND A,(HL) */
			case 0xE6:  RAM_FETCH_BYTE(REG_Z); ALU_AND(REG_Z);  ADDTS(7); break; /* AND A,n */
			case 0xA0:  ALU_AND( REG_B ); ADDTS(4); break; /* AND A,B */
			case 0xA1:  ALU_AND( REG_C ); ADDTS(4); break; /* AND A,C */
			case 0xA2:  ALU_AND( REG_D ); ADDTS(4); break; /* AND A,D */
			case 0xA3:  ALU_AND( REG_E ); ADDTS(4); break; /* AND A,E */
			case 0xA4:  ALU_AND( REG_H ); ADDTS(4); break; /* AND A,H */
			case 0xA5:  ALU_AND( REG_L ); ADDTS(4); break; /* AND A,L */
			case 0xA7:  ALU_AND( REG_A ); ADDTS(4); break; /* AND A,A */

			case 0xC4:	if ( F_Z ) { ADDTS(10); regs.PC += 2; } else { PUSH_PC; RAM_FETCH_WORD( regs.PC ); ADDTS(17); };  break; /* CALL NZ,dd */
			case 0xCC:  if ( F_Z ) { PUSH_PC; RAM_FETCH_WORD( regs.PC ); ADDTS(17); } else { ADDTS(10); regs.PC += 2; };  break; /* CALL Z,dd */
			case 0xD4:	if ( F_C ) { ADDTS(10); regs.PC += 2; } else { PUSH_PC; RAM_FETCH_WORD( regs.PC ); ADDTS(17); };  break; /* CALL NC,dd */
			case 0xDC:  if ( F_C ) { PUSH_PC; RAM_FETCH_WORD( regs.PC ); ADDTS(17); } else { ADDTS(10); regs.PC += 2; };  break; /* CALL C,dd */
			case 0xE4:	if ( F_P ) { ADDTS(10); regs.PC += 2; } else { PUSH_PC; RAM_FETCH_WORD( regs.PC ); ADDTS(17); };  break; /* CALL PO,dd */
			case 0xEC:  if ( F_P ) { PUSH_PC; RAM_FETCH_WORD( regs.PC ); ADDTS(17); } else { ADDTS(10); regs.PC += 2; };  break; /* CALL PE,dd */
			case 0xF4:	if ( F_S ) { ADDTS(10); regs.PC += 2; } else { PUSH_PC; RAM_FETCH_WORD( regs.PC ); ADDTS(17); };  break; /* CALL P,dd */
			case 0xFC:  if ( F_S ) { PUSH_PC; RAM_FETCH_WORD( regs.PC ); ADDTS(17); } else { ADDTS(10); regs.PC += 2; };  break; /* CALL M,dd */
			case 0xCD:  PUSH_PC; RAM_FETCH_WORD( regs.PC ); ADDTS(17); break; /* CALL dd */
	
			case 0x37:	CX(1); HX(0); NX(0); ADDTS(4); break; /* SCF */
			case 0x3F:	CX(F_C == 0); NX(0); ADDTS(4); break; /* CCF */
			case 0x2F:	REG_A = ~REG_A; HX(1); NX(1); ADDTS(4); break; /* CPL */

			case 0xBE:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_CP(REG_Z); ADDTS(7); break; /* CP (HL) */
			case 0xFE:  RAM_FETCH_BYTE(REG_Z); ALU_CP (REG_Z); ADDTS(7);break; /* CP n */
			case 0xB8:  ALU_CP( REG_B ); ADDTS(4); break; /* CP B */
			case 0xB9:  ALU_CP( REG_C ); ADDTS(4); break; /* CP C */
			case 0xBA:  ALU_CP( REG_D ); ADDTS(4); break; /* CP D */
			case 0xBB:  ALU_CP( REG_E ); ADDTS(4); break; /* CP E */
			case 0xBC:  ALU_CP( REG_H ); ADDTS(4); break; /* CP H */ 
			case 0xBD:  ALU_CP( REG_L ); ADDTS(4); break; /* CP L */
			case 0xBF:  ALU_CP( REG_A ); ADDTS(4); break; /* CP A */
		
			case 0xDD: z80_decode_ddfd(&regs.IX); break; /* IX */
			case 0xFD: z80_decode_ddfd(&regs.IY); break; /* IY */							
			case 0xCB: z80_decode_cb(); break; /* CB */
			case 0xED: z80_decode_ed(); break; /* ED */
			case 0x35:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_DEC(REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(11);break; /* DEC (HL) */
			case 0x05:  ALU_DEC( REG_B ); ADDTS(4); break; /* DEC B */
			case 0x0D:  ALU_DEC( REG_C ); ADDTS(4); break; /* DEC C */
			case 0x15:  ALU_DEC( REG_D ); ADDTS(4); break; /* DEC D */
			case 0x1D:  ALU_DEC( REG_E ); ADDTS(4); break; /* DEC E */
			case 0x25:  ALU_DEC( REG_H ); ADDTS(4); break; /* DEC H */
			case 0x2D:  ALU_DEC( REG_L ); ADDTS(4); break; /* DEC L */
			case 0x3D:  ALU_DEC( REG_A ); ADDTS(4); break; /* DEC A */
			case 0x0B:	regs.BC--; ADDTS(6); break; /* DEC BC */
			case 0x1B:	regs.DE--; ADDTS(6); break; /* DEC DE */
			case 0x2B:	regs.HL--; ADDTS(6); break; /* DEC HL */
			case 0x3B:	regs.SP--; ADDTS(6); break; /* DEC SP */

			case 0x34:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_INC(REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(11); break; /* INC (HL) */;
			case 0x04:  ALU_INC( REG_B ); ADDTS(4); break; /* INC B */
			case 0x0C:  ALU_INC( REG_C ); ADDTS(4); break; /* INC C */
			case 0x14:  ALU_INC( REG_D ); ADDTS(4); break; /* INC D */
			case 0x1C:  ALU_INC( REG_E ); ADDTS(4); break; /* INC E */
			case 0x24:  ALU_INC( REG_H ); ADDTS(4); break; /* INC H */
			case 0x2C:  ALU_INC( REG_L ); ADDTS(4); break; /* INC L */
			case 0x3C:  ALU_INC( REG_A ); ADDTS(4); break; /* INC A */
			case 0x03:	regs.BC++; ADDTS(6); break; /* INC BC */
			case 0x13:	regs.DE++; ADDTS(6); break; /* INC DE */
			case 0x23:	regs.HL++; ADDTS(6); break; /* INC HL */
			case 0x33:	regs.SP++; ADDTS(6); break; /* INC SP */

			
			
			case 0xF3:	 iff1 = 0;  iff2 = 0; ADDTS(4); break; /* DI */	
			case 0xFB:	 iff1 = 1;  iff2 = 1; ADDTS(4); break; /* EI */ 
			case 0x08:	regs.WZ = regs.AF; regs.AF = regs.AFPLUS; regs.AFPLUS = regs.WZ; ADDTS(4); break; /* EX AF, AFplus */ 
			case 0xEB:	regs.WZ = regs.DE; regs.DE = regs.HL; regs.HL = regs.WZ;  ADDTS(4); break; /* EX DE, HL */
			case 0xE3:	regs.WZ = regs.HL; RAM_READ_WORD(regs.SP,regs.HL); RAM_WRITE_WORD(regs.SP,regs.WZ);ADDTS(19); break; /* EX (SP),HL */ 
			case 0xD9: 	regs.WZ = regs.BC; regs.BC = regs.BCPLUS; regs.BCPLUS = regs.WZ;
						regs.WZ = regs.DE; regs.DE = regs.DEPLUS; regs.DEPLUS = regs.WZ;
						regs.WZ = regs.HL; regs.HL = regs.HLPLUS; regs.HLPLUS = regs.WZ;
						ADDTS(4); break; /* EXX */

			case 0x10:	if ( --REG_B == 0 ) { ADDTS(8); regs.PC++; } else { RAM_FETCH_BYTE(REG_Z); regs.PC += REG_Z; ADDTS(13);   } break; /* DJNZ d */
			
			case 0x76:	ADDTS(4); signals->HALT = 1; break; /* HALT */
			
			case 0xC2:	RAM_FETCH_WORD(regs.WZ); ADDTS(10); if ( ! F_Z ) { regs.PC = regs.WZ; } break; /* JP NZ,dd */
			case 0xCA:  RAM_FETCH_WORD(regs.WZ); ADDTS(10); if (   F_Z ) { regs.PC = regs.WZ; } break; /* JP Z,dd */
			case 0xD2:	RAM_FETCH_WORD(regs.WZ); ADDTS(10); if ( ! F_C ) { regs.PC = regs.WZ; } break; /* JP NC,dd */
			case 0xDA:	RAM_FETCH_WORD(regs.WZ); ADDTS(10); if (   F_C ) { regs.PC = regs.WZ; } break; /* JP C,dd */
			case 0xE2:	RAM_FETCH_WORD(regs.WZ); ADDTS(10); if ( ! F_P ) { regs.PC = regs.WZ; } break; /* JP PO,dd */
			case 0xEA:	RAM_FETCH_WORD(regs.WZ); ADDTS(10); if (   F_P ) { regs.PC = regs.WZ; } break; /* JP PE,dd */
			case 0xF2:	RAM_FETCH_WORD(regs.WZ); ADDTS(10); if ( ! F_S ) { regs.PC = regs.WZ; } break; /* JP P,dd */
			case 0xFA:  RAM_FETCH_WORD(regs.WZ); ADDTS(10); if (   F_S ) { regs.PC = regs.WZ; } break; /* JP M,dd */	
			case 0xC3:  RAM_FETCH_WORD(regs.WZ); ADDTS(10); regs.PC = regs.WZ; break; /* JP dd */
			case 0xE9:	RAM_READ_WORD(regs.HL,regs.PC); ADDTS(4); break; /* JP (HL) */
			
			case 0x20:	if ( ! F_Z ) { RAM_FETCH_BYTE(REG_Z); regs.PC += REG_Z; ADDTS(12); } else { ADDTS(7); regs.PC++; } ; break; /* JR NZ,d */
			case 0x28:	if (   F_Z ) { RAM_FETCH_BYTE(REG_Z); regs.PC += REG_Z; ADDTS(12); } else { ADDTS(7); regs.PC++; } ; break; /*  JR Z,d */
			case 0x30:	if ( ! F_C ) { RAM_FETCH_BYTE(REG_Z); regs.PC += REG_Z; ADDTS(12); } else { ADDTS(7); regs.PC++; } ; break; /* JR NC,d */
			case 0x38:	if (   F_C ) { RAM_FETCH_BYTE(REG_Z); regs.PC += REG_Z; ADDTS(12); } else { ADDTS(7); regs.PC++; } ; break; /*  JR C,d */
			case 0x18:	RAM_FETCH_BYTE(REG_Z); regs.PC += REG_Z; ADDTS(12);  break; /* JR d */

			case 0x3A:	RAM_FETCH_WORD(regs.WZ); RAM_READ_BYTE(regs.WZ,REG_A); ADDTS(13); break; /* LD A,(nn) */
			case 0x32:	RAM_FETCH_WORD(regs.WZ); RAM_WRITE_BYTE(regs.WZ,REG_A); ADDTS(13); break; /* LD (nn),A */
			case 0x22:	RAM_FETCH_WORD(regs.WZ); RAM_WRITE_WORD(regs.WZ,regs.HL); ADDTS(16); break; /* LD (nn),HL */
			case 0x0A:  RAM_READ_BYTE(regs.BC,REG_A); ADDTS(7); break; /* LD A,(BC) */
			case 0x1A:  RAM_READ_BYTE(regs.DE,REG_A); ADDTS(7); break; /* LD A,(DE) */
			case 0x2A:	RAM_FETCH_WORD(regs.WZ); RAM_READ_WORD(regs.WZ,regs.HL); ADDTS(16); break; /* LD HL,(nn) */

			case 0xF9:	regs.SP = regs.HL; ADDTS(6); break; /* LD SP,HL */
	
			case 0x7E:	RAM_READ_BYTE(regs.HL,REG_A); ADDTS(7);  break; /* LD A,(HL) */
			case 0x46:	RAM_READ_BYTE(regs.HL,REG_B); ADDTS(7);  break; /* LD B,(HL) */
			case 0x4E:	RAM_READ_BYTE(regs.HL,REG_C); ADDTS(7);  break; /* LD C,(HL) */
			case 0x56:	RAM_READ_BYTE(regs.HL,REG_D); ADDTS(7);  break; /* LD D,(HL) */
			case 0x5E:	RAM_READ_BYTE(regs.HL,REG_E); ADDTS(7);  break; /* LD E,(HL) */
			case 0x66:	RAM_READ_BYTE(regs.HL,REG_H); ADDTS(7);  break; /* LD H,(HL) */
			case 0x6E:	RAM_READ_BYTE(regs.HL,REG_L); ADDTS(7);  break; /* LD L,(HL) */
	
			case 0xB6:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_OR(REG_Z); ADDTS(7); break; /* OR A,(HL) */
			case 0xF6:  RAM_FETCH_BYTE(REG_W); ALU_OR( REG_W ); ADDTS(7); break; /* OR A,n */
			case 0xB0:  ALU_OR( REG_B ); ADDTS(4); break; /* OR A,B */
			case 0xB1:  ALU_OR( REG_C ); ADDTS(4); break; /* OR A,C */
			case 0xB2:  ALU_OR( REG_D ); ADDTS(4); break; /* OR A,D */
			case 0xB3:  ALU_OR( REG_E ); ADDTS(4); break; /* OR A,E */
			case 0xB4:  ALU_OR( REG_H ); ADDTS(4); break; /* OR A,H */
			case 0xB5:  ALU_OR( REG_L ); ADDTS(4); break; /* OR A,L */
			case 0xB7:  ALU_OR( REG_A ); ADDTS(4); break; /* OR A,A */

			case 0xC1:	POP_REG16(regs.BC); ADDTS(10); break; /* POP BC */
			case 0xD1:	POP_REG16(regs.DE); ADDTS(10); break; /* POP DE */
			case 0xE1:	POP_REG16(regs.HL); ADDTS(10); break; /* POP HL */
			case 0xF1:	POP_REG16(regs.AF); ADDTS(10); break; /* POP AF */
	
			case 0xC5:	PUSH_REG16(regs.BC); ADDTS(11); break; /* PUSH BC */
			case 0xD5:	PUSH_REG16(regs.DE); ADDTS(11); break; /* PUSH DE */
			case 0xE5:	PUSH_REG16(regs.HL); ADDTS(11); break; /* PUSH HL */
			case 0xF5:	PUSH_REG16(regs.AF); ADDTS(11); break; /* PUSH AF */

			case 0xC9:	POP_PC; ADDTS(10); break; /* RET */

			case 0xC0:	if ( ! F_Z ) { POP_PC; ADDTS(11); } else { ADDTS(5); } break; /* RET NZ */
			case 0xC8:  if (   F_Z ) { POP_PC; ADDTS(11); } else { ADDTS(5); } break; /* RET Z */
 			case 0xD0:	if ( ! F_C ) { POP_PC; ADDTS(11); } else { ADDTS(5); } break; /* RET NC */
			case 0xD8:	if (   F_C ) { POP_PC; ADDTS(11); } else { ADDTS(5); } break; /* RET C */
			case 0xE0:	if ( ! F_P ) { POP_PC; ADDTS(11); } else { ADDTS(5); } break; /* RET PO */
			case 0xE8:	if (   F_P ) { POP_PC; ADDTS(11); } else { ADDTS(5); } break; /* RET PE */
			case 0xF0:	if ( ! F_S ) { POP_PC; ADDTS(11); } else { ADDTS(5); } break; /* RET P */
			case 0xF8:  if (   F_S ) { POP_PC; ADDTS(11); } else { ADDTS(5); } break; /* RET M */

			case 0x17:	REG_W = F_C; CX(REG_A >> 7); NX(0); HX(0); REG_A = REG_A<<1 | REG_W; ADDTS(4); break; /* RLA */
			case 0x07:	REG_W = REG_A; CX(REG_A >> 7); NX(0); HX(0); REG_A = REG_A<<1 | (REG_W>>7); ADDTS(4); break; /* RLCA */
			case 0x1F:	REG_W = F_C; CX(REG_A&0x01); NX(0); HX(0); REG_A = REG_A>>1 | (REG_W<<7); ADDTS(4); break; /* RRA */
			case 0x0F:	REG_W = REG_A; CX(REG_A&0x01); NX(0); HX(0); REG_A = REG_A>>1 | (REG_W<<7); ADDTS(4); break; /* RRCA */
			
			case 0xC7:	PUSH_PC; regs.PC = 0x00; ADDTS(11); break; /* RST 00H */
			case 0xCF:	PUSH_PC; regs.PC = 0x08; ADDTS(11); break; /* RST 08H */
			case 0xD7:	PUSH_PC; regs.PC = 0x10; ADDTS(11); break; /* RST 10H */
			case 0xDF:	PUSH_PC; regs.PC = 0x18; ADDTS(11); break; /* RST 18H */
			case 0xE7:	PUSH_PC; regs.PC = 0x20; ADDTS(11); break; /* RST 20H */
			case 0xEF:	PUSH_PC; regs.PC = 0x28; ADDTS(11); break; /* RST 28H */
			case 0xF7:	PUSH_PC; regs.PC = 0x30; ADDTS(11); break; /* RST 30H */
			case 0xFF:	PUSH_PC; regs.PC = 0x38; ADDTS(11); break; /* RST 38H */

			case 0xDE:  RAM_FETCH_BYTE(REG_W); ALU_SBC(REG_W); ADDTS(7); break; /* SBC A,n */
			case 0x98:  ALU_SBC(REG_B); ADDTS(4); break; /* SBC A,B */
			case 0x99:  ALU_SBC(REG_C); ADDTS(4); break; /* SBC A,C */
			case 0x9A:  ALU_SBC(REG_D); ADDTS(4); break; /* SBC A,D */
			case 0x9B:  ALU_SBC(REG_E); ADDTS(4); break; /* SBC A,E */
			case 0x9C:  ALU_SBC(REG_H); ADDTS(4); break; /* SBC A,H */
			case 0x9D:  ALU_SBC(REG_L); ADDTS(4); break; /* SBC A,L */
			case 0x9F:  ALU_SBC(REG_A); ADDTS(4); break; /* SBC A,A */
			case 0x9E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SBC(REG_Z); ADDTS(7); break;; /* SBC A,(HL) */

			case 0xD6:  RAM_FETCH_BYTE(REG_W); ALU_SUB(REG_W); ADDTS(7); break; /* SUB A,n */
			case 0x90:  ALU_SUB(REG_B); ADDTS(4); break; /* SUB A,B */
			case 0x91:  ALU_SUB(REG_C); ADDTS(4); break; /* SUB A,C */
			case 0x92:  ALU_SUB(REG_D); ADDTS(4); break; /* SUB A,D */
			case 0x93:  ALU_SUB(REG_E); ADDTS(4); break; /* SUB A,E */
			case 0x94:  ALU_SUB(REG_H); ADDTS(4); break; /* SUB A,H */
			case 0x95:  ALU_SUB(REG_L); ADDTS(4); break; /* SUB A,L */
			case 0x97:  ALU_SUB(REG_A); ADDTS(4); break; /* SUB A,A */
			case 0x96:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SUB(REG_Z); ADDTS(7); break;; /* SUB A,(HL) */

			case 0xAE:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_XOR(REG_Z); ADDTS(7); break; /* XOR A,(HL) */
			case 0xEE:  RAM_FETCH_BYTE(REG_W); ALU_XOR( REG_W ); ADDTS(7); break; /* XOR A,n */
			case 0xA8:  ALU_XOR( REG_B ); ADDTS(4); break; /* XOR A,B */
			case 0xA9:  ALU_XOR( REG_C ); ADDTS(4); break; /* XOR A,C */
			case 0xAA:  ALU_XOR( REG_D ); ADDTS(4); break; /* XOR A,D */
			case 0xAB:  ALU_XOR( REG_E ); ADDTS(4); break; /* XOR A,E */
			case 0xAC:  ALU_XOR( REG_H ); ADDTS(4); break; /* XOR A,H */
			case 0xAD:  ALU_XOR( REG_L ); ADDTS(4); break; /* XOR A,L */
			case 0xAF:  ALU_XOR( REG_A ); ADDTS(4); break; /* XOR A,A */

			case 0xD3:	RAM_FETCH_BYTE(REG_W); write_io((((uint16_t)REG_A << 8) | REG_W),REG_A); ADDTS(11); break; /* OUT (N),A */
			case 0xDB:	RAM_FETCH_BYTE(REG_W); read_io((((uint16_t)REG_A << 8) | REG_W),&REG_A); ADDTS(11); break; /* IN A,(N) */


			default: 
				fprintf(stderr,"Unknown Instruction. Abort");
				exit(EXIT_FAILURE);
				break;
	}
}



//---------------------------------------------------------------------
void z80_decode_ddfd(uint16_t * hlidx) {

 char msg[512];
 		
		RAM_READ_BYTE(regs.PC, inst);
		regs.PC++;
		
		switch( inst) {
			
			case 0x21: RAM_FETCH_WORD((*hlidx)); ADDTS(14); break; /* LD IX,nn */
			case 0x86: RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_W); ALU_ADD(REG_W); ADDTS(19); break;  /* ADD A,(IX+d) */
			
			case 0x09:  ALU_ADD_16(*hlidx,regs.BC); ADDTS(15); break; /* ADD IX,BC */
			case 0x19:  ALU_ADD_16(*hlidx,regs.DE); ADDTS(15); break; /* ADD IX,DE */
			case 0x29:  ALU_ADD_16(*hlidx,*hlidx);  ADDTS(15); break; /* ADD IX,IX */
			case 0x39:  ALU_ADD_16(*hlidx,regs.SP); ADDTS(15); break; /* ADD IX,SP */		
		
			case 0xA6:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_W); ALU_AND(REG_W); ADDTS(19); break;  /* AND A,(IX+d) */
		    case 0xBE:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_W); ALU_CP(REG_W); ADDTS(19); break;  /* CP (IX + d) */
			case 0x35:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_W); ALU_DEC(REG_W); ADDTS(23);break; /* DEC (IX + d) */
			case 0x2B:	*hlidx = *hlidx - 1; ADDTS(10); break; /* DEC IX */
			case 0xE3:	regs.WZ = *hlidx; RAM_READ_WORD(regs.SP,*hlidx); RAM_WRITE_WORD(regs.SP,regs.WZ); ADDTS(23); break; /* EX (IX),HL */ 
			case 0x34:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_W); ALU_INC(REG_W); ADDTS(23);break; /* INC (IX + d) */
			case 0x23:	*hlidx = *hlidx + 1; ADDTS(10); break; /* INC IX */
			case 0xE9:	RAM_READ_WORD(*hlidx,regs.PC); ADDTS(8); break; /* JP (IX) */
		
			case 0x7E:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_A); ADDTS(19); break; /* LD A,(IX+d) */
			case 0x46:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_B); ADDTS(19); break; /* LD B,(IX+d) */
			case 0x4E:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_C); ADDTS(19); break; /* LD C,(IX+d) */
			case 0x56:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_D); ADDTS(19); break; /* LD D,(IX+d) */
			case 0x5E:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_E); ADDTS(19); break; /* LD E,(IX+d) */
			case 0x66:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_H); ADDTS(19); break; /* LD H,(IX+d) */
			case 0x6E:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_L); ADDTS(19); break; /* LD L,(IX+d) */

			case 0x36:	RAM_FETCH_BYTE(REG_Z); RAM_FETCH_BYTE(REG_W); RAM_WRITE_BYTE(*hlidx+REG_Z,REG_W); ADDTS(19); break; /* LD (IX+d),n */

			case 0x77:	RAM_FETCH_BYTE(REG_Z); RAM_WRITE_BYTE(*hlidx+REG_Z,REG_A); ADDTS(19); break; /* LD (IX+d),A */
			case 0x70:	RAM_FETCH_BYTE(REG_Z); RAM_WRITE_BYTE(*hlidx+REG_Z,REG_B); ADDTS(19); break; /* LD (IX+d),B */
			case 0x71:	RAM_FETCH_BYTE(REG_Z); RAM_WRITE_BYTE(*hlidx+REG_Z,REG_C); ADDTS(19); break; /* LD (IX+d),C */
			case 0x72:	RAM_FETCH_BYTE(REG_Z); RAM_WRITE_BYTE(*hlidx+REG_Z,REG_D); ADDTS(19); break; /* LD (IX+d),D */
			case 0x73:	RAM_FETCH_BYTE(REG_Z); RAM_WRITE_BYTE(*hlidx+REG_Z,REG_E); ADDTS(19); break; /* LD (IX+d),E */
			case 0x74:	RAM_FETCH_BYTE(REG_Z); RAM_WRITE_BYTE(*hlidx+REG_Z,REG_H); ADDTS(19); break; /* LD (IX+d),H */
			case 0x75:	RAM_FETCH_BYTE(REG_Z); RAM_WRITE_BYTE(*hlidx+REG_Z,REG_L); ADDTS(19); break; /* LD (IX+d),L */

			case 0x22:	RAM_FETCH_WORD(regs.WZ); RAM_WRITE_WORD(regs.WZ,*hlidx); ADDTS(20); break; /* LD (nn),IX */
			case 0x2A:	RAM_FETCH_WORD(regs.WZ); RAM_READ_WORD(regs.WZ,*hlidx); ADDTS(20); break; /* LD IX,(nn) */

			case 0xF9:	regs.SP = *hlidx; ADDTS(10); break; /* LD SP,IX */
			
			case 0xB6:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_W); ALU_OR(REG_W); ADDTS(19); break; /* OR A,(IX+d) */
			case 0xAE:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_W); ALU_XOR(REG_W); ADDTS(19); break; /* XOR A,(IX+d) */

			case 0xE1:	POP_REG16(*hlidx); ADDTS(14); break; /* POP IX */
			case 0xE5:	PUSH_REG16(*hlidx); ADDTS(15); break; /* PUSH IX */

			case 0x9E:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_W); ALU_SBC(REG_W); ADDTS(19); break; /* SBC A,(IX+d) */
			case 0x96:	RAM_FETCH_BYTE(REG_Z); RAM_READ_BYTE(*hlidx+REG_Z,REG_W); ALU_SUB(REG_W); regs.PC++; ADDTS(19); break; /* SUB A,(IX+d) */
		
		    case 0xCB:  uint8_t offset;
						RAM_FETCH_BYTE(offset);
					    RAM_FETCH_BYTE(inst);
						switch( inst) {
							case 0x46: 	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_BIT(REG_Z, 0); ADDTS(20); break; /* BIT 0.(IX+d) */
							case 0x4E:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_BIT(REG_Z, 1); ADDTS(20); break; /* BIT 1.(IX+d) */
							case 0x56: 	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_BIT(REG_Z, 2); ADDTS(20); break; /* BIT 2.(IX+d) */
							case 0x5E: 	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_BIT(REG_Z, 3); ADDTS(20); break; /* BIT 3.(IX+d) */
							case 0x66:  RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_BIT(REG_Z, 4); ADDTS(20); break; /* BIT 4.(IX+d) */
							case 0x6E: 	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_BIT(REG_Z, 5); ADDTS(20); break; /* BIT 5.(IX+d) */
							case 0x76: 	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_BIT(REG_Z, 6); ADDTS(20); break; /* BIT 6.(IX+d) */
							case 0x7E: 	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_BIT(REG_Z, 7); ADDTS(20); break; /* BIT 7.(IX+d) */	
							
							case 0x86:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RES(REG_Z,0); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* RES 0,(IX+d) */ 
							case 0x8E:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RES(REG_Z,1); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* RES 1,(IX+d) */
							case 0x96:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RES(REG_Z,2); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* RES 2,(IX+d) */
							case 0x9E:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RES(REG_Z,3); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* RES 3,(IX+d) */
							case 0xA6:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RES(REG_Z,4); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* RES 4,(IX+d) */ 
							case 0xAE:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RES(REG_Z,5); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* RES 5,(IX+d) */
							case 0xB6:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RES(REG_Z,6); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* RES 6,(IX+d) */
							case 0xBE:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RES(REG_Z,7); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* RES 7,(IX+d) */
				
							case 0x16:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RL(REG_Z); RAM_WRITE_BYTE(*hlidx + offset,REG_Z);  ADDTS(23); break; /* RL (IX+d) */
							case 0x06:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RLC(REG_Z); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* RLC (IX+d) */
							case 0x1E:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RR(REG_Z); RAM_WRITE_BYTE(*hlidx + offset,REG_Z);  ADDTS(23); break; /* RR (IX+d) */
							case 0x0E:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_RRC(REG_Z); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* RRC (IX+d) */
				
							case 0xC6:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SET(REG_Z,0); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SET 0,(IX+d) */ 
							case 0xCE:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SET(REG_Z,1); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SET 1,(IX+d) */
							case 0xD6:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SET(REG_Z,2); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SET 2,(IX+d) */
							case 0xDE:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SET(REG_Z,3); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SET 3,(IX+d) */
							case 0xE6:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SET(REG_Z,4); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SET 4,(IX+d) */ 
							case 0xEE:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SET(REG_Z,5); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SET 5,(IX+d) */
							case 0xF6:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SET(REG_Z,6); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SET 6,(IX+d) */
							case 0xFE:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SET(REG_Z,7); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SET 7,(IX+d) */
							
							case 0x26:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SLA(REG_Z); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SLA (IX+d) */
							case 0x2E:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SRA(REG_Z); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SRA (IX+d) */
							case 0x3E:	RAM_READ_BYTE(*hlidx + offset,REG_Z); ALU_SRL(REG_Z); RAM_WRITE_BYTE(*hlidx + offset,REG_Z); ADDTS(23); break; /* SRL (IX+d) */
							
								
							default:
								sprintf(msg,"Unknown Instruction after DD/FD CB :: %2X", inst);
								fprintf(stderr,msg);
								exit(EXIT_FAILURE);
								break;
							
						}
						break; /* CB */
			
			default:
				sprintf(msg,"Unknown Instruction after DD/FD:: %2X", inst);
				fprintf(stderr,msg);
				exit(EXIT_FAILURE);
				break;
		}
		
}

//---------------------------------------------------------------------
void z80_decode_cb() {

char msg[512];
		
		RAM_READ_BYTE(regs.PC, inst);
		regs.PC++;
		
		switch( inst) {
			
	
			case 0x07:	ALU_RLC(REG_A); ADDTS(8);  break; /* RLC A */	
			case 0x00:	ALU_RLC(REG_B); ADDTS(8);  break; /* RLC B */ 	
			case 0x01:	ALU_RLC(REG_C); ADDTS(8);  break; /* RLC C */
			case 0x02:	ALU_RLC(REG_D); ADDTS(8);  break; /* RLC D */ 	
			case 0x03:	ALU_RLC(REG_E); ADDTS(8);  break; /* RLC E */ 	
			case 0x04:	ALU_RLC(REG_H); ADDTS(8);  break; /* RLC H */ 	
			case 0x05:	ALU_RLC(REG_L); ADDTS(8);  break; /* RLC L */ 	
			case 0x06:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RLC(REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* RLC (HL) */	
			
			case 0x46:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_BIT(REG_Z,0); break; /* BIT 0,(HL) */ 
			case 0x4E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_BIT(REG_Z,1); break; /* BIT 1,(HL) */
			case 0x56:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_BIT(REG_Z,2); break; /* BIT 2,(HL) */ 
			case 0x5E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_BIT(REG_Z,3); break; /* BIT 3,(HL) */
			case 0x66:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_BIT(REG_Z,4); break; /* BIT 4,(HL) */  
			case 0x6E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_BIT(REG_Z,5); break; /* BIT 5,(HL) */ 
			case 0x76:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_BIT(REG_Z,6); break; /* BIT 6,(HL) */  
			case 0x7E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_BIT(REG_Z,7); break; /* BIT 7,(HL) */

			case 0x47:	ALU_BIT(REG_A,0); ADDTS(8); break; /* BIT 0,A */ 
			case 0x40:	ALU_BIT(REG_B,0); ADDTS(8); break; /* BIT 0,B */
			case 0x41:	ALU_BIT(REG_C,0); ADDTS(8); break; /* BIT 0,C */ 
			case 0x42:	ALU_BIT(REG_D,0); ADDTS(8); break; /* BIT 0,D */
			case 0x43:	ALU_BIT(REG_E,0); ADDTS(8); break; /* BIT 0,E */  
			case 0x44:	ALU_BIT(REG_H,0); ADDTS(8); break; /* BIT 0,H */ 
			case 0x45:	ALU_BIT(REG_L,0); ADDTS(8); break; /* BIT 0,L */  

			case 0x4F:	ALU_BIT(REG_A,1); ADDTS(8); break; /* BIT 1,A */ 
			case 0x48:	ALU_BIT(REG_B,1); ADDTS(8); break; /* BIT 1,B */
			case 0x49:	ALU_BIT(REG_C,1); ADDTS(8); break; /* BIT 1,C */ 
			case 0x4A:	ALU_BIT(REG_D,1); ADDTS(8); break; /* BIT 1,D */
			case 0x4B:	ALU_BIT(REG_E,1); ADDTS(8); break; /* BIT 1,E */  
			case 0x4C:	ALU_BIT(REG_H,1); ADDTS(8); break; /* BIT 1,H */ 
			case 0x4D:	ALU_BIT(REG_L,1); ADDTS(8); break; /* BIT 1,L */  

			case 0x57:	ALU_BIT(REG_A,2); ADDTS(8); break; /* BIT 2,A */ 
			case 0x50:	ALU_BIT(REG_B,2); ADDTS(8); break; /* BIT 2,B */
			case 0x51:	ALU_BIT(REG_C,2); ADDTS(8); break; /* BIT 2,C */ 
			case 0x52:	ALU_BIT(REG_D,2); ADDTS(8); break; /* BIT 2,D */
			case 0x53:	ALU_BIT(REG_E,2); ADDTS(8); break; /* BIT 2,E */  
			case 0x54:	ALU_BIT(REG_H,2); ADDTS(8); break; /* BIT 2,H */ 
			case 0x55:	ALU_BIT(REG_L,2); ADDTS(8); break; /* BIT 2,L */  

			case 0x5F:	ALU_BIT(REG_A,3); ADDTS(8); break; /* BIT 3,A */ 
			case 0x58:	ALU_BIT(REG_B,3); ADDTS(8); break; /* BIT 3,B */
			case 0x59:	ALU_BIT(REG_C,3); ADDTS(8); break; /* BIT 3,C */ 
			case 0x5A:	ALU_BIT(REG_D,3); ADDTS(8); break; /* BIT 3,D */
			case 0x5B:	ALU_BIT(REG_E,3); ADDTS(8); break; /* BIT 3,E */  
			case 0x5C:	ALU_BIT(REG_H,3); ADDTS(8); break; /* BIT 3,H */ 
			case 0x5D:	ALU_BIT(REG_L,3); ADDTS(8); break; /* BIT 3,L */  

			case 0x67:	ALU_BIT(REG_A,4); ADDTS(8); break; /* BIT 4,A */ 
			case 0x60:	ALU_BIT(REG_B,4); ADDTS(8); break; /* BIT 4,B */
			case 0x61:	ALU_BIT(REG_C,4); ADDTS(8); break; /* BIT 4,C */ 
			case 0x62:	ALU_BIT(REG_D,4); ADDTS(8); break; /* BIT 4,D */
			case 0x63:	ALU_BIT(REG_E,4); ADDTS(8); break; /* BIT 4,E */  
			case 0x64:	ALU_BIT(REG_H,4); ADDTS(8); break; /* BIT 4,H */ 
			case 0x65:	ALU_BIT(REG_L,4); ADDTS(8); break; /* BIT 4,L */  

			case 0x6F:	ALU_BIT(REG_A,5); ADDTS(8); break; /* BIT 5,A */ 
			case 0x68:	ALU_BIT(REG_B,5); ADDTS(8); break; /* BIT 5,B */
			case 0x69:	ALU_BIT(REG_C,5); ADDTS(8); break; /* BIT 5,C */ 
			case 0x6A:	ALU_BIT(REG_D,5); ADDTS(8); break; /* BIT 5,D */
			case 0x6B:	ALU_BIT(REG_E,5); ADDTS(8); break; /* BIT 5,E */  
			case 0x6C:	ALU_BIT(REG_H,5); ADDTS(8); break; /* BIT 5,H */ 
			case 0x6D:	ALU_BIT(REG_L,5); ADDTS(8); break; /* BIT 5,L */  

			case 0x77:	ALU_BIT(REG_A,6); ADDTS(8); break; /* BIT 6,A */ 
			case 0x70:	ALU_BIT(REG_B,6); ADDTS(8); break; /* BIT 6,B */
			case 0x71:	ALU_BIT(REG_C,6); ADDTS(8); break; /* BIT 6,C */ 
			case 0x72:	ALU_BIT(REG_D,6); ADDTS(8); break; /* BIT 6,D */
			case 0x73:	ALU_BIT(REG_E,6); ADDTS(8); break; /* BIT 6,E */  
			case 0x74:	ALU_BIT(REG_H,6); ADDTS(8); break; /* BIT 6,H */ 
			case 0x75:	ALU_BIT(REG_L,6); ADDTS(8); break; /* BIT 6,L */  

			case 0x7F:	ALU_BIT(REG_A,7); ADDTS(8); break; /* BIT 7,A */ 
			case 0x78:	ALU_BIT(REG_B,7); ADDTS(8); break; /* BIT 7,B */
			case 0x79:	ALU_BIT(REG_C,7); ADDTS(8); break; /* BIT 7,C */ 
			case 0x7A:	ALU_BIT(REG_D,7); ADDTS(8); break; /* BIT 7,D */
			case 0x7B:	ALU_BIT(REG_E,7); ADDTS(8); break; /* BIT 7,E */  
			case 0x7C:	ALU_BIT(REG_H,7); ADDTS(8); break; /* BIT 7,H */ 
			case 0x7D:	ALU_BIT(REG_L,7); ADDTS(8); break; /* BIT 7,L */  

			case 0x87:	ALU_RES(REG_A,0); ADDTS(8); break; /* RES 0,A */ 
			case 0x8F:	ALU_RES(REG_A,1); ADDTS(8); break; /* RES 1,A */ 
			case 0x97:	ALU_RES(REG_A,2); ADDTS(8); break; /* RES 2,A */ 
			case 0x9F:	ALU_RES(REG_A,3); ADDTS(8); break; /* RES 3,A */
			case 0xA7:	ALU_RES(REG_A,4); ADDTS(8); break; /* RES 4,A */ 
			case 0xAF:	ALU_RES(REG_A,5); ADDTS(8); break; /* RES 5,A */ 
			case 0xB7:	ALU_RES(REG_A,6); ADDTS(8); break; /* RES 6,A */ 
			case 0xBF:	ALU_RES(REG_A,7); ADDTS(8); break; /* RES 7,A */
		
			case 0x80:	ALU_RES(REG_B,0); ADDTS(8); break; /* RES 0,B */ 
			case 0x88:	ALU_RES(REG_B,1); ADDTS(8); break; /* RES 1,B */ 
			case 0x90:	ALU_RES(REG_B,2); ADDTS(8); break; /* RES 2,B */ 
			case 0x98:	ALU_RES(REG_B,3); ADDTS(8); break; /* RES 3,B */
			case 0xA0:	ALU_RES(REG_B,4); ADDTS(8); break; /* RES 4,B */ 
			case 0xA8:	ALU_RES(REG_B,5); ADDTS(8); break; /* RES 5,B */ 
			case 0xB0:	ALU_RES(REG_B,6); ADDTS(8); break; /* RES 6,B */ 
			case 0xB8:	ALU_RES(REG_B,7); ADDTS(8); break; /* RES 7,B */
		
			case 0x81:	ALU_RES(REG_C,0); ADDTS(8); break; /* RES 0,C */ 
			case 0x89:	ALU_RES(REG_C,1); ADDTS(8); break; /* RES 1,C */ 
			case 0x91:	ALU_RES(REG_C,2); ADDTS(8); break; /* RES 2,C */ 
			case 0x99:	ALU_RES(REG_C,3); ADDTS(8); break; /* RES 3,C */
			case 0xA1:	ALU_RES(REG_C,4); ADDTS(8); break; /* RES 4,C */ 
			case 0xA9:	ALU_RES(REG_C,5); ADDTS(8); break; /* RES 5,C */ 
			case 0xB1:	ALU_RES(REG_C,6); ADDTS(8); break; /* RES 6,C */ 
			case 0xB9:	ALU_RES(REG_C,7); ADDTS(8); break; /* RES 7,C */
		
			case 0x82:	ALU_RES(REG_D,0); ADDTS(8); break; /* RES 0,D */ 
			case 0x8A:	ALU_RES(REG_D,1); ADDTS(8); break; /* RES 1,D */ 
			case 0x92:	ALU_RES(REG_D,2); ADDTS(8); break; /* RES 2,D */
			case 0x9A:	ALU_RES(REG_D,3); ADDTS(8); break; /* RES 3,D */
			case 0xA2:	ALU_RES(REG_D,4); ADDTS(8); break; /* RES 4,D */ 
			case 0xAA:	ALU_RES(REG_D,5); ADDTS(8); break; /* RES 5,D */ 
			case 0xB2:	ALU_RES(REG_D,6); ADDTS(8); break; /* RES 6,D */ 
			case 0xBA:	ALU_RES(REG_D,7); ADDTS(8); break; /* RES 7,D */
		
			case 0x83:	ALU_RES(REG_E,0); ADDTS(8); break; /* RES 0,E */ 
			case 0x8B:	ALU_RES(REG_E,1); ADDTS(8); break; /* RES 1,E */ 
			case 0x93:	ALU_RES(REG_E,2); ADDTS(8); break; /* RES 2,E */ 
			case 0x9B:	ALU_RES(REG_E,3); ADDTS(8); break; /* RES 3,E */
			case 0xA3:	ALU_RES(REG_E,4); ADDTS(8); break; /* RES 4,E */ 
			case 0xAB:	ALU_RES(REG_E,5); ADDTS(8); break; /* RES 5,E */ 
			case 0xB3:	ALU_RES(REG_E,6); ADDTS(8); break; /* RES 6,E */ 
			case 0xBB:	ALU_RES(REG_E,7); ADDTS(8); break; /* RES 7,E */
		
			case 0x84:	ALU_RES(REG_H,0); ADDTS(8); break; /* RES 0,H */ 
			case 0x8C:	ALU_RES(REG_H,1); ADDTS(8); break; /* RES 1,H */ 
			case 0x94:	ALU_RES(REG_H,2); ADDTS(8); break; /* RES 2,H */ 
			case 0x9C:	ALU_RES(REG_H,3); ADDTS(8); break; /* RES 3,H */
			case 0xA4:	ALU_RES(REG_H,4); ADDTS(8); break; /* RES 4,H */ 
			case 0xAC:	ALU_RES(REG_H,5); ADDTS(8); break; /* RES 5,H */ 
			case 0xB4:	ALU_RES(REG_H,6); ADDTS(8); break; /* RES 6,H */ 
			case 0xBC:	ALU_RES(REG_H,7); ADDTS(8); break; /* RES 7,H */
		
			case 0x85:	ALU_RES(REG_L,0); ADDTS(8); break; /* RES 0,L */ 
			case 0x8D:	ALU_RES(REG_L,1); ADDTS(8); break; /* RES 1,L */ 
			case 0x95:	ALU_RES(REG_L,2); ADDTS(8); break; /* RES 2,L */ 
			case 0x9D:	ALU_RES(REG_L,3); ADDTS(8); break; /* RES 3,L */
			case 0xA5:	ALU_RES(REG_L,4); ADDTS(8); break; /* RES 4,L */ 
			case 0xAD:	ALU_RES(REG_L,5); ADDTS(8); break; /* RES 5,L */ 
			case 0xB5:	ALU_RES(REG_L,6); ADDTS(8); break; /* RES 6,L */ 
			case 0xBD:	ALU_RES(REG_L,7); ADDTS(8); break; /* RES 7,L */

			case 0x86:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RES(REG_Z,0); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* RES 0,(HL) */ 
			case 0x8E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RES(REG_Z,1); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* RES 1,(HL) */
			case 0x96:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RES(REG_Z,2); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* RES 2,(HL) */
			case 0x9E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RES(REG_Z,3); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* RES 3,(HL) */
			case 0xA6:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RES(REG_Z,4); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* RES 4,(HL) */ 
			case 0xAE:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RES(REG_Z,5); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* RES 5,(HL) */
			case 0xB6:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RES(REG_Z,6); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* RES 6,(HL) */
			case 0xBE:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RES(REG_Z,7); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* RES 7,(HL) */

			case 0x17:	ALU_RL(REG_A); ADDTS(8); break; /* RL A */
			case 0x10:	ALU_RL(REG_B); ADDTS(8); break; /* RL B */	
			case 0x11:	ALU_RL(REG_C); ADDTS(8); break; /* RL C */	
			case 0x12:	ALU_RL(REG_D); ADDTS(8); break; /* RL D */	
			case 0x13:	ALU_RL(REG_E); ADDTS(8); break; /* RL E */	
			case 0x14:	ALU_RL(REG_H); ADDTS(8); break; /* RL H */	
			case 0x15:	ALU_RL(REG_L); ADDTS(8); break; /* RL L */	
			case 0x16:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RL(REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break;	/* RL (HL) */

			case 0x1F:	ALU_RR(REG_A); ADDTS(8); break; /* RR A */	
			case 0x18:	ALU_RR(REG_B); ADDTS(8); break; /* RR B */	
			case 0x19:	ALU_RR(REG_C); ADDTS(8); break; /* RR C */	
			case 0x1A:	ALU_RR(REG_D); ADDTS(8); break; /* RR D */	
			case 0x1B:	ALU_RR(REG_E); ADDTS(8); break; /* RR E */
			case 0x1C:	ALU_RR(REG_H); ADDTS(8); break; /* RR H */	
			case 0x1D:	ALU_RR(REG_L); ADDTS(8); break; /* RR L */	
			case 0x1E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RR(REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break;	/* RR (HL) */

			case 0x0F:	ALU_RRC(REG_A); ADDTS(8); break; /* RRC A */
			case 0x08:	ALU_RRC(REG_B); ADDTS(8); break; /* RRC B */	
			case 0x09:	ALU_RRC(REG_C); ADDTS(8); break; /* RRC C */	
			case 0x0A:	ALU_RRC(REG_D); ADDTS(8); break; /* RRC D */	
			case 0x0B:	ALU_RRC(REG_E); ADDTS(8); break; /* RRC E */	
			case 0x0C:	ALU_RRC(REG_H); ADDTS(8); break; /* RRC H */	
			case 0x0D:	ALU_RRC(REG_L); ADDTS(8); break; /* RRC L */	
			case 0x0E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_RRC(REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* RRC (HL) */	

			case 0xC7:	ALU_SET(REG_A,0); ADDTS(8); break; /* SET 0,A */ 
			case 0xCF:	ALU_SET(REG_A,1); ADDTS(8); break; /* SET 1,A */ 
			case 0xD7:	ALU_SET(REG_A,2); ADDTS(8); break; /* SET 2,A */ 
			case 0xDF:	ALU_SET(REG_A,3); ADDTS(8); break; /* SET 3,A */
			case 0xE7:	ALU_SET(REG_A,4); ADDTS(8); break; /* SET 4,A */ 
			case 0xEF:	ALU_SET(REG_A,5); ADDTS(8); break; /* SET 5,A */ 
			case 0xF7:	ALU_SET(REG_A,6); ADDTS(8); break; /* SET 6,A */ 
			case 0xFF:	ALU_SET(REG_A,7); ADDTS(8); break; /* SET 7,A */
		
			case 0xC0:	ALU_SET(REG_B,0); ADDTS(8); break; /* SET 0,B */ 
			case 0xC8:	ALU_SET(REG_B,1); ADDTS(8); break; /* SET 1,B */ 
			case 0xD0:	ALU_SET(REG_B,2); ADDTS(8); break; /* SET 2,B */ 
			case 0xD8:	ALU_SET(REG_B,3); ADDTS(8); break; /* SET 3,B */
			case 0xE0:	ALU_SET(REG_B,4); ADDTS(8); break; /* SET 4,B */ 
			case 0xE8:	ALU_SET(REG_B,5); ADDTS(8); break; /* SET 5,B */ 
			case 0xF0:	ALU_SET(REG_B,6); ADDTS(8); break; /* SET 6,B */ 
			case 0xF8:	ALU_SET(REG_B,7); ADDTS(8); break; /* SET 7,B */
	
			case 0xC1:	ALU_SET(REG_C,0); ADDTS(8); break; /* SET 0,C */
			case 0xC9:	ALU_SET(REG_C,1); ADDTS(8); break; /* SET 1,C */ 
			case 0xD1:	ALU_SET(REG_C,2); ADDTS(8); break; /* SET 2,C */
			case 0xD9:	ALU_SET(REG_C,3); ADDTS(8); break; /* SET 3,C */
			case 0xE1:	ALU_SET(REG_C,4); ADDTS(8); break; /* SET 4,C */ 
			case 0xE9:	ALU_SET(REG_C,5); ADDTS(8); break; /* SET 5,C */ 
			case 0xF1:	ALU_SET(REG_C,6); ADDTS(8); break; /* SET 6,C */ 
			case 0xF9:	ALU_SET(REG_C,7); ADDTS(8); break; /* SET 7,C */
		
			case 0xC2:	ALU_SET(REG_D,0); ADDTS(8); break; /* SET 0,D */
			case 0xCA:	ALU_SET(REG_D,1); ADDTS(8); break; /* SET 1,D */ 
			case 0xD2:	ALU_SET(REG_D,2); ADDTS(8); break; /* SET 2,D */ 
			case 0xDA:	ALU_SET(REG_D,3); ADDTS(8); break; /* SET 3,D */
			case 0xE2:	ALU_SET(REG_D,4); ADDTS(8); break; /* SET 4,D */ 
			case 0xEA:	ALU_SET(REG_D,5); ADDTS(8); break; /* SET 5,D */ 
			case 0xF2:	ALU_SET(REG_D,6); ADDTS(8); break; /* SET 6,D */ 
			case 0xFA:	ALU_SET(REG_D,7); ADDTS(8); break; /* SET 7,D */
	
			case 0xC3:	ALU_SET(REG_E,0); ADDTS(8); break; /* SET 0,E */
			case 0xCB:	ALU_SET(REG_E,1); ADDTS(8); break; /* SET 1,E */ 
			case 0xD3:	ALU_SET(REG_E,2); ADDTS(8); break; /* SET 2,E */ 
			case 0xDB:	ALU_SET(REG_E,3); ADDTS(8); break; /* SET 3,E */
			case 0xE3:	ALU_SET(REG_E,4); ADDTS(8); break; /* SET 4,E */ 
			case 0xEB:	ALU_SET(REG_E,5); ADDTS(8); break; /* SET 5,E */ 
			case 0xF3:	ALU_SET(REG_E,6); ADDTS(8); break; /* SET 6,E */ 
			case 0xFB:	ALU_SET(REG_E,7); ADDTS(8); break; /* SET 7,E */
		
			case 0xC4:	ALU_SET(REG_H,0); ADDTS(8); break; /* SET 0,H */
			case 0xCC:	ALU_SET(REG_H,1); ADDTS(8); break; /* SET 1,H */ 
			case 0xD4:	ALU_SET(REG_H,2); ADDTS(8); break; /* SET 2,H */ 
			case 0xDC:	ALU_SET(REG_H,3); ADDTS(8); break; /* SET 3,H */
			case 0xE4:	ALU_SET(REG_H,4); ADDTS(8); break; /* SET 4,H */ 
			case 0xEC:	ALU_SET(REG_H,5); ADDTS(8); break; /* SET 5,H */ 
			case 0xF4:	ALU_SET(REG_H,6); ADDTS(8); break; /* SET 6,H */ 
			case 0xFC:	ALU_SET(REG_H,7); ADDTS(8); break; /* SET 7,H */
	
			case 0xC5:	ALU_SET(REG_L,0); ADDTS(8); break; /* SET 0,L */ 
			case 0xCD:	ALU_SET(REG_L,1); ADDTS(8); break; /* SET 1,L */ 
			case 0xD5:	ALU_SET(REG_L,2); ADDTS(8); break; /* SET 2,L */ 
			case 0xDD:	ALU_SET(REG_L,3); ADDTS(8); break; /* SET 3,L */
			case 0xE5:	ALU_SET(REG_L,4); ADDTS(8); break; /* SET 4,L */ 
			case 0xED:	ALU_SET(REG_L,5); ADDTS(8); break; /* SET 5,L */ 
			case 0xF5:	ALU_SET(REG_L,6); ADDTS(8); break; /* SET 6,L */ 
			case 0xFD:	ALU_SET(REG_L,7); ADDTS(8); break; /* SET 7,L */

			case 0xC6:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SET(REG_Z,0); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SET 0,(HL) */ 
			case 0xCE:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SET(REG_Z,1); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SET 1,(HL) */
			case 0xD6:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SET(REG_Z,2); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SET 2,(HL) */
			case 0xDE:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SET(REG_Z,3); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SET 3,(HL) */
			case 0xE6:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SET(REG_Z,4); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SET 4,(HL) */ 
			case 0xEE:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SET(REG_Z,5); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SET 5,(HL) */
			case 0xF6:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SET(REG_Z,6); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SET 6,(HL) */
			case 0xFE:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SET(REG_Z,7); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SET 7,(HL) */

			case 0x27:	ALU_SLA(REG_A); ADDTS(8);  break; /* SLA A */	
			case 0x20:	ALU_SLA(REG_B); ADDTS(8);  break; /* SLA B */ 	
			case 0x21:	ALU_SLA(REG_C); ADDTS(8);  break; /* SLA C */
			case 0x22:	ALU_SLA(REG_D); ADDTS(8);  break; /* SLA D */ 	
			case 0x23:	ALU_SLA(REG_E); ADDTS(8);  break; /* SLA E */ 	
			case 0x24:	ALU_SLA(REG_H); ADDTS(8);  break; /* SLA H */ 	
			case 0x25:	ALU_SLA(REG_L); ADDTS(8);  break; /* SLA L */ 	
			case 0x26:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SLA(REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SLA (HL) */	

			case 0x2F:	ALU_SRA(REG_A); ADDTS(8);  break; /* SRA A */	
			case 0x28:	ALU_SRA(REG_B); ADDTS(8);  break; /* SRA B */ 	
			case 0x29:	ALU_SRA(REG_C); ADDTS(8);  break; /* SRA C */
			case 0x2A:	ALU_SRA(REG_D); ADDTS(8);  break; /* SRA D */ 	
			case 0x2B:	ALU_SRA(REG_E); ADDTS(8);  break; /* SRA E */ 	
			case 0x2C:	ALU_SRA(REG_H); ADDTS(8);  break; /* SRA H */ 	
			case 0x2D:	ALU_SRA(REG_L); ADDTS(8);  break; /* SRA L */ 	
			case 0x2E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SRA(REG_Z);  RAM_WRITE_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SRA (HL) */	

			case 0x3F:	ALU_SRL(REG_A); ADDTS(8);  break; /* SRL A */	
			case 0x38:	ALU_SRL(REG_B); ADDTS(8);  break; /* SRL B */ 	
			case 0x39:	ALU_SRL(REG_C); ADDTS(8);  break; /* SRL C */
			case 0x3A:	ALU_SRL(REG_D); ADDTS(8);  break; /* SRL D */ 	
			case 0x3B:	ALU_SRL(REG_E); ADDTS(8);  break; /* SRL E */ 	
			case 0x3C:	ALU_SRL(REG_H); ADDTS(8);  break; /* SRL H */ 	
			case 0x3D:	ALU_SRL(REG_L); ADDTS(8);  break; /* SRL L */ 	
			case 0x3E:	RAM_READ_BYTE(regs.HL,REG_Z); ALU_SRL(REG_Z); RAM_READ_BYTE(regs.HL,REG_Z); ADDTS(15); break; /* SRL (HL) */	


			default: 
				sprintf(msg,"Unknown Instruction after CB:: %2X", inst);
				fprintf(stderr,msg);
				exit(EXIT_FAILURE);
				break;
		}
		
}

//---------------------------------------------------------------------
void z80_decode_ed() {

char msg[512];
		
		RAM_READ_BYTE(regs.PC, inst);
		regs.PC++;
		
		switch( inst) {
			
			case 0x4A:  ALU_ADC_HL(regs.BC); ADDTS(15); break; /* ADC HL,BC */
			case 0x5A:  ALU_ADC_HL(regs.DE); ADDTS(15); break; /* ADC HL,DE */
			case 0x6A:  ALU_ADC_HL(regs.HL); ADDTS(15); break; /* ADC HL,HL */
			case 0x7A:  ALU_ADC_HL(regs.SP); ADDTS(15); break; /* ADC HL,SP */
			
			case 0x42:  ALU_SBC_HL(regs.BC); ADDTS(15); break; /* SBC HL,BC */
			case 0x52:  ALU_SBC_HL(regs.DE); ADDTS(15); break; /* SBC HL,DE */
			case 0x62:  ALU_SBC_HL(regs.HL); ADDTS(15); break; /* SBC HL,HL */
			case 0x72:  ALU_SBC_HL(regs.SP); ADDTS(15); break; /* SBC HL,SP */

			
			case 0xA9:  RAM_READ_BYTE(regs.HL,REG_Z); NX(1); HX((REG_A & 0x0F) < (REG_Z & 0x0F)); SX((REG_Z-REG_A)>>7);ZX(REG_Z==REG_A);regs.BC--; regs.HL--;PX(regs.BC==0); ADDTS(16); break; /* CPD */					
			case 0xB9:  RAM_READ_BYTE(regs.HL,REG_Z); NX(1); HX((REG_A & 0x0F) < (REG_Z & 0x0F)); ZX(REG_Z==REG_A); regs.BC--; regs.HL--; PX(regs.BC == 0); if ( regs.BC != 0 && REG_Z != REG_A ) { regs.PC -= 2; ADDTS(21); }	else { ADDTS(16); } break; /* CPDR */
			case 0xA1:  RAM_READ_BYTE(regs.HL,REG_Z); NX(1); HX((REG_A & 0x0F) < (REG_Z & 0x0F)); SX((REG_Z-REG_A)>>7); ZX(REG_Z==REG_A); regs.BC--; regs.HL++;PX(regs.BC==0); ADDTS(16); break; /* CPI */					
			case 0xB1:  RAM_READ_BYTE(regs.HL,REG_Z); NX(1); HX((REG_A&0x0F) < (REG_Z & 0x0F)); ZX(REG_Z==REG_A);regs.BC--; regs.HL++; PX(regs.BC==0); if ( regs.BC!=0 && REG_Z!=REG_A ) { regs.PC -= 2; ADDTS(21); }	else { ADDTS(16); } break; /* CPIR */
				
			case 0x46:	 imode = 0; ADDTS(8); break; /* IM 0 */
			case 0x56:	 imode = 1; ADDTS(8); break; /* IM 1 */
			case 0x5E:	 imode = 2; ADDTS(8); break; /* IM 2 */
	
			case 0x4B:	RAM_FETCH_WORD(regs.WZ); RAM_READ_WORD(regs.WZ,regs.BC); ADDTS(20); break; /* LD BC,(nn) */ 
			case 0x5B:	RAM_FETCH_WORD(regs.WZ); RAM_READ_WORD(regs.WZ,regs.DE); ADDTS(20); break; /* LD DE,(nn) */ 
			case 0x6B:	RAM_FETCH_WORD(regs.WZ); RAM_READ_WORD(regs.WZ,regs.HL); ADDTS(20); break; /* LD HL,(nn) */ 
			case 0x7B:	RAM_FETCH_WORD(regs.WZ); RAM_READ_WORD(regs.WZ,regs.SP); ADDTS(20); break; /* LD SP,(nn) */ 

			case 0x43:	RAM_FETCH_WORD(regs.WZ); RAM_WRITE_WORD(regs.WZ,regs.BC); ADDTS(20); break; /* LD (nn),BC */ 
			case 0x53:	RAM_FETCH_WORD(regs.WZ); RAM_WRITE_WORD(regs.WZ,regs.DE); ADDTS(20); break; /* LD (nn),DE */
			case 0x63:	RAM_FETCH_WORD(regs.WZ); RAM_WRITE_WORD(regs.WZ,regs.HL); ADDTS(20); break; /* LD (nn),HL */
			case 0x73:	RAM_FETCH_WORD(regs.WZ); RAM_WRITE_WORD(regs.WZ,regs.SP); ADDTS(20); break; /* LD (nn),SP */ 
		
			case 0x57:	REG_A = REG_I; NX(0); HX(0); SX(REG_A >> 7); ZX(REG_A == 0); PX( iff2); ADDTS(9); break;  /* LD A,I */  
			case 0x47:	REG_I = REG_A; ADDTS(9); break;  /* LD I,A */  
			case 0x5F:	REG_A = REG_R; NX(0); HX(0); SX(REG_A >> 7); ZX(REG_A == 0); PX( iff2); ADDTS(9); break;  /* LD A,R */  
			case 0x4F:	REG_R = REG_A; ADDTS(9); break;  /* LD R,A */  
			
			case 0xA8:	RAM_READ_BYTE(regs.DE--,REG_Z); RAM_READ_BYTE(regs.HL--, REG_Z); regs.BC--; HX(0) ; NX(0);	PX(regs.BC != 0); ADDTS(16); break; /* LDD */
			case 0xB8:	RAM_READ_BYTE(regs.DE--,REG_Z); HX(0);PX(0);NX(0); RAM_READ_BYTE(regs.HL--, REG_Z); if (--regs.BC == 0 ) { ADDTS(16) } else { ADDTS(21) ; regs.PC -= 2; }	break; /* LDDR */
			case 0xA0:	RAM_READ_BYTE(regs.DE++,REG_Z); RAM_READ_BYTE(regs.HL++, REG_Z); regs.BC--; HX(0) ; NX(0);	PX(regs.BC != 0); ADDTS(16); break; /* LDI */
			case 0xB0:	RAM_READ_BYTE(regs.DE++,REG_Z); HX(0);PX(0);NX(0); RAM_READ_BYTE(regs.HL++, REG_Z); if (--regs.BC == 0 ) { ADDTS(16) } else { ADDTS(21) ; regs.PC -= 2; }	break; /* LDIR */
			
			case 0x44:	NX(1); CX(REG_A==0); PX(REG_A==0x80); REG_A = 0 - REG_A; HX((REG_A&0x0F) > 0); ZX(REG_A==0); SX(REG_A>>7); YX(REG_A>>5); XX(REG_A>>3);ADDTS(8); break; /* NEG */

			case 0x4D:	POP_PC; ADDTS(14); break; /* RETI */
			case 0x45:	POP_PC;  iff1 =  iff2; ADDTS(14); break; /* RETN */
					
			case 0x6F:	RAM_READ_BYTE(regs.HL, REG_W); RAM_WRITE_BYTE(regs.HL, ((REG_W<<4) | (REG_A&0x0F)));      REG_A = ((REG_A & 0xF0) | (REG_W>>4));   NX(0); HX(0); ZX(REG_A==0); SX(REG_A>>7); PX(z80_alu_check_parity(REG_A));	ADDTS(18); break; /* RLD */					
			case 0x67:	RAM_READ_BYTE(regs.HL, REG_W); RAM_WRITE_BYTE(regs.HL, ((REG_W>>4) | ((REG_A&0x0F)<<4))); REG_A = ((REG_A & 0xF0) | (REG_W&0x0F)); NX(0); HX(0); ZX(REG_A==0); SX(REG_A>>7); PX(z80_alu_check_parity(REG_A)); ADDTS(18); break; /* RRD */	
		
			case 0xBB:	REG_B--; RAM_READ_BYTE(regs.HL,REG_Z); write_io(regs.BC,REG_Z); regs.HL--; NX(1); if (REG_B == 0) { ADDTS(16); ZX(1);} else { ADDTS(21); regs.PC -= 2; } break; /* OTDR */
			case 0xB3:	REG_B--; RAM_READ_BYTE(regs.HL,REG_Z); write_io(regs.BC,REG_Z); regs.HL++; NX(1); if (REG_B == 0) { ADDTS(16); ZX(1);} else { ADDTS(21); regs.PC -= 2; } break; /* OTIR */

			case 0xBA:	REG_B--; read_io(regs.BC,&REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); regs.HL--; NX(1); if (REG_B == 0) { ADDTS(16); ZX(1);} else { ADDTS(21); regs.PC -= 2; } break; /* INDR */
			case 0xB2:	REG_B--; read_io(regs.BC,&REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); regs.HL++; NX(1); if (REG_B == 0) { ADDTS(16); ZX(1);} else { ADDTS(21); regs.PC -= 2; } break; /* INIR */
			 			
			case 0x79:	write_io(regs.BC,REG_A); ADDTS(12); break; /* OUT (C),A */
			case 0x41:	write_io(regs.BC,REG_B); ADDTS(12); break; /* OUT (C),B */
			case 0x49:	write_io(regs.BC,REG_C); ADDTS(12); break; /* OUT (C),C */
			case 0x51:	write_io(regs.BC,REG_D); ADDTS(12); break; /* OUT (C),D */
			case 0x59:	write_io(regs.BC,REG_E); ADDTS(12); break; /* OUT (C),E */
			case 0x61:	write_io(regs.BC,REG_H); ADDTS(12); break; /* OUT (C),H */
			case 0x69:	write_io(regs.BC,REG_L); ADDTS(12); break; /* OUT (C),L */

			case 0xAB:	REG_B--; RAM_READ_BYTE(regs.HL,REG_Z); write_io(regs.BC,REG_Z); regs.HL--; ZX(REG_B==0); NX(1); ADDTS(16); break; /* OUTD */
			case 0xA3:	REG_B--; RAM_READ_BYTE(regs.HL,REG_Z); write_io(regs.BC,REG_Z); regs.HL++; ZX(REG_B==0); NX(1); ADDTS(16); break; /* OUTI */
			
			case 0xAA:	REG_B--; read_io(regs.BC,&REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); regs.HL--; ZX(REG_B==0); NX(1); ADDTS(16); break; /* IND */
			case 0xA2:	REG_B--; read_io(regs.BC,&REG_Z); RAM_WRITE_BYTE(regs.HL,REG_Z); regs.HL++; ZX(REG_B==0); NX(1); ADDTS(16); break; /* INI */
			
					/* The HFLAG is missing in IN , how to calculate it? No idea */
			case 0x78:	read_io(regs.BC,&REG_A); NX(0); SX(REG_A>>7); ZX(REG_A==0); PX(z80_alu_check_parity(REG_A)); ADDTS(12); break; /* IN A,(C) */
			case 0x40:	read_io(regs.BC,&REG_B); NX(0); SX(REG_B>>7); ZX(REG_B==0); PX(z80_alu_check_parity(REG_B)); ADDTS(12); break; /* IN B,(C) */
			case 0x48:	read_io(regs.BC,&REG_C); NX(0); SX(REG_C>>7); ZX(REG_C==0); PX(z80_alu_check_parity(REG_C)); ADDTS(12); break; /* IN C,(C) */
			case 0x50:	read_io(regs.BC,&REG_D); NX(0); SX(REG_D>>7); ZX(REG_D==0); PX(z80_alu_check_parity(REG_D)); ADDTS(12); break; /* IN D,(C) */
			case 0x58:	read_io(regs.BC,&REG_E); NX(0); SX(REG_E>>7); ZX(REG_E==0); PX(z80_alu_check_parity(REG_E)); ADDTS(12); break; /* IN E,(C) */
			case 0x60:	read_io(regs.BC,&REG_H); NX(0); SX(REG_H>>7); ZX(REG_H==0); PX(z80_alu_check_parity(REG_H)); ADDTS(12); break; /* IN H,(C) */
			case 0x68:	read_io(regs.BC,&REG_L); NX(0); SX(REG_L>>7); ZX(REG_L==0); PX(z80_alu_check_parity(REG_L)); ADDTS(12); break; /* IN L,(C) */
			
										
			default: 
				sprintf(msg,"Unknown Instruction after ED:: %2X", inst);
				fprintf(stderr,msg);
				exit(EXIT_FAILURE);
				break;
		}
		
}



//----------------------------------------------------------------------
int z80_alu_check_parity( uint8_t input ) {
	
  input =  input ^ ( input >>  4 );
  input =  input ^ ( input >>  2 );  
  return ( input ^ ( input >>  1 ) ) & 0x1;
}

//----------------------------------------------------------------------
void z80_alu_daa() {
	
	switch(F_N) {
		     case 0: switch(F_C) {
			      case 0: switch(F_H) {
					       case 0: 
									if (((REG_A & 0x0F ) <= 9 ) && (((REG_A & 0xF0 ) >> 4) <= 9) ) 		{ CX(0);  }
									if (((REG_A & 0x0F ) >= 0x0A ) && (((REG_A & 0xF0 ) >> 4) <= 8) ) 	{ CX(0); REG_A += 6; }
									if (((REG_A & 0x0F ) <= 9 ) && (((REG_A & 0xF0 ) >> 4) >= 0x0A) ) 	{ CX(1); REG_A += 0x60; }      
									if (((REG_A & 0x0F ) > 0x0A ) && (((REG_A & 0xF0 ) >> 4) >= 9) ) 	{ CX(1); REG_A += 0x66; }      
					               break;
					       case 1: 
									if (((REG_A & 0x0F ) <= 3 ) && (((REG_A & 0xF0 ) >> 4) <= 9) ) 		{ CX(0); REG_A += 6; }
									if (((REG_A & 0x0F ) <= 3 ) && (((REG_A & 0xF0 ) >> 4) >= 0x0A) ) 	{ CX(1); REG_A += 0x66; }      
					               break;
					      }
			             break;
			      case 1: switch(F_H) {
					       case 0: 
									if (((REG_A & 0x0F ) <= 9 ) && (((REG_A & 0xF0 ) >> 4) <= 2) ) 		{ CX(1); REG_A += 0x60; }
									if (((REG_A & 0x0F ) >= 0x0A ) && (((REG_A & 0xF0 ) >> 4) <= 2) ) 	{ CX(1); REG_A += 0x66; }
					               break;
					       case 1: 
									if (((REG_A & 0x0F ) <= 3 ) && (((REG_A & 0xF0 ) >> 4) <= 3) ) 		{ CX(1); REG_A += 0x66; }
					               break;
					      }
			             break;
		        }
		       break;
		case 1: switch(F_C) {
			      case 0: switch(F_H) {
					       case 0: 
									if (((REG_A & 0x0F ) <= 9 ) && (((REG_A & 0xF0 ) >> 4) <= 9) ) 		{ CX(0); }
								   break;
					       case 1:  
									if (((REG_A & 0x0F ) >= 6 ) && (((REG_A & 0xF0 ) >> 4) <= 8) ) 		{ CX(0); REG_A += 0xFA;}
					               break;
					      }
			             break;
			      case 1: switch(F_H) {
					       case 0: 
									if (((REG_A & 0x0F ) <= 9 ) && (((REG_A & 0xF0 ) >> 4) >= 7) ) 		{ CX(1); REG_A += 0xA0; }
					               break;
					       case 1: 
									if (((REG_A & 0x0F ) >= 6 ) && (((REG_A & 0xF0 ) >> 4) >= 6) ) 		{ CX(1); REG_A += 0x9A; }
					               break;
					      }
			             break;
		        }
		       break;
	} 
	       
	ZX(REG_A == 0);
	SX(REG_A  >> 7); 
	PX(z80_alu_check_parity( REG_A));
	/* H Flag??? */       
}
