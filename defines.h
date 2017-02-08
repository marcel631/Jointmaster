/**
 *  Defines for your entire project at one place
 * 
 *	@author 	Tilen Majerle
 *	@email		tilen@majerle.eu
 *	@website	http://stm32f4-discovery.com
 *	@version 	v1.0
 *	@ide		Keil uVision 5
 *	@license	GNU GPL v3
 *	
 * |----------------------------------------------------------------------
 * | Copyright (C) Tilen Majerle, 2014
 * | 
 * | This program is free software: you can redistribute it and/or modify
 * | it under the terms of the GNU General Public License as published by
 * | the Free Software Foundation, either version 3 of the License, or
 * | any later version.
 * |  
 * | This program is distributed in the hope that it will be useful,
 * | but WITHOUT ANY WARRANTY; without even the implied warranty of
 * | MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * | GNU General Public License for more details.
 * | 
 * | You should have received a copy of the GNU General Public License
 * | along with this program.  If not, see <http://www.gnu.org/licenses/>.
 * |----------------------------------------------------------------------
 */
#ifndef TM_DEFINES_H
#define TM_DEFINES_H

/* Put your global defines for all libraries here used in your project */
#define TM_DISCO_STM32F4_DISCOVERY  // for use with f407VG board
//#define TM_DISCO_STM32F411_DISCOVERY // for use with f411 board

#define USE_STOP_SWITCHES
//#undef USE_STOP_SWITCHES
//#define USE_USB
#undef USE_USB

//#undef DEBUG_OUTPUT
#define DEBUG_OUTPUT
#undef DEBUG_SR

//---------------------------********IMPORTANT NOTE**********************--------------------------------------
//om te compileren voor F411 of F407, in stm32f4xx.h cmsis de define STM32F407xx of STM32F411xx aan of uitzetten.
//---------------------------******************************--------------------------------------

#ifdef USE_USB
   #define STM32F40_41xxx
#endif

//note, to work with normally closed stopswitches, compile without '!', otherwise ''. Thus normally closed "#define NCO(x) (x)" and normally open = "#define NCO(x) !(x)"
#define NCO(x) !(x) //normally open
//#define NCO(x) (x) //normally closed




#define CpuCriticalVar()  uint8_t cpuSR

#define CpuEnterCritical()              \
  do {                                  \
    asm (                               \
    "MRS   R0, PRIMASK\n\t"             \
    "CPSID I\n\t"                       \
    "STRB R0, %[output]"                \
    : [output] "=m" (cpuSR) :: "r0");   \
  } while(0)

#define CpuExitCritical()               \
  do{                                   \
    asm (                               \
    "ldrb r0, %[input]\n\t"             \
    "msr PRIMASK,r0;\n\t"               \
    ::[input] "m" (cpuSR) : "r0");      \
  } while(0)


#endif



