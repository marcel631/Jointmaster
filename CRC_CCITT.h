/**	
 * |----------------------------------------------------------------------
 * | Copyright (C) SMI Holding BV, www.smidev.nl, 2017
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
//---------------------------------------------------------------------------
#ifndef CRC_CCITTH
#define CRC_CCITTH
//---------------------------------------------------------------------------
#include "globals.h"
uint16 crctable16[256];
unsigned char CRC_CCITT(unsigned char*,  int);
void CalculateTable_CRC16();

#endif
