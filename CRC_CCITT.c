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

#include "CRC_CCITT.h"
#include "globals.h"
#include "defines.h"

//---------------------------------------------------------------------------
//---------------------------------------------------------------------------

void CalculateTable_CRC16()
{
    const uint16 generator = 0x1021;

    for (int divident = 0; divident < 256; divident++) /* iterate over all possible input byte values 0 - 255 */
    {
        uint16 curByte = (uint16) (divident << 8); /* move divident byte into MSB of 16Bit CRC */

        for (short bit = 0; bit < 8; bit++)
        {
            if ((curByte & 0x8000) != 0)
            {
                curByte <<= 1;
                curByte ^= generator;
            }
            else
            {
                curByte <<= 1;
            }
        }

        crctable16[divident] = curByte;
    }
}

unsigned char CRC_CCITT(unsigned char* pMsg,  int MsgLength)
{

    unsigned char Index;

    uint16 crc = 0xffff;

    for (int i = 0; i < MsgLength; i++)
    {
        /* XOR-in next input byte into MSB of crc, that's our new intermediate divident */
        /* Shift out the MSB used for division per lookuptable and XOR with the remainder */
    	Index = crc >> 8 ^ *pMsg++;
        crc = (uint16)((crc << 8) ^ (crctable16[Index]));
    }
    //*pMsg =crc >> 8;
   // pMsg++;
   // *pMsg =crc << 8;
    if(crc ==0)
          return 0;
     else
          return -1;

}

