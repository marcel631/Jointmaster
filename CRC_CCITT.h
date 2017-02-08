//---------------------------------------------------------------------------
#ifndef CRC_CCITTH
#define CRC_CCITTH
//---------------------------------------------------------------------------
#include "globals.h"
uint16 crctable16[256];
unsigned char CRC_CCITT(unsigned char*,  int);
void CalculateTable_CRC16();

#endif
