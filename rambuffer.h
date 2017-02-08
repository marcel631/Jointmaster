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

#ifndef ram_buf_H
#define ram_buf_H

#include "globals.h"

#define ram __attribute__ ((section (".data")))

extern char Command_buffer[max_command+1][STLN] ram;


#endif /* ram_buf_h*/
