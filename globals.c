/*
 * globals.c
 *
 *  Created on: May 31, 2015
 *      Author: SMI
 */

#include "globals.h"
#include "rambuffer.h"
#include "CRC_CCITT.h"
#include "stdlib.h"
#include <stdio.h>
#include <string.h>
#include "stm32F4xx_gpio.h"
#include "tm_stm32f4_disco.h"
#include <stm32f4xx_tim.h>
#include <stm32f4xx_rcc.h>
#include "tm_stm32f4_delay.h"
#if defined (USE_USB)
#include "tm_stm32f4_usb_vcp.h"
#else
#include "tm_stm32f4_usart.h"
//#include "tm_stm32f4_usart_dma.h"
#endif
#if defined (STM32F411xE)
#include "stm32f411xe.h"
#endif
#if defined (STM32F407xx)
#include "stm32f407xx.h"
#endif

#include "diag/Trace.h"
#include "fixmath.h"
#include <math.h>

unsigned int cyc[2];// note clock only accurate to 25.56 sec.
double x;

// Joseph Yiu's method
// From http://forums.arm.com/index.php?showtopic=13949

volatile unsigned int *DWT_CYCCNT     = (volatile unsigned int *)0xE0001004; //address of the register
volatile unsigned int *DWT_CONTROL    = (volatile unsigned int *)0xE0001000; //address of the register
volatile unsigned int *SCB_DEMCR         = (volatile unsigned int *)0xE000EDFC; //address of the register



#define STOPWATCH_START { cyc[0] = *DWT_CYCCNT;}
#define STOPWATCH_STOP { cyc[1] = *DWT_CYCCNT; cyc[1] = cyc[1] - cyc[0]; } // note clock only accurate to 25.56 sec.



//char rdBuf[APP_READ_BUFFER_SIZE];
char dummystr2[APP_READ_BUFFER_SIZE];
char dummystr[APP_READ_BUFFER_SIZE];
char dummystr3[APP_READ_BUFFER_SIZE];
//char wrBuf[APP_WRITE_BUFFER_SIZE];
char Command_data[STLN] = "1234567890";
char Saved_data[STLN] = "1234567890";
unsigned char unsigned_data[STLN];


double alpha, omega_acc, omega_max, omega_decc, frequency;
// Create Address Error Trap
//struct tagINTTREGBITS my_INTTREG;
unsigned int interrupt_error = 0;
int32 step_no = 0;
short slip_cause = 0;
int32 enable_st[max_stepper+1];// number of steps to decelerate upon stopswitch, note c starts array index at zero, we count in this program from 1. Thus reserve one more
int32 total_steps=0;
int32 paused_steps=0;
int32 total_steps1=10000;// only used for latency dummy calc.
int32 n_real_max1=7000;// only used for latency dummy calc.
int32 step_no1 = 5000;// only used for latency dummy calc.
int32 Q16_one = 1<<16;// one in Q16.15 format.
unsigned int enable = 0;
unsigned int status = 0;
short sign_total_steps[max_stepper+1];
short move_it[max_stepper+1];//only used for 3D movements.
short sign_encoder_ratio = 0;
short homing = 0;
short use3D=0; // when moving 3D, this is 1, else 0
int32 no_commands=0;
short prev_action[max_stepper+1]; // used for investigate_encoder in case of undefined status change
short start_phase = 0;
short paused = 0; //if paused is 1 then processing immediately stops but does not break program. processing continues when paused=0
short neglect = 0; //stopswitch to neglect when homing is 1
short use1 = 0; //when homing, use stopswitch 1 signal
short use2 = 0; //when homing use stopswitch 2 signal
unsigned int direction[max_stepper+1]; //CW or CCW
int32 total_steps_array[max_stepper+1];
int32 total_steps_d2_array[max_stepper+1];
int32 err_array[max_stepper+1];
int32 homing_steps = 0;

short stopsw=0;
short stopswh=0;
short Stop_after_decel_stopsw = 0;
short stopswitched=0;
short stepper=1;
short paused_stepper=1;
short c_stepper=1;
int32 n_decc, n_acc, n_acc_decc, n_real_min, n_real_max;
_Q16 cn, cn_1, Qn_real_min, Q_same, step1, step2, step3;
int prescaler;
int succes, config_read[max_stepper+1];
int32 mul;
int32 stepper_steps[max_stepper+1];
double  n_deccd, n_accd,dummyfloat;
double rpm[max_stepper+1];
double rpm_max[max_stepper+1];
double rpm_feed[max_stepper+1];

int32 steps_per_rpm[max_stepper+1];

double on_speed[max_stepper+1];


double lulgol;
int32 jpos=0;
int32 jpos_s=0;

// encoder variables
short ___diff, ___new_status, old_status[max_stepper+1];
short use_enc[max_stepper+1];  // As long as this number stays zero, no check for encoder is performed for this stepper
int32 steps_enc[max_stepper+1]; // number of steps counted on encoder
int32 burning_steps=0; // steps decelerating with burning stopswitch
int32 not_burning=0; // when homing and stopswitch switches off, number of not burning steps should reach 20.
int32 missed_enc[max_stepper+1]; // number of undefined steps on encoder
int32 max_allowed_enc[max_stepper+1];// maximum allowed number of missed steps
int32 missed_steps[max_stepper+1]; //missed steps
_Q16 step_enc_ratio[max_stepper+1];//= steps_mm/steps_mm_encoder, always around 1 in Q15/16 format
short A_val;
short B_val;
int64_t kul64;
int32 kul32;
int32 command_id=-65535; // current command
int32 saved_id=-65535; // current command
int32 first_command=0;
short check_packet_loss=0; //check_packet_loss, is checked when equal 1. Is initialized on every 8:y
uint32_t wait_millis;
uint32_t wait_timer;

unsigned int get_stopsw_status(unsigned int sw){
#if !defined (STM32F411xE)
		TM_GPIO_SetPinHigh(GPIOA,GPIO_Pin_3);
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_4)))   sw = sw ^ 1;     //1L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_5)))   sw = sw ^ 2;     //1R
		TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_12);
			if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_4)))   sw = sw ^ 4;     //2L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_5)))   sw = sw ^ 8; //2R
		TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_14);
			if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_15))) sw = sw ^ 16;      //3L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_8)))  sw = sw ^ 32; //3R
		TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_9);
			if (NCO (TM_GPIO_GetInputPinValue(GPIOE,GPIO_Pin_5))) sw = sw ^ 64;       //4L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOE,GPIO_Pin_6))) sw = sw ^ 128; //4R
#else
		TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_9);
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_4)))   sw = sw ^ 1;     //1L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_5)))   sw = sw ^ 2; //1R
		TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_12);
			if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_2)))   sw = sw ^ 4;     //2L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_3)))   sw = sw ^ 8; //2R
		TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_14);
			if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_15))) sw = sw ^ 16;      //3L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_8)))  sw = sw ^ 32;      //3R
		TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_15);
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_1))) sw = sw ^ 64;       //4L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_2))) sw = sw ^ 128; //4R
#endif
			return sw;
}


void initialize_encoder(short astepper)
// always call this function at the beginning of a number of steps for one step motor.
{
  investigate_encoder(astepper);
  missed_enc[astepper] = 0;
  steps_enc[astepper] = 0;
}


void investigate_encoder(short astepper)

{
#if !defined (STM32F411xE)
      switch (astepper) {
              case 1: A_val = TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_6);
                      B_val = TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_7);
                      break;
              case 2: A_val = TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_8);
		      	  	  B_val = TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_9);
		      	  	  break;
              case 3: A_val = TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_0);
		      	  	  B_val = TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_1);
		      	  	  break;
              case 4: A_val = TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_2);
              		  B_val = TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_3);
              		  break;
              }// switch stepper
#else
      switch (astepper) {
                    case 1: A_val = TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_6);
                            B_val = TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_7);
                            break;
                    case 2: A_val = TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_10);
      		      	  	  B_val = TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_11);
      		      	  	  break;
                    case 3: A_val = TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_0);
      		      	  	  B_val = TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_1);
      		      	  	  break;
                    case 4: A_val = TM_GPIO_GetInputPinValue(GPIOA,GPIO_Pin_8);
                    		  B_val = TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_9);
                    		  break;
                    }// switch stepper
#endif

  if (!A_val && !B_val) {___new_status= 1;}
    else if (A_val && !B_val) {___new_status= 2;}
         else if (A_val && B_val) {___new_status= 3;}
              else if (!A_val && B_val) {___new_status= 4;}
  if (old_status[astepper]==0) { ___diff =0;} //start situation
  else  ___diff = ___new_status - old_status[astepper];
  if ((___diff == -3) || (___diff == 1)) {++steps_enc[astepper];prev_action[astepper]=1;}
  else if ((___diff == 3) || (___diff == -1)) {--steps_enc[astepper];prev_action[astepper]=-1;}
       else if (abs(___diff) == 2) { ++missed_enc[astepper];if (prev_action[astepper] >= 0) {steps_enc[astepper]=steps_enc[astepper]+2;} else {steps_enc[astepper]=steps_enc[astepper]-2;}} //undefined status change, no good...
  old_status[astepper] = ___new_status;
  }// investigate_encoder



int tell_difference_encoder(short astepper)
// return number of missed steps, note that still encoder could be wrong or step motor.
// enc_pos_mm := steps_enc / steps_mm_encoder;
// step_pos_mm := steps/ steps_mm;
// missed steps= (step_pos_mm- enc_pos_mm) * steps_mm
// thus if step_enc_ratio between encoder steps and step motor steps = steps_mm/steps_mm_encoder then
// missed_steps= (steps/steps_mm - steps_enc/steps_mm_encoder) * steps_mm
// missed_steps= (steps - steps_enc * steps_mm/steps_mm_encoder)
// missed_steps= steps - steps_enc * step_enc_ratio
// do the calculation using our home-made Q47/16, note step_enc_ratio is already in Q15/16 format because multiplication is with around one, no overflow results

{
  investigate_encoder(astepper);
  //kul64 = (int64_t)(abs(steps_enc)) << 16;
 // if ((step_no > 52450) && (step_no < 52470)) {
 //	  trace_printf("kul64: %i \n",kul64);
 //  }
  kul64 = ((int64_t)(abs(steps_enc[astepper])) << 16)* step_enc_ratio[astepper];
  kul32 = kul64 >> 32;
  //if ((step_no > 52450) && (step_no < 52470)) {
//	  trace_printf("kul32: %i kul64: %i steps_enc %i\n",kul32,kul64,steps_enc);
//  }
  return abs(stepper_steps[astepper]-kul32);// note, currently the sign of step_enc_ration and stepno is not used because both are positive integers, note that absolute value steps_enc is shifted.
}




void check_command(char *dumstr, char achar, int *succs, char *dumstr2, short overwrite_command_id)
//*dumstr is input. Should be in form <abbbb$ccccX> where a is command char, bbbb= command information, cccc is optional and is command id, X is the step motor
// achar is the command to be checked. succes is 1 when the command is found.
// succes is zero when no command is not found, 1 when found, 2 when command does not contain closing bracket.
// dumstr2 contains " FAULT:  " when te command does not contain closing bracket '>' dumstr2 is bbbb when command is successfully found.
// stepper (global variable) is put equal X

{
       char *pos1;
       char *pos2;
       char *pos3;
       short found_d, first_comm;

       *dumstr2 = '\0';
       pos1 = strchr(dumstr,achar);
       pos2 = pos1-1;
       if ((pos1 != NULL) && (*pos2=='<'))//command is found
       {
           found_d = 1;
           pos2 = strchr(++pos1,'$');
           if (pos2 == NULL)
              {pos2 = strchr(pos1,'>');found_d=0;}
           if (pos2 != NULL)
              {
                  strncpy(dumstr2,pos1,pos2-pos1-1);
                  dumstr2[pos2-pos1-1] = '\0';
                  *succs=1;
                  if (achar == 'c') //when a c-command is send (save command in buffered list), the current stepper should not be overwritten............
                	  c_stepper = (short) *(--pos2)-'0';
                  else if (achar != 'm') stepper = (short) *(--pos2)-'0';  //in case of pause command, do not overwrite stepper because STM32 has to continue with current stepper once pause ==0
                  if (found_d){
                	 pos3 = strchr(++pos2,'>');
                     if (pos3 != NULL)
                                {
                    	            ++pos2;
                    	            strncpy(dummystr3,pos2,pos3-pos2);
                                    dummystr3[pos3-pos2] = '\0';
                                    if (overwrite_command_id) {
                                       command_id = atoi(dummystr3); //current command id, supplied by client. optional parameter.
                                    } else saved_id = atoi(dummystr3); //save for use in buffered commands, see main.c where it is used
                                }
                  } else {
                	  if (overwrite_command_id) {
                	  command_id = -65535;} else {saved_id = -65535;}
                  }
              } else  {strcpy(dumstr2," FAULT:  ");
                       dumstr2[strlen(dumstr2)-2] = achar;
                       *succs=2;}
       } else {*succs=0;}
}// check_command

void enable_disable(short astepper,unsigned int aenable)
{
#if !defined (STM32F411xE)
switch (astepper) {
             case 1: if (aenable) TM_GPIO_SetPinLow(GPIOA,GPIO_Pin_1);
                     else TM_GPIO_SetPinHigh(GPIOA,GPIO_Pin_1); break;// enable/ disable
             case 2: if (aenable) TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_10);
                     else TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_10); break;
             case 3: if (aenable) TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_12);
                     else TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_12); break;
             case 4: if (aenable) TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_7);
                     else TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_7); break;
          }
#else
switch (astepper) {
             case 1: if (aenable) TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_7);
                     else TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_7); break;// enable/ disable
             case 2: if (aenable) TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_10);
                     else TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_10); break;
             case 3: if (aenable) TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_12);
                     else TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_12); break;
             case 4: if (aenable) TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_13);
                     else TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_13); break;
          }
#endif
} //enable_disable

void getxyzw(char *dumstr)
// this routine is used to mill 3D line segments. the highest stepper distance is leading.
{
char *pos1;
char *pos2;

char dmm[APP_READ_BUFFER_SIZE];
int32 X,Y,Z,W,Xstepmotor;
float F;

dmm[0]='\0';
pos1 = strchr(dumstr,'%');
//kul = 1;
//if (pos1==NULL) trace_printf("stuff is null 1: %i \n",kul);
pos2 = strchr(++pos1,'%');
//if (pos2==NULL) trace_printf("stuff is null 2: %i \n",kul);

strncpy(dmm,pos1,pos2-pos1);
dmm[pos2-pos1]='\0';
X = atol(dmm);
if (X >= 0) {sign_total_steps[1] = 0;}
else {sign_total_steps[1] = 1;}
X=abs(X);
total_steps_array[1] = X;
total_steps = X;
stepper = 1;

pos1 = pos2;
pos2 = strchr(++pos1,'%');
strncpy(dmm,pos1,pos2-pos1);
dmm[pos2-pos1]='\0';
Y = atol(dmm);
if (Y >= 0) {sign_total_steps[2] = 0;}
else {sign_total_steps[2] = 1;}
Y = abs(Y);
total_steps_array[2] = Y;
if (Y > X) {stepper = 2;total_steps = Y;}

pos1 = pos2;
pos2 = strchr(++pos1,'%');
strncpy(dmm,pos1,pos2-pos1);
dmm[pos2-pos1]='\0';
Z = atol(dmm);
if (Z >= 0) {sign_total_steps[3] = 0;}
else {sign_total_steps[3] = 1;}
Z = abs(Z);
total_steps_array[3] = Z;
if (Z > total_steps_array[stepper]){stepper = 3;total_steps = Z;}

pos1 = pos2;
pos2 = strchr(++pos1,'%');
strncpy(dmm,pos1,pos2-pos1);
dmm[pos2-pos1]='\0';
W = atol(dmm);
if (W >= 0) {sign_total_steps[4] = 0;}
else {sign_total_steps[4] = 1;}
W = abs(W);
total_steps_array[4] = W;
if (W > total_steps_array[stepper]){stepper = 4;total_steps = W;}

pos1 = pos2;
pos2 = strchr(++pos1,'%');
strncpy(dmm,pos1,pos2-pos1);
dmm[pos2-pos1]='\0';
F = atof(dmm);
if (abs(F) > 1E-5) {
	  pos1 = pos2;
	  pos2 = strchr(++pos1,'%');//B contains stepmotor the feedrate is based on
	  strncpy(dmm,pos1,pos2-pos1);
	  dmm[pos2-pos1]='\0';
	  Xstepmotor = atol(dmm);
	for (int i = 1; i <= max_stepper; i++) {
		    	rpm[i]=F*rpm_feed[i]/rpm_feed[Xstepmotor];// only stepmotor1 is send as F...... Scale the other one accordingly
	}
}
else {//feedrate is zero, use previously send feedrates.
  pos1 = pos2;
  pos2 = strchr(++pos1,'%');
  strncpy(dmm,pos1,pos2-pos1);
  dmm[pos2-pos1]='\0';
  if (atol(dmm)==1) {// use maximum rate
	  for (int i = 1; i <= max_stepper; i++) {
		  rpm[i]=rpm_max[i];
	  }
  } else
  {// use feed rate
	  for (int i = 1; i <= max_stepper; i++) {
	    	rpm[i]=rpm_feed[i];
	  }
  }// use feed rate
}//feedrate is zero, use previously send feedrates.



for (int i = 1; i <= max_stepper; i++) {
	total_steps_d2_array[i]=2*total_steps_array[i];
	err_array[i] =total_steps_d2_array[i]-total_steps_array[stepper];
	}//for
use3D = 1;
if (total_steps == 0) {
#if defined(DEBUG_OUTPUT)
                        trace_printf("total steps zero: %i \n",total_steps);
#endif
}

}


int check_all_read(int jpos_size)
// routine returns true if all commands are read to go stepping. false if not.
// routine always prints last command OR ready if all commands are read.
// in use: a,c,f,h,k,l,o,p,q,r,s,v,y,w
{
	   if  ((jpos_size > 0) && (CRC_CCITT(&unsigned_data[0],  jpos_size+1)!=0)) {//array starts at zero. thus number of chars is jpos+1
		   strcpy(dummystr,"667:<crc fault>");
	   }
	   else{
       check_command(Command_data,'q', &succes,dummystr2,1);
       //try read total steps to be on speed in form <q number_of_steps stepper>, may only be used after setting all parameters, note maximum of 2^32 !!
       if (succes == 1) {
    	   if (strchr(dummystr2,'%') != NULL) { //line in 3D space
    		  getxyzw(dummystr2);
    	   }
    	   else {
    		 use3D = 0;
             total_steps = atol(dummystr2);
             if (total_steps >= 0) {sign_total_steps[stepper] = 0;}
             else {sign_total_steps[stepper] = 1;} // change direction once more!
             total_steps = abs(total_steps);  // when working with fixed point integers, bit shifting operations on signed integers gives compiler specific behaviour because of sign bit
    	   }
           strcpy(dummystr,"<1:");
           strcat(dummystr,dummystr2);
           strcat(dummystr,">#");
//           sprintf(dummystr2,"1:%ld>",total_steps); // convert back to validate result.
           stopsw = 0; stopswh = 0;stopswitched = 0;
           homing = 0; // normal stepping, stop on stopswitch
           config_read[stepper]=all_read;
       } else
       {
       check_command(Command_data,'v', &succes,dummystr2,1);
       //try read direction in form <v direction stepper>, succes is zero when no succes, 1 when succes, 2 when fault
       if (succes == 1) {
           direction[stepper] = atoi(dummystr2);
           strcpy(dummystr,"<9:");
           strcat(dummystr,dummystr2);
           strcat(dummystr,">#");
//           sprintf(dummystr2,"9:%i>",direction); // convert back to validate result.
           stopsw = 0; stopswh = 0; stopswitched = 0;
//           config_read[stepper]=config_read[stepper] | 32;
       } else
       {
       check_command(Command_data,'f', &succes,dummystr2,1);
       //try read enable in form <f enable stepper>, succes is zero when no succes, 1 when succes, 2 when fault
       if (succes == 1) { // note, no need to save state. do it directly.
           enable = atoi(dummystr2);
           strcpy(dummystr,"<11:");
           strcat(dummystr,dummystr2);
           strcat(dummystr,">#");
  //         sprintf(dummystr2,"11:%i>",enable); // convert back to validate result.
           enable_disable(stepper,enable);
       } else
       {
       check_command(Command_data,'s', &succes,dummystr2,1);
       //try read total_steps in form <s number_of_steps stepper>, succes is zero when no succes, 1 when succes, 2 when fault, note maximum of 2^32
       if (succes == 1) {
           total_steps = atol(dummystr2);
           if (total_steps >= 0) {sign_total_steps[stepper] = 0;}
           else {sign_total_steps[stepper] = 1;} // change direction once more!
           total_steps = abs(total_steps);  // when working with fixed point integers, bit shifting operations on signed integers gives compiler specific behaviour because of sign bit
           use3D = 0;
           strcpy(dummystr,"<6:");
           strcat(dummystr,dummystr2);
           strcat(dummystr,">#");
//           sprintf(dummystr2,"6:%ld>",total_steps); // convert back to validate result.
           homing = 0; // normal stepping, stop on stopswitch
           config_read[stepper]=config_read[stepper] | 1; // note, either homing or steps give this result.
       } else
       {
       check_command(Command_data,'h', &succes,dummystr2,1);
       //try read homing in form <h stopswitch_usage stepper>, succes is zero when no succes, 1 when succes, 2 when fault
       if (succes == 1) {
           neglect = atoi(dummystr2);
           switch (neglect){
           case 1: use1=1; use2=0; break;// neglect=1  means right stopswitch is used for homing.
           case 2: use1=0; use2=1; break;// neglect=2  means left stopswitch is used for homing.
           default: trace_printf("Value for neglect is not valid homing command"); break;
           }
           stopsw = 0; stopswh = 0; stopswitched = 0;
           strcpy(dummystr,"<6:");
           strcat(dummystr,dummystr2);
           strcat(dummystr,">#");
//           sprintf(dummystr2,"6:%i>",neglect); // convert back to validate result.
           homing = 1; // homing, do not stop on neglect stopswitch
           for (int i = 1; i <= max_stepper; i++) {
           		rpm[i]=rpm_max[i];
           }
           use3D = 0;
		   total_steps = 1000000000 ; // put equal to maximum.
           config_read[stepper]=config_read[stepper] | 1;
       } else
       {
       check_command(Command_data,'j', &succes,dummystr2,1);
       //try read homing in form <h stopswitch_usage stepper>, succes is zero when no succes, 1 when succes, 2 when fault
       if (succes == 1) {
           neglect = atoi(dummystr2);
           switch (neglect){
           case 1: use1=1; use2=0; break;// neglect=1  means right stopswitch is used for homing.
           case 2: use1=0; use2=1; break;// neglect=2  means left stopswitch is used for homing.
           default: trace_printf("Value for neglect is not valid homing command"); break;
           }
           stopsw = 0; stopswh = 0; stopswitched = 0;
           strcpy(dummystr,"<6:");
           strcat(dummystr,dummystr2);
           strcat(dummystr,">#");
//           sprintf(dummystr2,"6:%i>",neglect); // convert back to validate result.
           homing = 1; // homing, do not stop on neglect stopswitch
           for (int i = 1; i <= max_stepper; i++) {
           		rpm[i]=rpm_max[i];
           }
           use3D = 0;
		   total_steps = 1000000000 ; // put equal to maximum.
           config_read[stepper]=all_read;
       } else
       {
       check_command(Command_data,'p', &succes,dummystr2,1);
       //try read steps per rpm in form <p steps_per_rpm stepper>
       if (succes == 1) {
           steps_per_rpm[stepper]  = atoi(dummystr2);
           config_read[stepper]=config_read[stepper] | 2;
           strcpy(dummystr,"<5:");
           strcat(dummystr,dummystr2);
           strcat(dummystr,">#");
//           sprintf(dummystr2,"5:%i>",steps_per_rpm); // convert back to validate result.
       } else
       {
	       check_command(Command_data,'b', &succes,dummystr2,1);
	       //try read  in form <b max_allowed_enc stepper>  where max_allowed_enc is maximum number of steps to slip
	       if (succes == 1) {
		   max_allowed_enc[stepper]  = atoi(dummystr2)*4;// times 4 because if encoder states 1000 steps, in reality this is 4000 steps per RPM.
		   use_enc[stepper]=1;
	           strcpy(dummystr,"<13:");
	           strcat(dummystr,dummystr2);
	           strcat(dummystr,">#");
	//           sprintf(dummystr2,"5:%i>",steps_per_rpm); // convert back to validate result.
	}  else
    {
	       check_command(Command_data,'x', &succes,dummystr2,1);
	       //try read  in form <x stepper>  if this command is send, encoder is disabled for this stepper.
	       if (succes == 1) {
		       use_enc[stepper]=0;
	           strcpy(dummystr,"<14:");
	           strcat(dummystr,dummystr2);
	           strcat(dummystr,">#");
	//           sprintf(dummystr2,"5:%i>",steps_per_rpm); // convert back to validate result.
	}  else
	{
		       check_command(Command_data,'d', &succes,dummystr2,1);
		       //try read  in form <b step_enc_ratio stepper>
		       if (succes == 1) {
			   lulgol  = strtod(dummystr2, (char **)NULL);
			   lulgol = lulgol/4;  // because if encoder states 1000 steps, in reality this is 4000 steps per RPM.
			   if (lulgol >= 0) {sign_encoder_ratio=0;}
			   else {sign_encoder_ratio=1;lulgol=-lulgol;}
			   step_enc_ratio[stepper] = fix16_from_dbl(lulgol);// steps_per_rpm/encoder_steps_RPM
			   strcpy(dummystr,"<12:");
			   strcat(dummystr,dummystr2);
			   strcat(dummystr,">#");// note, for floats, the ">" must be added !
	}else
	{
		       check_command(Command_data,'i', &succes,dummystr2,1);
		       //try read  in form <i delay_in_seconds stepper>
		       if (succes == 1) {
			   lulgol  = strtod(dummystr2, (char **)NULL);
			   wait_millis = round(lulgol*1000); // milli seconds
			   wait_timer=TM_Time;
			   strcpy(dummystr,"<18:");
			   strcat(dummystr,dummystr2);
			   strcat(dummystr,">#");// note, for floats, the ">" must be added !
	} else
       {
       check_command(Command_data,'l', &succes,dummystr2,1);
       //try read prescaler in form <l prescaler stepper> note, no longer in use, can be used for something else. Prescaler is now calculated for every command.
       if (succes == 1) {
           prescaler  = atoi(dummystr2);
           config_read[stepper]=config_read[stepper] | 16;
           strcpy(dummystr,"<4:");
           strcat(dummystr,dummystr2);
           strcat(dummystr,">#");
//           sprintf(dummystr2,"4:%i>",prescaler); // convert back to validate result.
       } else
       {
       check_command(Command_data,'r', &succes,dummystr2,1);
       //try read rpm in form <r rpm stepper>, max speed
       if (succes == 1) {
          rpm[stepper]  = strtod(dummystr2, (char **)NULL);
          rpm_max[stepper] = rpm[stepper];
          config_read[stepper]=config_read[stepper] | 4;
          strcpy(dummystr,"<3:");
          strcat(dummystr,dummystr2);
          strcat(dummystr,">#");// note, for floats, the ">" must be added !
//          sprintf(dummystr2,"3:%f>",rpm); // convert back to validate result.
       } else
       {
              check_command(Command_data,'g', &succes,dummystr2,1);
              //try read rpm in form <r rpm stepper>, max speed
              if (succes == 1) {
                 rpm_feed[stepper]  = strtod(dummystr2, (char **)NULL);
                 strcpy(dummystr,"<17:");
                 strcat(dummystr,dummystr2);
                 strcat(dummystr,">#");// note, for floats, the ">" must be added !
       //          sprintf(dummystr2,"3:%f>",rpm); // convert back to validate result.
              } else
       {
       check_command(Command_data,'a', &succes,dummystr2,1);
       // enable/ disable stepper
       if (succes == 1) {
          enable_st[stepper]  = atoi(dummystr2);
          if (enable_st[stepper]==0) config_read[stepper] = config_init;
          //config_read[stepper]=config_read[stepper] | 128;
          strcpy(dummystr,"<15:");
          strcat(dummystr,dummystr2);
          strcat(dummystr,">#");
//          sprintf(dummystr2,"12:%i>",decel_steps); // convert back to validate result.
       } else
       {
       check_command(Command_data,'c', &succes,dummystr2,1);
       //Read commands in buffer, form <c command>
       if (succes == 1) {// note, this code is repeated in main.c. modifications here should be reflected there as well.............
    	  if (((check_packet_loss) && (first_command==command_id)) || (!check_packet_loss)){
    	  ++first_command;
    	  if (no_commands < max_command)  ++no_commands;
    	  strcpy(Command_buffer[no_commands],dummystr2); // save command in buffer for future use.
          strcpy(dummystr,"10:<c");
          strcat(Command_buffer[no_commands],itoa(c_stepper,dummystr2,10) );
          if (command_id != -65535){
             strcat(Command_buffer[no_commands],"$");
             strcat(Command_buffer[no_commands],itoa(command_id,dummystr2,10) );
          }
          strcat(dummystr,Command_buffer[no_commands]);
          strcat(dummystr,">");
          strcat(dummystr, itoa(no_commands,dummystr2,10));
          strcat(dummystr,";0#");
    	  } else {
              strcpy(dummystr,"666:<c");
              strcat(dummystr,itoa(first_command,dummystr2,10) );
              strcat(dummystr,">");
              strcat(dummystr, itoa(no_commands,dummystr2,10));
              strcat(dummystr,";0#");
    	  }

          //trace_printf("command: %s\n", Command_buffer[no_commands]);

//          sprintf(dummystr2,"10:%i>",stepper); // convert back to validate result.
       } else
       {
       check_command(Command_data,'o', &succes,dummystr2,1);
       //try read time to be on speed in form <o on_speed stepper>
       if (succes == 1) {
           on_speed[stepper]  =  strtod(dummystr2, (char **)NULL);
           strcpy(dummystr,"<2:");
           strcat(dummystr,dummystr2);
           strcat(dummystr,">#");// note, for floats, the ">" must be added !
//           sprintf(dummystr2,"2:%f>",on_speed); // convert back to validate result.
           config_read[stepper]=config_read[stepper] | 8;
       }
       else
              {
              check_command(Command_data,'u', &succes,dummystr2,1);
              //try read time to be on speed in form <o on_speed stepper>
              if (succes == 1) {
            	  unsigned int sw1;
            	  sw1 = get_stopsw_status(0);
                  strcpy(dummystr,"<19:");
                  strcat(dummystr,itoa(sw1,dummystr2,10));
                  strcat(dummystr,">#");
              }
       else
       {
       check_command(Command_data,'e', &succes,dummystr2,1);
       //try read time to be on speed in form <o on_speed stepper>
       if (succes == 1) {
    	   check_packet_loss = 1;
           first_command  =  atoi(dummystr2);
           strcpy(dummystr,"<16:");
           strcat(dummystr,dummystr2);
           strcat(dummystr,">#");
       } else
       {
       check_command(Command_data,'m', &succes,dummystr2,1);
    	          //try read paused, can only become one here because check_all_read is only called when paused==zero and no step command is running. so if it becomes zero no harm is done
       if (succes == 1) {
    	       	      paused = atoi(dummystr2);
    	       	      if (paused==0){
    	       	          stepper=paused_stepper;}
    	       	      else {paused_stepper=stepper;}
    	              strcpy(dummystr,"<19:");
    	              strcat(dummystr,dummystr2);
    	              strcat(dummystr,">#");
       } else
       {
       check_command(Command_data,'y', &succes,dummystr2,1);   //ready for new program, program reset
       if (succes == 1) {
       if (paused==1) {stepper=paused_stepper;}
       config_read[stepper]=config_init;
       status = 0;
       paused = 0;
       paused_steps =0;
       homing = 0;
       stopswitched = 0;
       stopsw = 0;
       first_command=0;
       wait_timer = 0;
       step_no = total_steps+1;
       check_packet_loss=0;
       no_commands = 0;
       strcpy(dummystr,"<");
       //memcpy(dummystr,dummystr2,STLN);
       strcat(dummystr,dummystr2);
       strcat(dummystr,">#");// note, for floats, the ">" must be added !
//       sprintf(dummystr2,"%s>",Command_data); // convert back to validate result.
       } else
       {
              check_command(Command_data,'z', &succes,dummystr2,1);   //ready for new program, program reset
              if (succes == 1) {
            	  strcpy(dummystr,"<14:");
            	  strcat(dummystr,dummystr2);
#if defined (USE_USB)
                  strcat(dummystr,"version:4.3 USB>#");  // return current version number
#else
                  strcat(dummystr,"version:4.3 Bluetooth>#");  // return current version number
#endif
       } else
       {
       check_command(Command_data,'k', &succes,dummystr,1);   //ready to go
       if (succes == 1)
          {if (config_read[stepper]==255) config_read[stepper]=all_read;}
       else memcpy(dummystr, Command_data, STLN); // command unknown after all
       }       }       }       }       }       }       }      }    	}     }    }     } }}}}}}}}}}}


       if (config_read[stepper]==all_read) {
    	   strcpy(dummystr,"<0:rdy>0;#");
#if defined (USE_USB)
	   TM_USB_VCP_Puts(dummystr);
#else
	   //TM_USART_DMA_Send(USART2, (uint8_t *)dummystr, strlen(dummystr));
	   //TM_USART_Send(USART2, (uint8_t *)dummystr, strlen(dummystr));
#if defined(DEBUG_SR)
       trace_printf("Sending: %s\n", dummystr);
#endif

#if !defined (STM32F411xE)
	   TM_USART_Puts(USART3,dummystr);
#else
	   TM_USART_Puts(USART2,dummystr);
#endif
#endif
	   return 1;
       } else {
    	          //Give feedback upon status and how many commands pending in buffer.
    	          if (no_commands > 0) {
    	             strcat(dummystr,itoa(no_commands,dummystr2,10));
    	          }
    	          else if (status>0) {strcat(dummystr,itoa(9999,dummystr2,10));}
    	               else {strcat(dummystr,itoa(0,dummystr2,10));}
    	          strcat(dummystr,";");
    	           if (homing ==0){
    	              strcat(dummystr,itoa((int)(100.0*((float) step_no/(float) total_steps)),dummystr2,10));
    	              strcat(dummystr,"#");
    	           } else {strcat(dummystr,"0#");}
#if defined(DEBUG_SR)
                  trace_printf("Sending: %s\n", dummystr);
#endif

#if defined (USE_USB)
    	          TM_USB_VCP_Puts(dummystr);
#else
    	          //TM_USART_DMA_Send(USART2, (uint8_t *)dummystr, strlen(dummystr));
    	          //TM_USART_Send(USART2, (uint8_t *)dummystr, strlen(dummystr));
#if !defined (STM32F411xE)
    	          TM_USART_Puts(USART3,dummystr);
#else
    	          TM_USART_Puts(USART2,dummystr);
#endif
#endif
    	          return 0;}
}


void EnableTimerInterrupt()
 {
     NVIC_InitTypeDef nvicStructure;

     nvicStructure.NVIC_IRQChannel = 28;//TIM2_IRQn gives stupid resolve error
     nvicStructure.NVIC_IRQChannelPreemptionPriority = 0;
     nvicStructure.NVIC_IRQChannelSubPriority = 1;
     nvicStructure.NVIC_IRQChannelCmd = ENABLE;
     NVIC_Init(&nvicStructure);
 }

void DisableTimerInterrupt()
 {
     NVIC_InitTypeDef nvicStructure;
     nvicStructure.NVIC_IRQChannel = 28; //TIM2_IRQn gives stupid resolve error
     nvicStructure.NVIC_IRQChannelCmd = DISABLE;
     NVIC_Init(&nvicStructure);
 }


void InitializeTimer(uint16_t a_prescaler,uint32_t period)
{
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef timerInitStructure;
    timerInitStructure.TIM_Prescaler = a_prescaler-1;
    timerInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    timerInitStructure.TIM_Period = period-1;
    timerInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    timerInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &timerInitStructure);
    TIM2->CR1 |= TIM_CR1_ARPE | TIM_CR1_CEN; // autoreload on, counter enabled

    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_Cmd(TIM2, ENABLE);
}


void check_cn()
{
_Q16 n_real_min_un_int;
double div_result, dummyfloat;
          div_result =  total_steps/2.0;
          n_acc_decc = round(div_result);
          n_accd = pow(omega_max,2.0);
          div_result = n_accd/(0.736*alpha*omega_acc);
          n_accd = div_result;
          n_deccd = total_steps * 1.0;
          n_deccd = n_deccd-div_result;
          if (n_deccd < 0) {n_deccd = 0;}
          if (n_accd > total_steps) {n_accd = total_steps;}
          n_acc = round(n_accd);
          n_decc = round(n_deccd);

          n_real_min = MIN(n_acc,n_acc_decc);
          n_real_max = MAX(n_decc,n_acc_decc);
          switch (total_steps)
          {//case to prevent division by zero
          case 1:
             n_real_min = 1;
             n_real_max = 2;
             break;
          case 2:
             n_real_min = 0;
             n_real_max = 3;
             break;
          case 3:
             n_real_min = 2;
             n_real_max = 1;
             break;
          }

          n_real_min_un_int = n_real_min; // note that maximum is 65535 !!!!!!!


          Qn_real_min = fix16_from_int(n_real_min_un_int); // _Q16 from integer
          Q16_one = fix16_from_int(1);
          Qn_real_min = fix16_div(Q16_one,Qn_real_min);

          Q_same = fix16_from_int(total_steps-n_real_max-1);   // _Q16 that equals (total_steps-n_real_max-1)
          Q_same = fix16_div(Q16_one,Q_same);

          dummyfloat = 0.676*frequency*sqrt(2.0*alpha/omega_acc)+0.5;
          cn=cn_1 = fix16_from_dbl(dummyfloat);
          step_no = 1;
}// check_cn


void move_stepper_phase_a(int astepper){
if (!enable_st[astepper]) return;
if (use_enc[astepper] && (homing==0)) {
    investigate_encoder(astepper);}// one call below insufficient, missed steps..........
#if defined(USE_STOP_SWITCHES)
		// normally open stopswitches, in fault condition a one, opto coupler is conducting, leading to low value on discovery input. modify defines.h to change NCO function

#if !defined (STM32F411xE)
		switch (astepper) {
		case 1: TM_GPIO_SetPinHigh(GPIOA,GPIO_Pin_3);
		if (stopswitched==0){ // in this way the stopsw status stays the same upon deceleration.
			stopsw=0;
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_4)))   stopsw = 1;     //1L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_5))) stopsw = 2; //1R
		}
		break;
		case 2: TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_12);
		if (stopswitched==0){ // in this way the stopsw status stays the same upon deceleration.
			stopsw=0;
			if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_4)))   stopsw = 3;     //2L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_5))) stopsw = 4; //2R
		}
		break;
		case 3: TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_14);
		if (stopswitched==0){ // in this way the stopsw status stays the same upon deceleration.
			stopsw=0;
			if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_15))) stopsw = 5;      //3L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_8))) stopsw = 6; //3R
		}
		break;
		case 4: TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_9);
		if (stopswitched==0){ // in this way the stopsw status stays the same upon deceleration.
			stopsw=0;
			if (NCO (TM_GPIO_GetInputPinValue(GPIOE,GPIO_Pin_5))) stopsw = 7;       //4L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOE,GPIO_Pin_6))) stopsw = 8; //4R
		}
		break;
		}
#else
		switch (astepper) {
		case 1: TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_9);
		if (stopswitched==0){ // in this way the stopsw status stays the same upon deceleration.
			stopsw=0;
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_4)))   stopsw = 1;     //1L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_5))) stopsw = 2; //1R
		}
		break;
		case 2: TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_12);
		if (stopswitched==0){ // in this way the stopsw status stays the same upon deceleration.
			stopsw=0;
			if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_2)))   stopsw = 3;     //2L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_3))) stopsw = 4; //2R
		}
		break;
		case 3: TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_14);
		if (stopswitched==0){ // in this way the stopsw status stays the same upon deceleration.
			stopsw=0;
			if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_15))) stopsw = 5;      //3L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_8))) stopsw = 6; //3R
		}
		break;
		case 4: TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_15);
		if (stopswitched==0){ // in this way the stopsw status stays the same upon deceleration.
			stopsw=0;
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_1))) stopsw = 7;       //4L
			if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_2))) stopsw = 8; //4R
		}
		break;
		}
#endif
#else
		//TM_DISCO_LedToggle(LED_ORANGE); return; // indicate stopswitch
		stopsw=0;
#if !defined (STM32F411xE)
		switch (astepper) {
		case 1: TM_GPIO_SetPinHigh(GPIOA,GPIO_Pin_3); break;
		case 2: TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_12); break;
		case 3: TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_14); break;
		case 4: TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_9); break;
		}
#else
		switch (astepper) {
		case 1: TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_9); break;
		case 2: TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_12); break;
		case 3: TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_14); break;
		case 4: TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_15); break;
		}
#endif
		TM_DISCO_LedOn(LED_ORANGE); // use orange LED to simulate stepper

#endif
}// move_stepper

void move_stepper_phase_b(int astepper){
stepper_steps[astepper]++;
if (!enable_st[astepper]) return;
#if defined(USE_STOP_SWITCHES)
#else
		TM_DISCO_LedOff(LED_ORANGE); // use orange LED to simulate stepper
#endif
#if !defined (STM32F411xE)
		switch (astepper) {
		case 1: TM_GPIO_SetPinLow(GPIOA,GPIO_Pin_3); break;
		case 2: TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_12); break;
		case 3: TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_14); break;
		case 4: TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_9); break;
		}
#else
		switch (astepper) {
		case 1: TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_9); break;
		case 2: TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_12); break;
		case 3: TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_14); break;
		case 4: TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_15); break;
		}
#endif
}


void check_moving_steppers(){
    for (int i = 1; i <= max_stepper; i++) {
    	if (i != stepper) {
    		if (err_array[i] > 0) {
    			move_stepper_phase_a(i);
    			move_it[i]=1;
    			err_array[i] = err_array[i]-total_steps_d2_array[stepper];
    		} else {move_it[i]=0;}
    		err_array[i] = err_array[i]+total_steps_d2_array[i];
    	} else {move_it[i]=1; move_stepper_phase_a(i);}
    	if (stopsw != 0) {break;}
    }//for
}



void take_step ()

{
//	if (status != 1) {TM_DISCO_LedToggle(LED_ORANGE); return;}

	if (homing>0) {
		stopswh=0;  //note, stopswh is only used to signal back over the interface if the stopswitch is really off after homing.
		if (homing==1) {homing_steps++;}
		else {homing_steps--;}
#if !defined (STM32F411xE)
		switch (stepper) {
		case 1: if (enable_st[1]){
		TM_GPIO_SetPinHigh(GPIOA,GPIO_Pin_3);//C4 = 1L, C5 =1R thus if use1= 1, then use2= 0 and thus C4 is normally used
		if ((stopswitched==0) || (homing==2)){ // in this way the stopsw status stays the same upon deceleration.
	       if ((NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_4))&&use1) || (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_5))&&use2)) stopsw = 1; else stopsw = 0;
		}
		if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_4)) && !use1)   stopswh = 1; if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_5))&& !use2) stopswh = 2;
		}
		break;
		case 2: if (enable_st[2]){
		TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_12);
		if ((stopswitched==0) || (homing==2)){ // in this way the stopsw status stays the same upon deceleration.
		   if ((NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_4))&&use1) || (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_5))&&use2)) stopsw = 1; else stopsw = 0;
		}
		if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_4))&& !use1)   stopswh = 3; if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_5))&& !use2) stopswh = 4;} break;
		case 3: if (enable_st[3]){
		TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_14);
		if ((stopswitched==0) || (homing==2)){ // in this way the stopsw status stays the same upon deceleration.
		   if ((NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_15))&&use1) || (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_8))&&use2)) stopsw = 1; else stopsw = 0;
		}
		if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_15))&& !use1) stopswh = 5; if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_8))&& !use2) stopswh = 6;}break;
		case 4: if (enable_st[4]){
		TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_9);
		if ((stopswitched==0) || (homing==2)){ // in this way the stopsw status stays the same upon deceleration.
		   if ((NCO (TM_GPIO_GetInputPinValue(GPIOE,GPIO_Pin_5))&&use1) || (NCO (TM_GPIO_GetInputPinValue(GPIOE,GPIO_Pin_6))&&use2)) stopsw = 1; else stopsw = 0;
		}
		if (NCO (TM_GPIO_GetInputPinValue(GPIOE,GPIO_Pin_5))&& !use1) stopswh = 7; if (NCO (TM_GPIO_GetInputPinValue(GPIOE,GPIO_Pin_6))&& !use2) stopswh = 8;}break;
		} //switch stepper
#else
		switch (stepper) {
		case 1:
		if (enable_st[1]){
		TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_9);//C4 = 1L, C5 =1R thus if use1= 1, then use2= 0 and thus C4 is normally used
		if ((stopswitched==0) || (homing==2)){ // in this way the stopsw status stays the same upon deceleration.
	       if ((NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_4))&&use1) || (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_5))&&use2)) stopsw = 1; else stopsw = 0;
		}
		if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_4))&& !use1)   stopswh = 1; if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_5))&& !use2) stopswh = 2;}break;
		case 2:
		if (enable_st[2]){
		TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_12);
		if ((stopswitched==0) || (homing==2)){ // in this way the stopsw status stays the same upon deceleration.
		   if ((NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_2))&&use1) || (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_3))&&use2)) stopsw = 1; else stopsw = 0;
		}
		if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_2))&& !use1)   stopswh = 3; if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_3))&& !use2) stopswh = 4;} break;
		case 3:
		if (enable_st[3]){
		TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_14);
		if ((stopswitched==0) || (homing==2)){ // in this way the stopsw status stays the same upon deceleration.
		   if ((NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_15))&&use1) || (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_8))&&use2)) stopsw = 1; else stopsw = 0;
		}
		if (NCO (TM_GPIO_GetInputPinValue(GPIOB,GPIO_Pin_15))&& !use1) stopswh = 5; if (NCO (TM_GPIO_GetInputPinValue(GPIOD,GPIO_Pin_8))&& !use2) stopswh = 6;}break;
		case 4:
		if (enable_st[4]){
		TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_15);
		if ((stopswitched==0) || (homing==2)){ // in this way the stopsw status stays the same upon deceleration.
		   if ((NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_1))&&use1) || (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_2))&&use2)) stopsw = 1; else stopsw = 0;
		}
		if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_1))&& !use1) stopswh = 7; if (NCO (TM_GPIO_GetInputPinValue(GPIOC,GPIO_Pin_2))&& !use2) stopswh = 8;}break;
		} //switch stepper
#endif
		//if (homing == 2){
			//   trace_printf("stopsw %i totals_steps %i step_no %i \n",stopsw, total_steps, step_no);// note, difficult construct because trace_printf cannot handle %f.
		//}
// if the commands below are used, then homming does not succeed when the other stopswitch is burning. This is more safe but irritating because operator has to turn motor by hand.
//		if ((stopswh>0) && (Stop_after_decel_stopsw==0)){// Homing hit the other stopswitch. Stop after decelerating. by checking with Stop_after_decel_stopsw it is assured that deceleration completes.
//			step_no = total_steps+1;
//			Stop_after_decel_stopsw = stopswh;
//			stopswitched = 1;
//		}
		if ((homing == 2) && (stopsw==0)) {
			not_burning++;
			if (not_burning == HOMING_STEPS) {
				step_no = total_steps+1;// done
			}
		} // not_burning should reach at least HOMING_STEPS
} else
	{// normal stepping mode, check for stopswitches burning
	if (use3D==1) {
        check_moving_steppers();
    }
    else{
    	move_stepper_phase_a(stepper);
    }// not using 3D
		//		if (use_enc[stepper]) {
//				investigate_encoder();}// one call below insufficient, missed steps..........
	}// normal stepping mode

	if ((stopsw) && (!(stopswitched))){// do not stop immediately upon stopswitch. decelerate with at maximum decel_steps
		if (step_no <= n_real_min)
		   {step_no = n_real_max + n_real_min-step_no;}
		else if (step_no < n_real_max)
		        {step_no = n_real_max;}
		if (homing==2)
			burning_steps = 0;
		   else	if (!((total_steps-step_no==1) && (homing==1))) {// if step_no==1 while homing, this means that burning_steps from previous deceleration has meaning
		           burning_steps = total_steps-step_no;
		   }

		//trace_printf("homing %i burning steps %i totals_steps %i step_no %i \n",homing,burning_steps,total_steps, step_no);// note, difficult construct because trace_printf cannot handle %f.
		TM_DISCO_LedOn(LED_ORANGE); //notice that stopswitch is on.
		//n_real_min = 0;
		//n_real_max = 1;
		stopswitched =1;
		//#if defined(USE_STOP_SWITCHES)
		//     TIM2->ARR = 5000; // stopswitch
		//#endif
	}// (stopsw) && (!(stopswitched))


	if ((step_no <= total_steps)) {
		if (step_no < n_real_min) {
			// cn_1 - (( (2*cn_1)/(4*step_no+1) ) * (n_real_min-step_no)/ n_real_min);

			//Qstep_no = (_Q16) step_no << 16;  // _Q16 from integer
			//Q4step_no = Qstep_no << 2; // 4* step_no

			//step1 = Q_one+Q4step_no;//4 * step_no + 1
			// NOTE Q15.16 RANGE IS -32768 to 32768 !!!
			step1 = fix16_from_int(1+4*step_no);//4 * step_no + 1
			//step2 = cn_1 << 1; //(2*cn_1)
			step3 = fix16_mul(cn_1,fix16_div(Q16_one,step1)) << 1; //(cn_1)/(4*step_no+1)
			//step1 =  Qn_real_min-Qstep_no;//n_real_min-step_no
			step1 =  fix16_from_int (n_real_min-step_no);//2*(n_real_min-step_no)
			step2 = fix16_mul(step1,Qn_real_min);   // (n_real_min-step_no)/n_real_min)
			cn = fix16_mul(-step3,step2);
			cn = fix16_add(cn,cn_1);
			//gul = _itofQ16(cn);
			//rul = (long) cn >> 16;
			//mul = (unsigned int) ((long) cn >> 16);
		}
		else if (step_no >= n_real_max)
		{
			//cn = cn_1 - ((2*cn_1)/(4*(step_no-total_steps)+1))   * (step_no-n_real_max) / ((total_steps-n_real_max-1)));
			//Qstep_no = (_Q16) step_no << 16;  // _Q16 from integer
			step1 = fix16_from_int(4*(step_no-total_steps) + 1);//4*(step_no-total_steps)+1)
			//step2 = cn_1 << 1; //(2*cn_1)
			step3 = fix16_mul(cn_1,fix16_div(Q16_one,step1)) << 1;//((cn_1)/(4*(step_no-total_steps)+1))
			step1 = fix16_from_int(step_no-n_real_max);//2*(step_no-n_real_max)
			step2 = fix16_mul(step1,Q_same);//(step_no-n_real_max) / ((total_steps-n_real_max-1))
			cn = fix16_mul(-step3,step2);
			cn = fix16_add(cn,cn_1);
		}
		else{ // perform latency dummy calc
			step1 = fix16_from_int(4*(step_no1-total_steps1) + 1);//4*(step_no-total_steps)+1)
			//step2 = cn_1 << 1; //(2*cn_1)
			step3 = fix16_mul(cn_1,fix16_div(Q16_one,step1)) << 1;//((cn_1)/(4*(step_no-total_steps)+1))
			step1 = fix16_from_int(step_no1-n_real_max1);//2*(step_no-n_real_max)
			step2 = fix16_mul(step1,Q_same);//(step_no-n_real_max) / ((total_steps-n_real_max-1))
			cn_1 =  fix16_mul(-step3,step2);
			cn_1 =  fix16_add(0,cn_1);
		}

		cn_1 = cn;

		mul =  ((int32) cn >> 17)* prescaler;


		//mul=mul-200;// around 200 instructions in interrupt...........
		mul--;                      //period is calculated using ARR+1
		if (mul < 1) {
			mul=1;
		}

		TIM2->ARR = mul;
		step_no++;

		//TIM2->ARR = 500;
		if (use3D==1) {
			for (int i = 1; i <= max_stepper; i++) {
				if (move_it[i]==1) {
					move_stepper_phase_b(i);
				}
			}//for
		 }// use3D
		 else{
		    	move_stepper_phase_b(stepper);
		 }// not using 3D

		//   take_step;
		//   delay(cn_1/frequency)
        		 //   tn := tn + cn_1/frequency;
		//   omega = alpha*frequency/cn_1;
		//IFS0bits.T3IF = 0; // Clear Timer1 Interrupt Flag
		//T2CONbits.TON = 1;// Start 32-bit Timer/*
	} //if ((step_no <= total_steps))
	else { // normal finish, step_no > total_steps
		status = 2;
		//TIM2->ARR = 50000;
		STOPWATCH_STOP
		TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
		//trace_printf("%i missed_steps, %i step_no, %i steps_enc, %i missed encoder\n",missed_steps, step_no, steps_enc, missed_enc);
	}
	if ((use3D==1) && (homing==0)) {
				for (int i = 1; i <= max_stepper; i++) {
					if (use_enc[i]) {
							missed_steps[i] = tell_difference_encoder(i);
							//if (step_no % 100 == 1) {
							//trace_printf("%i missed_steps, %i step_no, %i steps_enc\n",missed_steps, step_no, steps_enc);}
							if (missed_steps[i] > max_allowed_enc[i]){
								status = 5;
								slip_cause = i;// communicate the step motor that caused too much slip over the interface.
								STOPWATCH_STOP
								TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
								return;
							}// too much steps missed.
						}  // use_enc
				}//loop over all steppers
	}//use3D
	else
	if ((use_enc[stepper]) && (homing==0)) {
		missed_steps[stepper] = tell_difference_encoder(stepper);
		//if (step_no % 100 == 1) {
		//trace_printf("%i missed_steps, %i step_no, %i steps_enc\n",missed_steps, step_no, steps_enc);}
		if (missed_steps[stepper] > max_allowed_enc[stepper]){
			status = 5;
			STOPWATCH_STOP
			TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
		}// too much steps missed.
	}  // use_enc
}// take_step


void start_stepping()
{
          // timer interrupt consists of 88 instructions thus max frequency is 168000000/88=1.909E+6 (almost 2 mHz)
          //prescaler is used to limit to 32760, range of Q15.16
	  //TIM_Prescaler = N - 1; Divides the Bus/TIM clock down by N
          //TIM_Period = N - 1; Divide that clock down by N, ie the *period* is N ticks.
          //So assume your TIMCLK is ticking at 72 MHz, the *external* clock
          //If TIM_Prescaler = 71; the timers *internal* clock will be 1 MHz
          //If TIM_Period = 999; the 1MHz is divided by a 1000, so it becomes 1KHz
          //Your update interrupt with fire every 1ms
          //Now if you want to play with the channels, say in a PWM mode you can play with the output pins.
          //If TIM_Pulse = 500; the channel will out a frequency of 1 KHz, with a 50/50 duty cycle. ie TIM_Pulse = N / 2
          //suppose timer runs at 168000 Hz, prescaler 1000, set in main.c InitializeTimer(999,500);


	      *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
	      *DWT_CYCCNT = 0; // reset the counter
	      *DWT_CONTROL = *DWT_CONTROL | 1 ; // enable the counter


	      omega_max = 2*PI*rpm[stepper]/60;
	      omega_acc =  omega_max/on_speed[stepper];
	      omega_decc = omega_acc;
	      lulgol = 2.0 / steps_per_rpm[stepper];
	      alpha = PI*lulgol;
	      frequency = 168000.0;

	      for (int i = 0; i <= 7; i++) {
	    	  switch (i) {
	    	  case 0:prescaler =1;break;
	    	  case 1:prescaler =2;break;
	    	  case 2:prescaler =4;break;
	    	  case 3:prescaler =8;break;
	    	  case 4:prescaler =16;break;
	    	  case 5:prescaler =32;break;
	    	  case 6:prescaler =64;break;
	    	  case 7:prescaler =128;break;
	    	  }//switch
	    	  lulgol = 0.676*(frequency/prescaler)*sqrt(2*alpha/omega_acc);
	    	  if (lulgol < 32760.0){
	    	           break;}
	          }
          frequency = 168000/prescaler;

          check_cn();
          Stop_after_decel_stopsw = 0;
          slip_cause = stepper;
          short thedir;
          if (use3D==1) {
//        	  strcpy(dummystr,"35:<");
    	      for (int i = 1; i <= max_stepper; i++) {
    	      stepper_steps[i] = 0;
    	      if (use_enc[i]) {initialize_encoder(i);}
    	      prev_action[i] = 0;
        	  thedir = direction[i];
        	            //trace_printf("thedir %i\n",thedir);
        	            if ((thedir) && (sign_total_steps[i])) {thedir=0;}
        	            else if ((!(thedir)) && (sign_total_steps[i])) {thedir=1;}
        	                         //trace_printf("thedir %i\n",thedir);
        	                         //trace_printf("diriction stepper %i\n",direction[stepper]);
        	                         //trace_printf("sign_total_steps %i\n",sign_total_steps);
//        	  strcat(dummystr,itoa(thedir,dummystr2,10) );
        	  #if !defined (STM32F411xE)
        	            switch (i) {
        	               case 1: if (enable_st[1]) {TM_GPIO_SetPinLow(GPIOA,GPIO_Pin_1);if (thedir) TM_GPIO_SetPinHigh(GPIOA,GPIO_Pin_2); else TM_GPIO_SetPinLow(GPIOA,GPIO_Pin_2);} break; // enable + stepper direction
        	               case 2: if (enable_st[2]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_10);if (thedir) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_11); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_11);} break;
        	               case 3: if (enable_st[3]) {TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_12);if (thedir) TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_13); else TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_13);} break;
        	               case 4: if (enable_st[4]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_7);if (thedir) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_8); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_8);} break;
        	            }
        	  #else
        	            switch (i) {
        	               case 1: if (enable_st[1]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_7);if (thedir) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_8); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_8);} break; // enable + stepper direction
        	               case 2: if (enable_st[2]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_10);if (thedir) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_11); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_11);} break;
        	               case 3: if (enable_st[3]) {TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_12);if (thedir) TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_13); else TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_13);} break;
        	               case 4: if (enable_st[4]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_13);if (thedir) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_14); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_14);} break;
        	            }
        	  #endif
    	      }//for
//    	      strcat(dummystr,">#");
//    	      #if defined (USE_USB)
//    	      	   TM_USB_VCP_Puts(dummystr);
//			  #endif
          } //use3d
          else {
          thedir = direction[stepper];
          if (use_enc[stepper]) {initialize_encoder(stepper);}
          prev_action[stepper] = 0;
          stepper_steps[stepper] = 0;
          //trace_printf("thedir %i\n",thedir);
          if ((thedir) && (sign_total_steps[stepper])) {thedir=0;}
          else if ((!(thedir)) && (sign_total_steps[stepper])) {thedir=1;}
                       //trace_printf("thedir %i\n",thedir);
                       //trace_printf("diriction stepper %i\n",direction[stepper]);
                       //trace_printf("sign_total_steps %i\n",sign_total_steps);
          if (homing==1){// when homing, reverse direction.
        	 if (direction[stepper]) {thedir =0;}
        	 else thedir=1;
        	 homing_steps=0;
          }
#if !defined (STM32F411xE)
          switch (stepper) {
             case 1: if (enable_st[1]) {TM_GPIO_SetPinLow(GPIOA,GPIO_Pin_1);if (thedir) TM_GPIO_SetPinHigh(GPIOA,GPIO_Pin_2); else TM_GPIO_SetPinLow(GPIOA,GPIO_Pin_2);} break; // enable + stepper direction
             case 2: if (enable_st[2]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_10);if (thedir) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_11); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_11);} break;
             case 3: if (enable_st[3]) {TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_12);if (thedir) TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_13); else TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_13);} break;
             case 4: if (enable_st[4]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_7);if (thedir) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_8); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_8);} break;
          }
#else
          switch (stepper) {
             case 1: if (enable_st[1]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_7);if (thedir) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_8); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_8);} break; // enable + stepper direction
             case 2: if (enable_st[2]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_10);if (thedir) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_11); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_11);} break;
             case 3: if (enable_st[3]) {TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_12);if (thedir) TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_13); else TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_13);} break;
             case 4: if (enable_st[4]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_13);if (thedir) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_14); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_14);} break;
          }
#endif
          }// not using 3D
           mul =  ((int32) cn >> 17)* prescaler;
           mul--;        //period is calculated using ARR+1
          //mul=mul-200;// around 200 instructions in interrupt...........
           if (mul < 1) {
               mul=1;
           }

           TIM2->ARR = mul;
           //TIM2->ARR = 500;
           //TIM_SetAutoreload(TIM2,10000);



#if defined(DEBUG_OUTPUT)
           if (total_steps==0){
           trace_printf("total_steps: %i\n", total_steps);
           }
#endif


           TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
        /* Start the Timer */
           STOPWATCH_START

}





void end_stepping(int switch_off, int emergency_stop)
// note, this routine always puts a string to the server.
{
          TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
          double kul;
          //char kulstring[50];
          kul = cyc[1]/168000000.0;
          int d1 = kul;            // Get the integer part (678).
          int d2 = trunc((kul-d1) * 10000);   // Get fractional part (0.01234567) and Turn into integer (123).

          //trace_printf("werk %d.%04d seconds\n",d1,d2);// note, difficult construct because trace_printf cannot handle %f.
          if (use3D==1){
        	  if (switch_off){
        		  for (int i = 1; i <= max_stepper; i++) {
#if !defined (STM32F411xE)
        			  switch (i) {
        			  case 1: if (enable_st[1]) {TM_GPIO_SetPinHigh(GPIOA,GPIO_Pin_1);} break;//disable
        			  case 2: if (enable_st[2]) {TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_10);} break;
        			  case 3: if (enable_st[3]) {TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_12);} break;
        			  case 4: if (enable_st[4]) {TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_7);} break;
        			  }
#else
        			  switch (i) {
        			  case 1: if (enable_st[1]) {TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_7);} break;//disable
        			  case 2: if (enable_st[2]) {TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_10);} break;
        			  case 3: if (enable_st[3]) {TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_12);} break;
        			  case 4: if (enable_st[4]) {TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_13);} break;
        			  }
#endif
        		  }//for
        	  }	// switch off
          }
          else{
        	  if (switch_off){
#if !defined (STM32F411xE)
        		  switch (stepper) {
        		  case 1: if (enable_st[1]) {TM_GPIO_SetPinHigh(GPIOA,GPIO_Pin_1);} break;//disable
        		  case 2: if (enable_st[2]) {TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_10);} break;
        		  case 3: if (enable_st[3]) {TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_12);} break;
        		  case 4: if (enable_st[4]) {TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_7);} break;
        		  }
#else
        		  switch (stepper) {
        		  case 1: if (enable_st[1]) {TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_7);} break;//disable
        		  case 2: if (enable_st[2]) {TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_10);} break;
        		  case 3: if (enable_st[3]) {TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_12);} break;
        		  case 4: if (enable_st[4]) {TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_13);} break;
        		  }
#endif
        	  }
          }// not using 3D
          if (emergency_stop==1) {
        	  strcpy(dummystr,"<8:y>");
        	  TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
          }
          else {// no emergency stop
          if ((stopswitched) && !(homing==0)) { // stopswitch was on, this is no immediate stop. Decelerate to startposition.
             if ((stopsw) && (homing<2)) { // reverse movement decel_steps to go to startposition.
#if defined(DEBUG_OUTPUT)
        	                                    trace_printf("ending decelerating: %i\n", command_id);
#endif
                config_read[stepper]=all_read;
                total_steps = burning_steps + HOMING_STEPS; // note + 20 steps.
                check_cn();// NOTE TODOOOOOOOOOOOO
                //stopswitched = 0; do not do this, the stopswitch is burning, stopswitched is one already, otherwise would lead to immediate stop.
                status = 1;
                not_burning=0; // when homing and stopswitch switches off, number of not burning steps should reach 20.

                homing++; //only do this one time !
#if !defined (STM32F411xE)
                switch (stepper) {
                  case 1: if (enable_st[1]) {TM_GPIO_SetPinLow(GPIOA,GPIO_Pin_1);if (direction[stepper]) TM_GPIO_SetPinHigh(GPIOA,GPIO_Pin_2); else TM_GPIO_SetPinLow(GPIOA,GPIO_Pin_2);} break; // enable + stepper direction
                  case 2: if (enable_st[2]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_10);if (direction[stepper]) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_11); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_11);} break;
                  case 3: if (enable_st[3]) {TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_12);if (direction[stepper]) TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_13); else TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_13);} break;
                  case 4: if (enable_st[4]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_7);if (direction[stepper]) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_8); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_8);} break;
                }
#else
                switch (stepper) {
                  case 1: if (enable_st[1]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_7);if (direction[stepper]) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_8); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_8);} break; // enable + stepper direction
                  case 2: if (enable_st[2]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_10);if (direction[stepper]) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_11); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_11);} break;
                  case 3: if (enable_st[3]) {TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_12);if (direction[stepper]) TM_GPIO_SetPinHigh(GPIOB,GPIO_Pin_13); else TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_13);} break;
                  case 4: if (enable_st[4]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_13);if (direction[stepper]) TM_GPIO_SetPinHigh(GPIOE,GPIO_Pin_14); else TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_14);} break;
                }
#endif
                strcpy(dummystr,"<wyhoming>");
                //trace_printf("wyhoming % s \n","kul");
                TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
             }
             else if ((Stop_after_decel_stopsw==0) && (!(not_burning==HOMING_STEPS) && (homing==2))) { // reverse movement decel_steps to go to startposition. stopswitch is not off yet, direction is OK
#if defined(DEBUG_OUTPUT)
        	                                    trace_printf("ending reverse: %u\n", command_id);
#endif
            	 // do this until not_burning== HOMING_STEPS or user presses stop.
                 	 config_read[stepper]=all_read;
            	     total_steps = HOMING_STEPS; // note + 20 steps.
            	     check_cn();// NOTE TODOOOOOOOOOOOO
            	     //stopswitched = 0; do not do this, the stopswitch is burning, stopswitched is one already, otherwise would lead to immediate stop.
            	     status = 1;
#if !defined (STM32F411xE)
            	     switch (stepper) {
            	                       case 1: if (enable_st[1]) {TM_GPIO_SetPinLow(GPIOA,GPIO_Pin_1);}break; // enable
            	                       case 2: if (enable_st[2]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_10);}break;
            	                       case 3: if (enable_st[3]) {TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_12);}break;
            	                       case 4: if (enable_st[4]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_7);}break;
            	                     }
#else
            	     switch (stepper) {
          	                 	       case 1: if (enable_st[1]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_7);}break; // enable
            	                 	   case 2: if (enable_st[2]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_10);}break;
            	                 	   case 3: if (enable_st[3]) {TM_GPIO_SetPinLow(GPIOB,GPIO_Pin_12);}break;
            	                 	   case 4: if (enable_st[4]) {TM_GPIO_SetPinLow(GPIOE,GPIO_Pin_13);}break;
            	                 	  }
#endif
            	     strcpy(dummystr,"<wyhoming>#");
            	     //trace_printf("wyhoming % s \n","kul");
            	     TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
             }
             else {// now really done, stopswh is probably off after homing communicate in form
                //sprintf(dummystr,"<wydone:%i;%i>",stopswh,pending_commands); // convert back to validate result.
#if defined(DEBUG_OUTPUT)
        	                                    trace_printf("ending homing: %u\n", command_id);
#endif
            	 TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
            	 status = 0;
            	 if (Stop_after_decel_stopsw>0){
            	    no_commands = 0; // because stopswitch was burning, now do not perform the commands that are still in the buffer.
            	 }
                 itoa(Stop_after_decel_stopsw,dummystr2,10);
                 strcpy(dummystr,"<wydone:");
                 strcat(dummystr,dummystr2);
                 strcat(dummystr,";");
                 itoa(no_commands,dummystr2,10);
                 strcat(dummystr,dummystr2);
                 strcat(dummystr,"@0");
                 itoa(homing_steps,dummystr2,10);
                 strcat(dummystr,dummystr2);
                 strcat(dummystr,";0>");
                 if (command_id != -65535){
                    itoa(command_id,dummystr2,10);
                    strcat(dummystr,dummystr2);
                 }
                 strcat(dummystr,"&#");
                 //strcat(dummystr,">");
             }
          }  else {  //normal stepping ended succesfully, not homing. Stopswitch might be burning
                //sprintf(dummystr,"<wydone:%i;%i>",stopswh,pending_commands); // convert back to validate result.
#if defined(DEBUG_OUTPUT)
        	                                    trace_printf("ending normally: %u\n", command_id);
#endif
        	    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);

                itoa(stopsw,dummystr2,10);
                if (stopsw!=0) {
                	no_commands=0;// empty buffer because stopswitch is burning, otherwise another axis continues
                }
                strcpy(dummystr,"<wydone:");
                strcat(dummystr,dummystr2);
                strcat(dummystr,";");
                itoa(no_commands,dummystr2,10);
                strcat(dummystr,dummystr2);
                strcat(dummystr,"@");
                if (status==5) {
                   itoa(slip_cause,dummystr2,10);
                   no_commands=0;
                } //missed too many steps
                else {itoa(0,dummystr2,10);}
                strcat(dummystr,dummystr2);
                itoa(missed_steps[slip_cause]/4,dummystr2,10);
                strcat(dummystr,dummystr2);
                strcat(dummystr,";");
                itoa(missed_enc[slip_cause]/4,dummystr2,10);//undefined status change
                strcat(dummystr,dummystr2);
                strcat(dummystr,">");
                if (command_id != -65535){
                   itoa(command_id,dummystr2,10);
                   strcat(dummystr,dummystr2);
                }
                strcat(dummystr,"&#");
      	        for (int i = 1; i <= max_stepper; i++) {
      	          itoa(stepper_steps[i],dummystr2,10);
      	          strcat(dummystr,dummystr2);
      	          strcat(dummystr,"?");
      	        }


                status = 0;
                step_no= 1;
          }   // finished without burning stopswitch
          }// no emergency_stop

#if defined(DEBUG_OUTPUT)
                 trace_printf("%s \n",dummystr);
#endif

#if defined (USE_USB)
            	 TM_USB_VCP_Puts(dummystr);                //include w an y for all possible commands.
#else
            	 //TM_USART_DMA_Send(USART2, (uint8_t *)dummystr, strlen(dummystr));
            	 //TM_USART_Send(USART2, (uint8_t *)dummystr, strlen(dummystr));
#if !defined (STM32F411xE)
            	 TM_USART_Puts(USART3,dummystr);
#else
              	 TM_USART_Puts(USART2,dummystr);
#endif
#endif
}// end_stepping



