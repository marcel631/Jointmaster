/*
 * globals.h
 *
 *  Created on: May 31, 2015
 *      Author: SMI
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include <stdio.h>
#include <stdlib.h>
#include "fixmath.h"

#define APP_READ_BUFFER_SIZE 64
#define APP_WRITE_BUFFER_SIZE 64
#define STLN 64

#define PI 3.1415926535897932384626433832795
#define HOMING_STEPS 20 // number of steps the stopswitch should be off
#define all_read 36
#define Maxint 2147483647  //maximum unsigned int 32 bits
#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define max_stepper 4
#define config_init 240 //208//80 //64
#define max_command 1024

typedef fix16_t _Q16; //15 bits decimal, 16 bits behind comma thus Q15.16 = signed 32 bits fixed point. NOTE Q15.16 RANGE IS -32768 to 32768 !!!
typedef int32_t int32;
typedef int16_t int16;
typedef uint16_t uint16;

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************
// way to use ccmram  __attribute__ ((section (".ccm")))= {....};
extern char dummystr2[APP_READ_BUFFER_SIZE];
extern char dummystr3[APP_READ_BUFFER_SIZE];
extern char dummystr[APP_READ_BUFFER_SIZE];

extern char Command_data[STLN];
extern char Saved_data[STLN];
extern unsigned char unsigned_data[STLN];
extern short move_it[max_stepper+1];
extern double alpha ;
extern double omega_acc ;
extern double omega_max ;
extern double omega_decc ;
extern double frequency ;
extern int32 homing_steps ;
extern unsigned int interrupt_error ;
extern int32 step_no ;
extern int32 stepper_steps[max_stepper+1] ;
extern int32 enable_st[max_stepper+1] ;// number of steps to decelerate upon stopswitch
extern int32 total_steps ;
extern int32 paused_steps ;
extern int32 total_steps1 ;// only used for latency dummy calc.
extern int32 n_real_max1 ;// only used for latency dummy calc.
extern int32 step_no1 ;// only used for latency dummy calc.
extern int32 Q16_one ;
extern unsigned int enable ;
extern unsigned int status ;
extern short homing ;
extern short sign_total_steps[max_stepper+1] ;
extern short sign_encoder_ratio ;
extern int32 no_commands ;
extern short neglect ; //stopswitch to neglect when homing is 1
extern short use1 ; //when homing, use stopswitch 1 signal
extern short use2 ; //when homing use stopswitch 2 signal
extern short use3D ; // when moving 3D, this is 1, else 0
extern unsigned int direction[max_stepper+1] ; //CW or CCW
extern short stopsw ;
extern short slip_cause ; // step motor that caused maximum encoder slip.
extern short stopswh ;
extern short Stop_after_decel_stopsw ;
extern short stopswitched ;
extern short stepper ;
extern short c_stepper ;
extern short paused_stepper ;
extern int32 n_decc ;
extern int32 n_acc ;
extern int32 n_acc_decc ;
extern int32 n_real_min ;
extern int32 n_real_max ;
extern _Q16 cn ;
extern _Q16 cn_1 ;
extern _Q16 Qn_real_min ;
extern _Q16 Q_same ;
extern _Q16 step1 ;
extern _Q16 step2 ;
extern _Q16 step3 ;
extern _Q16 Q_one ;
extern int prescaler ;
extern int succes ;
extern int config_read[max_stepper+1] ;
extern int32 total_steps_array[max_stepper+1] ;
extern int32 total_steps_d2_array[max_stepper+1] ;
extern int32 err_array[max_stepper+1] ;
extern int32 mul ;
extern double  n_deccd ;
extern double  n_accd ;
extern double rpm[max_stepper+1] ;
extern double rpm_max[max_stepper+1] ;
extern double rpm_feed[max_stepper+1] ;
extern int32 steps_per_rpm[max_stepper+1] ;
extern int32 jpos ;
extern int32 jpos_s ;
extern double on_speed[max_stepper+1] ;

// encoder variables
extern short ___diff ;
extern short ___new_status ;
extern short old_status[max_stepper+1] ;
extern short prev_action[max_stepper+1] ;
extern short paused ;
extern short use_enc[max_stepper+1] ;  // As long as this number stays zero, no check for encoder is performed for this stepper
extern int32 steps_enc[max_stepper+1] ; // number of steps counted on encoder
extern int32 burning_steps ; // steps decelerating with burning stopswitch
extern int32 not_burning ; // when homing and stopswitch switches off, number of not burning steps should reach HOMING_STEPS
extern int32 missed_enc[max_stepper+1] ; // number of undefined steps on encoder
extern int32 max_allowed_enc[max_stepper+1] ;// maximum allowed number of missed steps
extern int32 missed_steps[max_stepper+1] ; //missed steps
extern _Q16 step_enc_ratio[max_stepper+1] ;//= steps_mm/steps_mm_encoder, always around 1 in Q15/16 format
extern short A_val ;
extern short B_val ;
extern int64_t kul64 ;
extern int32 kul32 ;
extern int32 first_command ;
extern int32 command_id ;
extern int32 saved_id ; // current command id and saved command id for buffered commands
extern short check_packet_loss ;
extern uint32_t wait_millis ;
extern uint32_t wait_timer ;






void check_command(char *dumstr, char achar, int *succs, char *dumstr2, short overwrite_command_id);
int check_all_read(int jpos_size);
void InitializeTimer(uint16_t prescaler,uint32_t period);
void EnableTimerInterrupt();
void start_stepping();
void end_stepping(int switch_off, int emergency_stop);
void take_step ();
void enable_disable(short astepper,unsigned int aenable);
void initialize_encoder(short astepper);
void investigate_encoder(short astepper);
int tell_difference_encoder(short astepper);




#endif /* GLOBALS_H_ */
