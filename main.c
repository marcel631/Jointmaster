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

#include "defines.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "diag/Trace.h"
#include "rambuffer.h"

#include "tm_stm32f4_delay.h"
#if defined (USE_USB)
#include "tm_stm32f4_usb_vcp.h"
#else
#include "tm_stm32f4_usart.h"
//#include "tm_stm32f4_usart_dma.h"
#endif
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_pwm.h"
#include "tm_stm32f4_watchdog.h"
#include "stm32f4xx_dbgmcu.h"
#include "fixmath.h"
#include "globals.h"
#include "CRC_CCITT.h"
#include "usb_defines.h"


// ----------------------------------------------------------------------------
//
// Semihosting STM32F4 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// To demonstrate semihosting, display a message on the standard output
// and another message on the standard error.
//
// Then demonstrates how to blink a led with 1Hz, using a
// continuous loop and SysTick delays.
//
// On DEBUG, the uptime in seconds is also displayed on the trace device.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_ITM, OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//


// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"
short green_on;
int32 paused_perc;
#if defined (USE_USB)
TM_USB_VCP_Result USB_result;
#endif
int8_t char_avail;



//float lulgol;


void write_timer_done(){
	 strcpy(dummystr,"<wydone:0;0@00;0>");
	 if (command_id != -65535){
	    itoa(command_id,dummystr2,10);
	    strcat(dummystr,dummystr2);
	 }
	 strcat(dummystr,"&#");
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
}



void TIM2_IRQHandler()
 {
     if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
     {
	 take_step();
	 TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
     }
 } //TIM2_IRQHandler


void init8y() {
    jpos=0;
    jpos_s = -3;
    paused = 0;
    paused_steps = 0;
    homing = 0;
    stopswitched = 0;
    stopsw = 0;
    step_no = total_steps+1;
    wait_timer = 0;
    check_packet_loss=0;
    no_commands = 0;
    first_command = 0;
    config_read[stepper] = config_init; // note, when there is user interaction then complete new program is necessary.
    status = 0;
}


int read_char(uint8_t c)
// read chars when status = 0. Not stepping
{
	                  if (((c == (uint8_t)('<')) && (jpos_s < 0)) || (jpos==STLN-1) || ((jpos_s > 0) && (jpos > jpos_s + 2)))  { // start of new command, jpos_s must be less than zero because '<' char can also be crc code
                        jpos=0;
                        jpos_s = -3;
                     }
                     unsigned_data[jpos]= (unsigned char) c;
                     char_avail = 0;
                     if (jpos_s < 0) {
                       Command_data[jpos] = (char) c;   // always reading just one char !
                       if (Command_data[jpos] == '>')  {// new command available
                          Command_data[jpos+1]='\0';
                          jpos_s = jpos; // crc code is always 2 bytes after '>' char, not within '>' because crc code could be '>'.........
                       }
                     } //jpos_s < 0
                     if (jpos == jpos_s + 2) {
#if defined(DEBUG_SR)
                        trace_printf("received: %s \n",Command_data);
#endif
                        if (check_all_read(jpos)) {         // check_all_read always writes back te latest command. Thus waitstate until command is written to host.
                        	jpos=0;//succesful command.
                        	jpos_s = -3;
                           return 1;
                           }
                        else {//nonsense command or crc wrong.
                        	jpos = 0;
                        	jpos_s = -3;
                           return 0;
                           }
                     } // new command available
                     else { //no command yet, advance, echo the read char and try read next char
                           jpos++;
                           //TM_USB_VCP_Putc(c);
                           return 0;
                     }
} // read_char


int
main(int argc, char* argv[])
{
  // Show the program parameters (passed via semihosting).
  // Output is via the semihosting output channel.
  //trace_dump_args(argc, argv);

  // Send a greeting to the trace device (skipped on Release).
  //trace_puts("Hello ARM World!");

  // Send a message to the standard output.
  //puts("Standard output message.");

  // Send a message to the standard error.
  //fprintf(stderr, "Standard error message.\n");

  // At this stage the system clock should have already been configured
  // at high speed.
#if defined(DEBUG_OUTPUT)
  trace_printf("System clock: %uHz\n", SystemCoreClock);
#endif

  CalculateTable_CRC16();// calculate crc lookup table


//#define LOOP_COUNT (3)
//  int loops = LOOP_COUNT;
  if (argc > 1)
    {
      // If defined, get the number of loops from the command line,
      // configurable via semihosting.
//      loops = atoi (argv[1]);
    }


  uint8_t c;
  TM_PWM_TIM_t TIM4_Data;
      /* System Init */
      SystemInit();

      /* Initialize LED's. Make sure to check settings for your board in tm_stm32f4_disco.h file */
      TM_DISCO_LedInit();

      TM_DELAY_Init();

      /* Set PWM to 1kHz frequency on timer TIM2 */
         /* 1 kHz = 1ms = 1000us */
//#if !defined (STM32F411xE)
      TM_PWM_InitTimer(TIM4, &TIM4_Data, 1);
//#endif

// stepper 1 output// note, driving push pull so that defined voltages are at the output. format enable, direction pulse
#if !defined (STM32F411xE)
      TM_GPIO_Init(GPIOA, GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
#else
      TM_GPIO_Init(GPIOE, GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
#endif
      // stepper 1 input
      if (NCO(1)==1) // normally closed stopswitches, in fault condition a zero, opto coupler is not conducting, leading to high value on discovery input (pull up resistor),
    	  //thus also pull up internal discovery pull-up, so that in case of bad connection, there is a fault.
         TM_GPIO_Init(GPIOC, GPIO_Pin_4 | GPIO_Pin_5, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
      else // normally open stopswitches, in fault condition a one, opto coupler is conducting, leading to low value on discovery input,
    	  //thus also pull down internal discovery pull-up, so that in case of bad connection, there is a fault.
    	  TM_GPIO_Init(GPIOC, GPIO_Pin_4 | GPIO_Pin_5, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_High);

      // stepper 2 output
      TM_GPIO_Init(GPIOE, GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
      // stepper 2 input
      //TM_GPIO_Init(GPIOE, GPIO_Pin_13 | GPIO_Pin_14, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
#if !defined (STM32F411xE)
      if (NCO(1)==1) // see stepper 1 for explanation
         TM_GPIO_Init(GPIOB, GPIO_Pin_4 | GPIO_Pin_5, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
      else TM_GPIO_Init(GPIOB, GPIO_Pin_4 | GPIO_Pin_5, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_High);
#else
      if (NCO(1)==1) // see stepper 1 for explanation
               TM_GPIO_Init(GPIOD, GPIO_Pin_2 | GPIO_Pin_3, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
      else TM_GPIO_Init(GPIOD, GPIO_Pin_2 | GPIO_Pin_3, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_High);
#endif
      // stepper 3 output
      TM_GPIO_Init(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
      // stepper 3 input
      if (NCO(1)==1) { // // see stepper 1 for explanation
        TM_GPIO_Init(GPIOB, GPIO_Pin_15, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
        TM_GPIO_Init(GPIOD, GPIO_Pin_8, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
      } else
      {
    	  TM_GPIO_Init(GPIOB, GPIO_Pin_15, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_High);
    	  TM_GPIO_Init(GPIOD, GPIO_Pin_8, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_High);
      }

      // stepper 4 output// note, driving push pull so that defined voltages are at the output.
      //TM_GPIO_Init(GPIOE, GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
#if !defined (STM32F411xE)
      TM_GPIO_Init(GPIOE, GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
      // stepper 4 input
      if (NCO(1)==1)  // see stepper 1 for explanation
         TM_GPIO_Init(GPIOE, GPIO_Pin_5 | GPIO_Pin_6, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
      else TM_GPIO_Init(GPIOE, GPIO_Pin_5 | GPIO_Pin_6, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_High);
#else
      TM_GPIO_Init(GPIOE, GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, TM_GPIO_Mode_OUT, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
            // stepper 4 input
            if (NCO(1)==1)  // see stepper 1 for explanation
               TM_GPIO_Init(GPIOC, GPIO_Pin_1 | GPIO_Pin_2, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_UP, TM_GPIO_Speed_High);
            else TM_GPIO_Init(GPIOC, GPIO_Pin_1 | GPIO_Pin_2, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_DOWN, TM_GPIO_Speed_High);
#endif

      //encoder 1 input
      TM_GPIO_Init(GPIOC, GPIO_Pin_6, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);// no pull up/down, step motor board has own pull up/downm
      TM_GPIO_Init(GPIOC, GPIO_Pin_7, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
      //encoder 2 input
#if !defined (STM32F411xE)
      TM_GPIO_Init(GPIOC, GPIO_Pin_8, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
      TM_GPIO_Init(GPIOC, GPIO_Pin_9, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
#else
      TM_GPIO_Init(GPIOD, GPIO_Pin_10, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
      TM_GPIO_Init(GPIOD, GPIO_Pin_11, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
#endif

      //encoder 3 input
      TM_GPIO_Init(GPIOD, GPIO_Pin_1, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
      TM_GPIO_Init(GPIOD, GPIO_Pin_0, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
      //encoder 4 input
#if !defined (STM32F411xE)
      TM_GPIO_Init(GPIOD, GPIO_Pin_3, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
      TM_GPIO_Init(GPIOD, GPIO_Pin_2, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
#else
      TM_GPIO_Init(GPIOA, GPIO_Pin_8, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
      TM_GPIO_Init(GPIOC, GPIO_Pin_9, TM_GPIO_Mode_IN, TM_GPIO_OType_PP, TM_GPIO_PuPd_NOPULL, TM_GPIO_Speed_High);
#endif


      enable_disable(1,0);
      enable_disable(2,0);
      enable_disable(3,0);
      enable_disable(4,0);

      EnableTimerInterrupt();
      InitializeTimer(1000,50000);
      /* Disable watchdog when we are in debug mode */
      DBGMCU->APB1FZ |= DBGMCU_IWDG_STOP;
          /* Initialize watchdog timer */
             /* Set timeout to 1s */
             /* If we are in debug mode, watchdog won't start even if we enable it */

      TM_GPIO_TogglePinValue(GPIOD, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);
      if (TM_WATCHDOG_Init(TM_WATCHDOG_Timeout_1s)) {
                 /* System was reset by watchdog */
          TM_DISCO_LedOn(LED_RED);
          TM_DISCO_LedOff(LED_GREEN);
          green_on =0;
      } else {
                 /* System was not reset by watchdog */
          TM_DISCO_LedOn(LED_GREEN);
          TM_DISCO_LedOff(LED_RED);
          green_on =1;
      }

      TM_DISCO_LedOff(LED_ORANGE);

      // Short loop.
//        for (int i = 0; i < loops; i++)
//          {
//            TM_DISCO_LedOn(LED_GREEN);
//            Delayms(500);
//            TM_DISCO_LedOff(LED_GREEN);
//            Delayms(500);
//            ++seconds;
            // Count seconds on the trace device.
//            trace_printf("Second %u\n", seconds);
//          }


#if defined (USE_USB)
      /* Initialize USB VCP */
      TM_USB_VCP_Init();
#else
      /* This pins are used on Nucleo boards for USB to UART via ST-Link */
      /* Init USART2 on pins TX = PA2, RX = PA3  for F411 (connect RX to HC05 TX and TX to HC05 RX for successfull communication)*/
      /* Init USART 3 on pins TX= PB10 and RX = PB11 for F407 */
#if !defined (STM32F411xE)
      TM_USART_Init(USART3, TM_USART_PinsPack_1, 9600);
#else
          TM_USART_Init(USART2, TM_USART_PinsPack_1, 9600);
#endif
          /* Init TX DMA for USART2 */
//          TM_USART_DMA_Init(USART2);
#endif

//#if !defined (STM32F411xE)
          /* Initialize PWM on TIM2, Channel 1 and PinsPack 2 = PA5 */
      TM_PWM_InitChannel(&TIM4_Data, TM_PWM_Channel_4, TM_PWM_PinsPack_2);

          /* Set channel 1 value, 500us pulse high = 500 / 1000 = 0.5 = 50% duty cycle */
      TM_PWM_SetChannelMicros(&TIM4_Data, TM_PWM_Channel_4, 5000);
//#endif


       //TIM_Prescaler = N - 1; Divides the Bus/TIM clock down by N
      //TIM_Period = N - 1; Divide that clock down by N, ie the *period* is N ticks.
      //So assume your TIMCLK is ticking at 72 MHz, the *external* clock
      //If TIM_Prescaler = 71; the timers *internal* clock will be 1 MHz
      //If TIM_Period = 999; the 1MHz is divided by a 1000, so it becomes 1KHz
      //Your update interrupt with fire every 1ms
      //Now if you want to play with the channels, say in a PWM mode you can play with the output pins.
      //If TIM_Pulse = 500; the channel will out a frequency of 1 KHz, with a 50/50 duty cycle. ie TIM_Pulse = N / 2
      //suppose timer runs at 168000 Hz, prescaler 1000, set in main.c InitializeTimer(999,500);



      status=0;
      no_commands=0;
      paused=0;
      paused_steps=0;
      char_avail=0;


      jpos = 0;
      jpos_s = -3;

      for (int i = 1; i <= max_stepper; i++)
           {
             config_read[i] = config_init;
             use_enc[i]=0;
             rpm[i]=60.0; rpm_max[i]=60.0; rpm_feed[i]=60.0;
             enable_st[i]=1;
             max_allowed_enc[i]=5;//arbitrary value........
             step_enc_ratio[i] = 1 <<16;
             prev_action[i]=0;
             stepper_steps[i]=0;
             old_status[i]=0;
           }

      TIM2->ARR = 10000;
      wait_timer = 0;


      while (1) {
    	  TM_WATCHDOG_Reset();  // periodically reset the watchdog
    	  /* USB configured OK, drivers OK */
    	  if (((status==2) || (status==5)) && (paused_steps <= 0)) {
    	  // finished set by timer23 interrupt, sync on commands, status = 2 means step_no > total_steps, status = 5, to many missed encoder steps
    	  // status = 1, ready to start stepping.
    	                     config_read[stepper] = config_init;
    	  #if defined(DEBUG_OUTPUT)
    	          	                                    trace_printf("ending command (main.c): %i\n", command_id);
    	  #endif
    	                     end_stepping(0,0); // end_stepping writes message including 'w and sets status=0
    	                }
    	  if ((paused==0) && (paused_steps > 0)) {// status is probably 2 or 5
    	      total_steps = paused_steps;
    	      paused_steps = 0;
    	      status=1;
    	      start_stepping();
    	  }
#if defined (USE_USB)
          if (TM_USB_VCP_GetStatus() == TM_USB_VCP_CONNECTED) {
              /* Turn on GREEN led */
        	  if (!green_on){
        	  green_on =1;
              TM_DISCO_LedOn(LED_GREEN);
              TM_DISCO_LedOff(LED_RED);
        	  }
              /* If something arrived at VCP */
        	  USB_result = TM_USB_VCP_Getc(&c);
        	  char_avail = (USB_result == TM_USB_VCP_DATA_OK);
#else
#if !defined (STM32F411xE)
               char_avail = TM_USART_Getc(USART3,&c);
#else
               char_avail = TM_USART_Getc(USART2,&c);
#endif
#endif
              if ((char_avail) || ((no_commands > 0) && (status==0))) {
            	  if ((status==0) && (wait_timer == 0) && (paused==0)) {// status == 0 when not in step interrupt
        	                       if (no_commands > 0) {// command in buffer
                                             strcpy(Saved_data,Command_data);// while copying a saved command, there might be a new command send to stm32. Thus save it temporarily
        	                    	         strcpy(Command_data,"<");
        	                    	         strcat(Command_data,Command_buffer[1]);
        	                    	         strcat(Command_data,">");
        	                    	         for (int i = 1; i <= no_commands-1; i++)
        	                    	         {strcpy(Command_buffer[i],Command_buffer[i+1]);}
        	                    	         no_commands--;
        	                    		     status = check_all_read(0);// no need to check crc again, buffered commando's have been checked.
        	                    		     strcpy(Command_data,Saved_data);
        	                    		     if (status==1) {
        	                    		    	 start_stepping();
        	                    		     }// status == 1 , new step program
        	                    	}// command in buffer
        	                    	if ((char_avail) && (status==0)) {// data on USB, note there might have been a command in buffer that set status=1
        	                	            read_next: status = read_char(c);// not a complete stepping program yet.
        	                                if (status == 1) {  // stepping program complete
        	                                    start_stepping();
#if defined(DEBUG_OUTPUT)
    #if defined (USE_USB)
        	                                    TM_USB_VCP_Settings_t Settings;
        	                                    TM_USB_VCP_GetSettings(&Settings);
        	                                    trace_printf("baudrate: %u\n", Settings.Baudrate);
    #endif
#endif
        	                                }
#if defined (USE_USB)
        	                                else {
        	                                	USB_result = TM_USB_VCP_Getc(&c);
        	                                    char_avail = (USB_result == TM_USB_VCP_DATA_OK);
        	                                    if (char_avail) {goto read_next;}  //check if USB buffer empty
        	                                }
#else
        	                                else {
#if !defined (STM32F411xE)
        	                                	char_avail = TM_USART_Getc(USART3,&c);
#else
        	                                	char_avail = TM_USART_Getc(USART2,&c);
#endif
        	                                	if (char_avail) {
        	                                		goto read_next;
        	                                	}  //if (c), check if USB buffer empty
        	                                 } //else
#endif
        	                    	   } else if (char_avail) goto readwhilestepping;// both command in buffer and a character ! ugly jump.........
            	  }// status == 0 when not in step interrupt
        	      else {  //stepping or paused, status > 0. Can be finished either by stopswitch or user interrupt
        	    	                       if ((wait_timer > 0) && ((TM_Time - wait_timer) > wait_millis)) { //when %i command is read, stepmotors should not move for delay seconds. Commands should be processed of course.
        	    	                    	   wait_timer = 0;// now if status = zero, processing will continue.
        	    	                    	   write_timer_done();
        	    	                       }

readwhilestepping:                         if (char_avail) {
        	    	                         if (((c == (int)'<') && (jpos_s < 0)) || (jpos==STLN-1) || ((jpos_s > 0) && (jpos > jpos_s + 2)))  { // start of new command
#if defined(DEBUG_OUTPUT)
        	                                      //if (jpos==STLN-1) trace_printf("STLN: %u\n", jpos);
#endif

        	                                    jpos=0;
        	                                    jpos_s = -3;
        	                                 }
        	    	                         unsigned_data[jpos]= (unsigned char) c;
        	    	                         char_avail = 0;
        	    	                         if (jpos_s < 0) {
        	                                   Command_data[jpos] = (char) c;   // always reading just one char !
        	                                   if (Command_data[jpos] == '>')  {// new command available
        	                                       Command_data[jpos+1]='\0';
        	                                       jpos_s = jpos;
        	                                   }
        	    	                         } //jpos_s < 0
                                           }//char_avail
        	    	                       if (jpos == jpos_s + 2) {
        	    	                    	   if  (CRC_CCITT(&unsigned_data[0],  jpos+1)==0){ //array starts at zero. thus number of chars is jpos+1
        	                                   //trace_printf("command: %s\n", Command_data);
        	                                   check_command(Command_data,'y', &succes,dummystr2,1);
        	                                   if (succes ==1) {
        	                                	   if (paused==1)
        	                                	      {stepper=paused_stepper;}
        	                                	   end_stepping(1,1); // end_stepping writes message including 8:y
        	                                	   init8y();
        	                                       // Finished, Stop any Timer23 operation
// switch step motors off(disable) and stop timer interrupt.
        	                                   } // succes == 1
        	                                   else { // check buffered command
        	                                	   check_command(Command_data,'c', &succes,dummystr2,0); // check if there are commands to add to buffer.
        	                                	   if (succes==1) {
        	                                		   jpos=0;
        	                                		   jpos_s = -3;
        	                                		   if (((check_packet_loss) && (first_command==saved_id)) || (!check_packet_loss)){
        	                                		      ++first_command;
        	                                		      if (no_commands < max_command)  ++no_commands;
        	                                	    	  strcpy(Command_buffer[no_commands],dummystr2); // save command in buffer for future use.
        	                                	          strcat(Command_buffer[no_commands],itoa(c_stepper,dummystr2,10) );
        	                                	          if (saved_id != -65535){ // note, this code is repeated in global.c inside command routine. modifications here should be reflected there as well.............
        	                                	        	  strcat(Command_buffer[no_commands],"$");
        	                                	        	  strcat(Command_buffer[no_commands],itoa(saved_id,dummystr2,10) );
        	                                	          }
        	                                	          strcpy(Command_data,"10:<c");
        	                                	          strcat(Command_data,Command_buffer[no_commands]);
        	                                	          strcat(Command_data,">");
        	                                		   } else {
        	                                			         strcpy(Command_data,"666:<c");
        	                                			         strcat(Command_data,itoa(first_command,dummystr2,10) );
        	                                			         strcat(Command_data,">");
        	                                		   }
        	                                	   } else
        	                                		   { // check pause command
        	                                		     check_command(Command_data,'m', &succes,dummystr2,0); // check if command to pause is send.
        	                                		     if (succes==1) {
        	                                		    	 jpos = 0;
        	                                		    	 jpos_s = -3;
        	                                		    	 strcpy(Command_data,"<19:");
        	                                		    	 strcat(Command_data,dummystr2);
        	                                		    	 strcat(Command_data,">");
        	                                		    	 paused = atoi(dummystr2);
        	                                		    	 if (paused==1) {
        	                                     		    	 paused_stepper=stepper;
        	                                     		    	 CpuCriticalVar();
        	                                     		    	 CpuEnterCritical();
                                                                 if (step_no > n_real_max) {//already decelerating
                                                                	 paused_steps=0;
                                                                 } else
                                                                 {
                                                                 paused_perc = (int)(100.0*((float) step_no/(float) total_steps));
        	                                		    		 paused_steps= total_steps- step_no - (total_steps-n_real_max); //these steps still have to be made after pause =0;
        	                                		    		 total_steps = step_no + (total_steps-n_real_max);// we will not stop immediately, but decelerate so no slip.
        	                                		    		 n_real_max=step_no;// start decelerating now.
                                                                 }
                                                                 CpuExitCritical();
        	                                		    	 } else stepper=paused_stepper;
        	                                		      }
        	                                		     else
        	                                	         {jpos=0;jpos_s=-3;} //nonsense command or <w>
        	                                		   }//check pause command
        	                                       }  // check ultra buffered command
        	                                	   //Give feedback upon status and how many commands pending in buffer.
         	                                	   if (no_commands > 0) {
         	                                		   strcat(Command_data,itoa(no_commands,dummystr2,10));
         	                                	   }
         	                                	   else if (status>0) {//working on final command
         	                                		   strcat(Command_data,itoa(-9999,dummystr2,10));
         	                                	   }
         	                                		   	else {strcat(Command_data,itoa(0,dummystr2,10));}
         	                                	   strcat(Command_data,";");//append percentage done.
         	                                	   if (homing ==0){
         	                                		  if (paused==1){
         	                                			 strcat(Command_data,itoa(paused_perc,dummystr2,10));
         	                                		  }
         	                                		  else {strcat(Command_data,itoa((int)(100.0*((float) step_no/(float) total_steps)),dummystr2,10));}
         	                                	   strcat(Command_data,"#");
         	                                	   } else {strcat(Command_data,"0#");}
#if defined(DEBUG_SR)
                                                   trace_printf("Sending: %s\n", Command_data);
#endif

#if defined (USE_USB)
         	                                	   TM_USB_VCP_Puts(Command_data);
#if defined(DEBUG_OUTPUT)
//                                                   trace_printf("Step_no: %i, total_steps: %i, Command data: %s\n", step_no, total_steps, Command_data);
#endif

#else
         	                                	   //TM_USART_DMA_Send(USART2, (uint8_t *)Command_data, strlen(Command_data));
         	                                	   //TM_USART_Send(USART2, (uint8_t *)Command_data, strlen(Command_data));
#if !defined (STM32F411xE)
         	                                	  TM_USART_Puts(USART3,Command_data);
#else
         	                                	  TM_USART_Puts(USART2,Command_data);
#endif
#endif
    	                                	   } // crc correct
    	                                	   else {// crc incorrect, request resend
    	                                		     jpos = 0;
    	                                		     jpos_s = -3;
#if defined(DEBUG_OUTPUT)
                                                     trace_printf("crc wrong: %s \n",Command_data);
#endif
    	                                		     strcat(Command_data,"667:<crc fault>");

#if defined (USE_USB)
         	                                	   TM_USB_VCP_Puts(Command_data);
#if defined(DEBUG_OUTPUT)
//                                                   trace_printf("Step_no: %i, total_steps: %i, Command data: %s\n", step_no, total_steps, Command_data);
#endif

#else
         	                                	   //TM_USART_DMA_Send(USART2, (uint8_t *)Command_data, strlen(Command_data));
         	                                	   //TM_USART_Send(USART2, (uint8_t *)Command_data, strlen(Command_data));
#if !defined (STM32F411xE)
         	                                	  TM_USART_Puts(USART3,Command_data);
#else
         	                                	  TM_USART_Puts(USART2,Command_data);
#endif
#endif
    	                                	       	} // crc incorrect, request resend
        	                               } // new command available
        	                            else { //status > 0, no command yet, advance and try read next char
        	                             jpos++;
        	                             //TM_USB_VCP_Putc(c);
        	                             }
        	                       }// stepping, status > 0
        	   }// char on USB TM_USB_VCP_DATA_OK
#if defined (USE_USB)
           } // TM_USB_VCP_CONNECTED
          else {// USB not connected
        	  if (green_on){
        	  green_on=0;


#if defined (USE_USB)
      /* Initialize USB VCP */
        	  TM_USB_DeInit();
        	  Delayms(3000);
              TM_USB_VCP_Init();
#else
      /* This pins are used on Nucleo boards for USB to UART via ST-Link */
      /* Init USART2 on pins TX = PA2, RX = PA3 */
#if !defined (STM32F411xE)
              TM_USART_Init(USART3, TM_USART_PinsPack_1, 9600);
#else
              TM_USART_Init(USART2, TM_USART_PinsPack_1, 9600);
#endif
          /* Init TX DMA for USART2 */
//          TM_USART_DMA_Init(USART2);
#endif


              TM_DISCO_LedOff(LED_GREEN);
              TM_DISCO_LedOn(LED_RED);
        	  }
          } // USB NOT OK
#endif

      }// while (1) loop


  return 0;
}




#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
