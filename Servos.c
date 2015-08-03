/******************************************************
  File:
    Servos.c

  Descrpitions:
    Pair of functions to run the servos used in ME 218B
	 project.

  Notes:
   Servos
   HS-645MG: ~180 deg, NewWidth Range [600, 2400]
   HX5010:   ~180 deg, Newwidth Range [600, 2400]
   HXT900:   ~90 deg,  NewWidth Range [650, 1650]

  Authors
    P. Sherman, C. Bai  03/01/14
*******************************************************/
/*-------------------- Include Files ---------------------*/

#include <hidef.h>
#include <mc9s12e128.h>     /* derivative information for the E128 */
#include <S12E128bits.h>    /* bit definitions for the E128 */ 
#include <Bin_Const.h>
#include <termio.h>
#include "S12eVec.h"
#include "Servos.h"

/*----------------------- Define ------------------------*/
#define INITIAL_4 0xEFA0
#define INITIAL_5 0xE000
#define INITIAL_6 0xDA80

/*---------------------- Functions -----------------------*/
void InitServos(void)
{
   //Set up for Timers
	TIM1_TSCR1 |= _S12_TEN; //Enable Timers
   TIM1_TSCR2 = _S12_PR1; //Clock Scaling 10.92 ms rollover rate
   //Select Channels for output compare
   TIM1_TIOS |= _S12_IOS4 | _S12_IOS5 | _S12_IOS6; 
   //Program Rise on Compare
	TIM1_TCTL1 |= (_S12_OM4 | _S12_OL4) | (_S12_OM5 | _S12_OL5)
                   | (_S12_OM6 | _S12_OL6);

	TIM1_TTOV |= _S12_TOV4 | _S12_TOV5 | _S12_TOV6; //Bit to ToggleOverFlow
   TIM1_TC4 = INITIAL_4;
   TIM1_TC5 = INITIAL_5;
   TIM1_TC6 = INITIAL_6;

} /* End InitServos */



void SetServo(unsigned char ChannelNum, unsigned int NewWidth){

   switch(ChannelNum)
   {
      case 0:
         TIM1_TC4 = 0xFFFF - 6*NewWidth;
         break;
      case 1:
         TIM1_TC5 = 0xFFFF - 6*NewWidth;
         break;
      case 2:
         TIM1_TC6 = 0xFFFF - 6*NewWidth;
         break;
   }	
} /* End SetServo */
