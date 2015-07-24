/****************************************************************************
 Module
   IRemitter.c

 Revision
   1.0.1

 Description
   This is the first service for the Test Harness under the 
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 10/21/13 19:38 jec      created to test 16 possible serves, we need a bunch
                         of service test harnesses
 08/05/13 20:33 jec      converted to test harness service
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for the framework and this service
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "IRemitter.h"
#include "S12eVec.h"
#include "ES_Timers.h"

#include <hidef.h>
#include <mc9s12e128.h>     /* derivative information for the E128 */
#include <S12E128bits.h>    /* bit definitions for the E128 */
#include <Bin_Const.h>
#include <termio.h>

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 1.024mS/tick timing
#define ONE_SEC 976
#define HALF_SEC (ONE_SEC/2)
#define TWO_SEC (ONE_SEC*2)
#define FIVE_SEC (ONE_SEC*5)

#define _ms_ *1500
#define Period40ms (40 _ms_)
#define Period10ms (10 _ms_)

#define IRemitter_ADDRESS DDRT
#define IRemitter_PORT PTT
#define IRemitter_PIN  BIT7HI

#define ReloadLED_ADDRESS DDRP
#define ReloadLED_PORT PTP
#define ReloadLED_PIN BIT5HI
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
//void InitPWM(void);
static void InitTimer(void);
void interrupt _Vec_tim0ch6 Timer40ms (void);
void interrupt _Vec_tim0ch7 Timer10ms (void);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static unsigned int count = 0;
static unsigned int ballCount = 0;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTestHarnessService10

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, and does any 
     other required initialization for this service
 Notes

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitIRemitter ( uint8_t Priority )
{
  ES_Event ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  IRemitter_ADDRESS |= IRemitter_PIN;
  IRemitter_PORT &= ~IRemitter_PIN;
  ReloadLED_ADDRESS |= ReloadLED_PIN;
  ReloadLED_PORT &= ~ReloadLED_PIN;
  
  // post the initial transition event
  ThisEvent.EventType = ES_INIT;
  if (ES_PostToService( MyPriority, ThisEvent) == true)
  {
      return true;
  }else
  {
      return false;
  }
}

/****************************************************************************
 Function
     PostTestHarnessService10

 Parameters
     EF_Event ThisEvent ,the event to post to the queue

 Returns
     bool false if the Enqueue operation failed, true otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostIRemitter( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTestHarnessService10

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunIRemitter( ES_Event ThisEvent )
{
  ES_Event ReturnEvent, NewEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  if (ThisEvent.EventType == RELOAD_BALLS)
  {
    count = 0;
    ES_Timer_InitTimer(IRemitterTimer, 3000);  
    IRemitter_PORT |= IRemitter_PIN;
    InitTimer();
    ReloadLED_PORT |= ReloadLED_PIN; //Turn the Reload LED on
    
  }
  
  if (ThisEvent.EventType == ES_TIMEOUT)
  { 
    printf("i = %d, Balls = %d \r\n",count,ballCount);
    if (ballCount < 5)
    { 
      count = 0;
      ES_Timer_InitTimer(IRemitterTimer, 3000);
      IRemitter_PORT |= IRemitter_PIN;
      ReloadLED_PORT |= ReloadLED_PIN; //Turn the Reload LED ON
      InitTimer();
    } 
    else 
    { 
      ballCount = 0;
      ReloadLED_PORT &= ~ReloadLED_PIN; //Turn the Reload LED OFF
      TIM0_TIE &= ~(_S12_C6I|_S12_C7I); //disable oc4,5 interrupt
      NewEvent.EventType = StartShootingMotors;
      PostShoot(NewEvent); 
   }
  }
  
  return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void InitTimer(void)
{
   
   TIM0_TSCR1 =_S12_TEN; //Enable
   TIM0_TSCR2 =(_S12_PR2);//|_S12_PR1|_S12_PR0); //Set Prescale to /16
 
   //Output compare OC4,5
   TIM0_TIOS |= (_S12_IOS6|_S12_IOS7);//Set output compare
   TIM0_TCTL1 &= ~(_S12_OL6 | _S12_OM6 | _S12_OL7 | _S12_OM7);// no pin connected
   TIM0_TC6 = TIM0_TCNT + Period40ms;
   TIM0_TC7 = TIM0_TCNT + Period10ms;
   TIM0_TFLG1 = _S12_C6F; //clear OC4 flag;
   TIM0_TFLG1 = _S12_C7F; //clear OC5 flag;
   TIM0_TIE |= (_S12_C6I|_S12_C7I); //enable oc4,5 interrupt   
   EnableInterrupts; //Enable Interrupts
   
   printf("InitTimer\n");
}

void interrupt _Vec_tim0ch6 Timer40ms (void)
{
   IRemitter_PORT |= IRemitter_PIN; // Set Signal HI
   TIM0_TC6 = TIM0_TCNT + Period40ms; // program next compare
   TIM0_TFLG1 = _S12_C6F; //clear OC4 flag;
   ReloadLED_PORT |= ReloadLED_PIN; //Turn the Reload LED ON  
   
} /* End Interrupt Timer40ms */


void interrupt _Vec_tim0ch7 Timer10ms (void)
{
   IRemitter_PORT &= ~IRemitter_PIN; // Set Signal LO
   TIM0_TC7 = TIM0_TC6 + Period10ms; // program next compare
   TIM0_TFLG1 = _S12_C7F; //clear OC5 flag;
   
   count += 1;
   ReloadLED_PORT &= ~ReloadLED_PIN; //Turn the Reload LED OFF  
   if (count >= 10){
      TIM0_TIE &= ~(_S12_C6I|_S12_C7I); //disable oc4,5 interrupt   
      ballCount += 1;  
   }
   
} /* End Interrupt Timer10ms */

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

