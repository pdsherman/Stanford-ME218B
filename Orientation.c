/****************************************************************************
 Module
   Orientation.c

 Revision
   1.0.1

 Description
  The game board for the project is mostly white but includes a line of
  red tape at the mid line and green tape on either side to designate the 
  home sections. A simple tape sensor is used to examine the color on the ground
  as the robot moves. 

 History
 When           Who     What/Why
 -------------- ---     --------
 03/10/15 12:10 pds05    Changed file for use in project
 10/21/13 19:38 jec      created to test 16 possible serves, we need a bunch
                         of service test harnesses
 08/05/13 20:33 jec      converted to test harness service
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "JSRcommand.h"
#include <hidef.h>
#include <mc9s12e128.h>     /* derivative information for the E128 */
#include <S12E128bits.h>    /* bit definitions for the E128 */
#include <Bin_Const.h>
#include <termio.h>
#include "S12eVec.h"
#include "ADS12.h"
#include "Orientation.h"
#include "DCMotor.h"
#include "Bot.h"
#include "JSRcommand.h"
#include "Shoot.h"
#include "IRemitter.h"

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 1.024mS/tick timing
#define ONE_SEC 976
#define HALF_SEC (ONE_SEC/2)
#define TWO_SEC (ONE_SEC*2)
#define FIVE_SEC (ONE_SEC*5)

//Tape Sensors
#define RIGHT_TAPESENSOR_PIN 6 

//Tape Colors
#define WHITE 0
#define RED 1
#define GREEN 2
#define BLACK 3

#define AlignTapeTime 10 //20ms


/*---------------------------- Module Functions ---------------------------*/

/***************************************************************************
 private functions
 ***************************************************************************/
//void AlignWithTape(void);
/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;

static unsigned int TargetColor;
static unsigned int CurrentOState;
static unsigned int RELOAD_STATUS = DARK_RELOAD_STATUS;
static bool DontChangeKnightFlag = false;
static bool NotYetDetected = true;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitTestHarnessService13

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
bool InitOrientation ( uint8_t Priority )
{
   ES_Event ThisEvent;

   MyPriority = Priority;
   ADS12_Init("AAAAAAAA"); //Analog Inputs
 
   CurrentOState = Align2Horizontal;
   TargetColor = WHITE;
  
   ThisEvent.EventType = ES_INIT;
   if (ES_PostToService( MyPriority, ThisEvent) == true)
      return true;
   else
      return false;
}

/****************************************************************************
 Function
     PostTestHarnessService13

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
bool PostOrientation( ES_Event ThisEvent )
{
   return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunTestHarnessService13

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Robot looks for the first line of green tape, then the red mid-line and then 
   the green again. Once the 2nd green tape is spotted, the robot moves for
   another 1/2 second before stopping
 
 Author 
   Patrick Sherman,   03/02/14, 16:30
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunOrientation( ES_Event ThisEvent )
{
   ES_Event ReturnEvent;
   ES_Event NewEvent;
   ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
       
   switch (ThisEvent.EventType)
   {
      case(UpdateTargetColor):
         TargetColor = ThisEvent.EventParam;
         NotYetDetected = true;
         break;
  		
      case(Right_Tape):
         //Seeing Red Tape
         if(ThisEvent.EventParam == TargetColor && TargetColor == RED)
         {
            NotYetDetected = false;
            if(GetCurrentRound() == 1 || GetCurrentRound() == 3)
            {
               timedTranslate(65, (ONE_SEC*3)/8);
            }
            else 
            {
               timedTranslate(-65, (ONE_SEC*3)/4);
            }
         }
  		   //Seeing Green Tape
         if(ThisEvent.EventParam == TargetColor && TargetColor == GREEN)
         {
            ES_Timer_InitTimer(Tape_Timer, 6*ONE_SEC/4);
         }
         break;
  		
      case(ES_TIMEOUT):
         if((ThisEvent.EventParam == Pause_Timer) && (NotYetDetected == true))
         {
            ES_Timer_StopTimer(StopMoving_Timer);
            if(GetCurrentRound() == 1 || GetCurrentRound() == 3)
            {
               //Forward into Home
               timedTranslate(65, 1*(ONE_SEC)/4);
            }
            else 
            {
               //Backing into Home
               timedTranslate(-65, (ONE_SEC)/2);
            }
         }
         else if (ThisEvent.EventParam == Tape_Timer)
         {
            TargetColor = RED;
         }
         else if((ThisEvent.EventParam == StopMoving_Timer) &&
                   (NotYetDetected == true))
         {
            NotYetDetected = false;
            if(GetCurrentRound() == 1 || GetCurrentRound() == 3)
            {
               //Forward into Home
               timedTranslate(65, (ONE_SEC*1)/4);
            }
            else 
            {
               //Backing into Home
               timedTranslate(-65, (ONE_SEC)/4);
            }
         }
         break;
   } //End Swich  
  
   return ReturnEvent;
}

/***************************************************************************
                        private functions
 **************************************************************************/

/*----------------Tape Sensor Event Checkers-----------------------*/

/* 
Function: Check4RightTape
-----------------------------
Event Checker for tape center on side of bot
*/
bool Check4RightTape(void) {
	ES_Event NewEvent;
	static unsigned int RightTapeFlag = WHITE;
   static unsigned int LastPinState;
   static unsigned int CurrentPinState;
   static int numTimesSeen = 0;
    
    bool ChangeSeen = false;                                  
    
    CurrentPinState = 9*ADS12_ReadADPin(COLOR_TAPESENSOR_PIN)/10 
                        + LastPinState/10;
    
    LastPinState = CurrentPinState;

    
    if(RightTapeFlag != WHITE)
    {
      if(CurrentPinState < 220) //See White Tape
      {
        RightTapeFlag = WHITE;
        ChangeSeen = true;
      }
    }
    if (RightTapeFlag != RED)
    {
      if(CurrentPinState > 260 && CurrentPinState < 315) //See Red Tape 
      {
         RightTapeFlag = RED;
         ChangeSeen = true;
      }
    }
    if (RightTapeFlag != GREEN)
    {
      if(CurrentPinState > 350 && CurrentPinState < 440) //See Green Tape 
      {
         RightTapeFlag = GREEN;
         ChangeSeen = true;
      }
    }
    if (RightTapeFlag != BLACK)
    {
      if(CurrentPinState > 460) //See Black Tape 
      {	 
         RightTapeFlag = BLACK;
         ChangeSeen = true;
      }
    }   
        
    if (ChangeSeen == true)
    {
    	numTimesSeen = 0;
    }
    else
    {
       numTimesSeen++;	
    }
    
    if(numTimesSeen == 400)
    {
       NewEvent.EventType = Right_Tape;
       NewEvent.EventParam = RightTapeFlag;
       PostOrientation(NewEvent);
       return true;
    }
    if (numTimesSeen > 1000)
    {
    	 numTimesSeen = 1001;
    }
    
    return false;
} /* End Check4RightTape */
 
/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

