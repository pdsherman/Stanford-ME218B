/****************************************************************************
 Module
   JSRcommand.c

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
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/

/***************************************************************************
 private functions
 ***************************************************************************/
//void AlignWithTape(void);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;

static unsigned int TargetColor;
static unsigned int CurrentOState; //Align to horizontal or vertical tapes.
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
  /********************************************
   in here you write your initialization code
   *******************************************/
  // post the initial transition event
  
  ADS12_Init("AAAAAAAA"); //Already initialized in IR_Detect.c file
 
  CurrentOState = Align2Horizontal;
  TargetColor = WHITE;
  
  
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
   add your description here
 Notes
   
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunOrientation( ES_Event ThisEvent )
{
  ES_Event ReturnEvent;
  ES_Event NewEvent;
  ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
  if (RED_DARK_PORT&RED_DARK_PIN == RED_DARK_PIN) //Hi
  {
    RELOAD_STATUS = RED_RELOAD_STATUS;  //RED KNIGHT
    DontChangeKnightFlag = true;
  }
  else if (DontChangeKnightFlag == false)
  {
    RELOAD_STATUS = DARK_RELOAD_STATUS; //DARK KNIGHT
  }	
  
      
  switch (ThisEvent.EventType)
  {
  	case(UpdateTargetColor):
  	   printf("Change Target Color: %d\n\r", ThisEvent.EventParam);
  		TargetColor = ThisEvent.EventParam;
  		NotYetDetected = true;
  		break;
  		
  	case(Right_Tape):
  	   //Seeing Red Tape
  		if(ThisEvent.EventParam == TargetColor && TargetColor == RED)
  		{
  		 printf("STOP\n\r");
  		 NotYetDetected = false;
  		 if(GetCurrentRound() == 1 || GetCurrentRound() == 3)
  		   {
  			   timedTranslate(65, (ONE_SEC*3)/8);
  			   puts("Move forward for 0.75 sec\n\r");
  		   }
  		   else 
  		   {
  			   timedTranslate(-65, (ONE_SEC*3)/4);
  			   puts("Move backward for 0.5 sec\n\r");
  		   }

  		}
  		//Seeing Green Tape
  		if(ThisEvent.EventParam == TargetColor && TargetColor == GREEN)
  		{
  		   ES_Timer_InitTimer(Tape_Timer, 6*ONE_SEC/4);
  			printf("Change Target Color to Red\n\r");
  		}
  		break;
  		
  	case(ES_TIMEOUT):
  	   if((ThisEvent.EventParam == Pause_Timer)&& (NotYetDetected == true))
  	   {
  	      ES_Timer_StopTimer(StopMoving_Timer);
  	      printf("### Pause_Timer TIMEOUT, Tape Detected!! ###\n\r");  
  		   if(GetCurrentRound() == 1 || GetCurrentRound() == 3)
  		   {
  		      //Forward into Home
  			   timedTranslate(65, 1*(ONE_SEC)/4);
  			   puts("Move forward for 0.5 sec\n\r");
  		   }
  		   else 
  		   {
  		      //Backing into Home
  			   timedTranslate(-65, (ONE_SEC)/2);
  			   puts("Move backward for 0.5 sec\n\r");
  		   }
  		
  	   }
  	   
      else if (ThisEvent.EventParam == Tape_Timer)
      {
         TargetColor = RED;
      }
      
      else if((ThisEvent.EventParam == StopMoving_Timer)&& (NotYetDetected == true))
  	   {
			 
  	      NotYetDetected = false;
  	      printf("### StopMoving_Timer TIMEOUT, Tape NOT detected ###%%%\n\r");  
  		   if(GetCurrentRound() == 1 || GetCurrentRound() == 3)
  		   {
  		      //Forward into Home
  			   timedTranslate(65, (ONE_SEC*1)/4);
  			   puts("Move forward for 0.5 sec\n\r");
  		   }
  		   else 
  		   {
  		      //Backing into Home
  			   timedTranslate(-65, (ONE_SEC)/4);
  			   puts("Move backward for 0.5 sec\n\r");
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
    
    CurrentPinState = 9*ADS12_ReadADPin(COLOR_TAPESENSOR_PIN)/10 + LastPinState/10;
    
    LastPinState = CurrentPinState;

    //printf("\t\tRight CurrentPinState = %u \n\r", CurrentPinState);
    
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
      printf("Color: %d\n\r", RightTapeFlag);
      printf("\tNumTimesSeen = %u \n\r", numTimesSeen);
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
       printf("Posting Right Tape Event \n\r");
       printf("Color: %d\n\r", RightTapeFlag);
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

