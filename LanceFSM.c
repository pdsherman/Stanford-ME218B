/****************************************************************************
 Module
   LanceFSM.c

 Revision
   1.0.1

 Description
   This is a file for implementing flat state machine for deploying a
	lance for the Winter 2014 ME218B Project. Lance is allowed to be 
   extended for a total of 3 seconds before being retracted. Additionally
   it must wait 1 second before being extended again

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 03/10/14 20:30 PS       Edited file for use with Lance state machine
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "LanceFSM.h"
#include "Servos.h"

#include <stdio.h>
#include <hidef.h>
#include <mc9s12e128.h>     /* derivative information for the E128 */
#include <S12E128bits.h>    /* bit definitions for the E128 */ 
#include <Bin_Const.h>
#include <termio.h>
#include "S12eVec.h"

/*----------------------------- Module Defines ----------------------------*/
#define LANCE_SERVO 2
#define DEPLOY_WIDTH 900
#define RETRACT_WIDTH 1600

#define ONE_SEC 976
/*---------------------------- Module Functions ---------------------------*/

/*---------------------------- Module Variables ---------------------------*/
static LanceState_t CurrentState;
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitLance

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets up the initial transition and does any
     other required initialization for this state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 18:55
****************************************************************************/
bool InitLance ( uint8_t Priority )
{
   ES_Event ThisEvent;
  
   MyPriority = Priority;
   //Put lance state machine into the Initial PseudoState
   CurrentState = Retracted;
 
   return true; 
}

/****************************************************************************
 Function
     PostLance

 Parameters
     EF_Event ThisEvent , the event to post to the queue

 Returns
     boolean False if the Enqueue operation failed, True otherwise

 Description
     Posts an event to this state machine's queue
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:25
****************************************************************************/
bool PostLance( ES_Event ThisEvent )
{
   return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunLance

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   When a deploy lance event is recieved, the robot uses a servo to extend
   the lance and a timer is used to wait the 3 seconds to retract lance
   and 1 second wait until the lance is able to be extended again.

 Author 
   P. Sherman,        03/10/14, 20:40  
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunLance( ES_Event ThisEvent )
{
   ES_Event ReturnEvent;
   ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
 
   //Deploy Lance if event is posted and lance is in valid state 
   if(ThisEvent.EventType == Deploy_Lance && CurrentState == Retracted)
   {
      SetServo(LANCE_SERVO, DEPLOY_WIDTH);
	   ES_Timer_InitTimer(Lance_Timer, 3*ONE_SEC);
	   CurrentState = Deployed;
    
   } 

   //Timeout Event recieved
   else if(ThisEvent.EventType == ES_TIMEOUT)
   {
      if(CurrentState == Deployed)
      {
         SetServo(LANCE_SERVO, RETRACT_WIDTH);
         ES_Timer_InitTimer(Lance_Timer, ONE_SEC);
         CurrentState = Inactive;
	   } else if (CurrentState == Inactive)
      {
         CurrentState = Retracted;
      }
   }

   return ReturnEvent;
}

/****************************************************************************
 Function
     QueryLance

 Parameters
     None

 Returns
     TemplateState_t The current state of the Template state machine

 Description
     returns the current state of the Template state machine
 Notes

 Author
     J. Edward Carryer, 10/23/11, 19:21
****************************************************************************/
LanceState_t QueryLance ( void )
{
   return(CurrentState);
}

/***************************************************************************
 private functions
 ***************************************************************************/

