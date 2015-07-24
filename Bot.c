/****************************************************************************
 Module
   Bot.c

 Revision
   1.0.1

 Description
   This is a template file for implementing flat state machines under the
   Gen2 Events and Services Framework.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"

#include "Bot.h"
#include "IR_Detect.h"
#include "Servos.h"
#include "Shoot.h"
#include "IRemitter.h"
#include "Orientation.h"
#include "DCMotor.h"
#include "JSRcommand.h"

#include <stdio.h>
#include <hidef.h>
#include <mc9s12e128.h>     /* derivative information for the E128 */
#include <S12E128bits.h>    /* bit definitions for the E128 */ 
#include <Bin_Const.h>
#include <termio.h>
#include "S12eVec.h"
#include "ads12.h"


/*----------------------------- Module Defines ----------------------------*/
//LEDs
#define LED_ADDRESS DDRP
#define LED_PORT PTP
#define MATCH_LED BIT4HI
#define RELOAD_LED BIT5HI
#define RECESS_LED BIT3HI

#define BOT_FREQ 1250
#define GOAL_FREQ 2083

#define MOTOR_SPEED 100

//JSR Commands
#define STATUS_QUERY 0x3F
#define SCORE_QUERY  0xC3
#define NOTHING      0xFF
#define WAIT         0x00
#define START_ROUND  0x01
#define RECESS       0x03
#define SUDDEN_DEATH 0x04
#define END	         0x05

                               
//Tape Sensors Options
#define WHITE 0
#define RED 1
#define GREEN 2
#define BLACK 3

// these times assume a 1.024mS/tick timing
#define ONE_SEC 976
/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this machine.They should be functions
   relevant to the behavior of this state machine
*/

/*---------------------------- Module Variables ---------------------------*/
// everybody needs a state variable, you may need others as well.
// type of state variable should match htat of enum in header file
static BotState_t CurrentState;
static unsigned char CurrentRound;

static unsigned int RELOAD_STATUS = DARK_RELOAD_STATUS;
static bool DontChangeKnightFlag = false;

// with the introduction of Gen2, we need a module level Priority var as well
static uint8_t MyPriority;


/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitBot

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
bool InitBot( uint8_t Priority )
{

  MyPriority = Priority;

  // put us into the Initial State
  LED_ADDRESS |= MATCH_LED | RELOAD_LED | RECESS_LED; //Pins for LEDS as Outputs
  LED_PORT &= ~(MATCH_LED | RELOAD_LED | RECESS_LED); //LEDs start off
  
  CurrentState = Recess; 
  CurrentRound = 0;
  return true;
}

/****************************************************************************
 Function
     PostBot

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
bool PostBot( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunBot

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   add your description here
 Notes
   uses nested switch/case to implement the machine.
 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunBot( ES_Event ThisEvent )
{
  ES_Event ReturnEvent, NewEvent;
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
  
  
  
  
  //If Game Ends
  if(ThisEvent.EventType == NEW_COMMAND_RECEIVED && ThisEvent.EventParam == END)
  {
	  //Turn off Motors
	  puts("Turn Off Motors\n\r");
	  translateMotor(0);
	  LED_PORT &= ~MATCH_LED; 
	  CurrentState = Recess; 
	  NewEvent.EventType = StopAligning;
	  PostIR_Detect(NewEvent);
	  NewEvent.EventType = StopShootingMotors;
	  PostShoot(NewEvent);
  }
  
  if(ThisEvent.EventType == NEW_COMMAND_RECEIVED && ThisEvent.EventParam == WAIT)
  {
     CurrentRound = 0;
     translateMotor(0);
	  LED_PORT &= ~MATCH_LED; 
	  CurrentState = Recess;
	  NewEvent.EventType = StartShootingMotors;
	  PostShoot(NewEvent);
	  SetServo(1, 1500);
  }

  if(ThisEvent.EventType == ES_TIMEOUT){
      //Go Backwards and be searching for bot
      puts("Go Backwards\n\r");
    	rightMotor(-75);
      leftMotor(74);
      
      NewEvent.EventType = StartAlign;
		NewEvent.EventParam = BOT_FREQ;
		PostIR_Detect(NewEvent); 
  }
  
  switch ( CurrentState )
  {
    case(Recess):
		if(ThisEvent.EventType == NEW_COMMAND_RECEIVED)//StartRound
	    {
	      
			if(ThisEvent.EventParam == START_ROUND || ThisEvent.EventParam == SUDDEN_DEATH)
			{
	            if(CurrentRound == 0)
	            {
	               LED_PORT |= MATCH_LED;   //Turn on MatchIndicator LED
	            }
	            LED_PORT &= ~RECESS_LED; 
			   	CurrentRound++;
	            CurrentState = PasDArmes;
	            NewEvent.EventType = UpdateTargetColor;
	            NewEvent.EventParam = GREEN;
	            PostOrientation(NewEvent);
	          
	            printf("Starting Round %d\n\r", CurrentRound);
	          
	            /* Start Moving
	            Forward for Round 1, 3
	            Backward for 2, SuddenDeath */
	            if(CurrentRound == 2 || CurrentRound == 4)
	            {
	               ES_Timer_InitTimer(StopMoving_Timer, 25*ONE_SEC);
	              if(QueryIR_Detect() == Aligned)
	              {
	                //Shoot Balls
	                printf("Shooting Balls - Aligned\n\r");
	                NewEvent.EventType = Shoot_Ball;
	                NewEvent.EventParam = 5;
	                PostShoot(NewEvent);
	                ES_Timer_InitTimer(Bot_Timer, 5*ONE_SEC);
	              }
	              else
	              {
	                NewEvent.EventType = StopAligning;
	                NewEvent.EventParam = 1;
	                PostIR_Detect(NewEvent);
			     
	                printf("Shooting Balls\n\r");
	                NewEvent.EventType = Shoot_Ball;
	                NewEvent.EventParam = 5;
	                PostShoot(NewEvent);
	                ES_Timer_InitTimer(Bot_Timer, 6*ONE_SEC);
	              }
	              
	              
	            }
          
		      if (CurrentRound == 1 || CurrentRound == 3)
		      {
		        NewEvent.EventType = StartShootingMotors;
			     PostShoot(NewEvent); 
			     ES_Timer_InitTimer(StopMoving_Timer, 25*ONE_SEC);
			     NewEvent.EventType = StartAlign;
			     NewEvent.EventParam = BOT_FREQ;
			     PostIR_Detect(NewEvent);
			   
			     //Go Forwards
			     puts("Go Forwards\n\r");
			     rightMotor(75);
              leftMotor(-74);
		      }
			}
          

	   }// end if on EventType 
      break; //End CurrentState = Recess

    case(PasDArmes):
      if(ThisEvent.EventType == NEW_COMMAND_RECEIVED && ThisEvent.EventParam == RECESS)
	   {
	     LED_PORT |= RECESS_LED;
        printf("Round %d Ending\n\r", CurrentRound);
        if(CurrentRound == 1 || CurrentRound == 3)
        { //Align with goal
			 NewEvent.EventType = StartAlign;
			 NewEvent.EventParam = GOAL_FREQ;
			 PostIR_Detect(NewEvent);
			 NewEvent.EventType = StartShootingMotors;
			 PostShoot(NewEvent);
        }
        
        if(CurrentRound == 2)
        {
         SetServo(1, 1485);
         NewEvent.EventType = StopShootingMotors;
         PostShoot(NewEvent);
         NewEvent.EventType = RELOAD_BALLS;
         PostShoot(NewEvent);
         PostIRemitter(NewEvent);
         NewEvent.EventType = StopAligning;
         PostIR_Detect(NewEvent);
        }
      
        CurrentState = Recess;
	   }
      break; //End CurrentState = PasDArmes
      
    		
  }// end switch on Current State
  return ReturnEvent;
}

/****************************************************************************
 Function
     QueryBot

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
BotState_t QueryBot(void)
{
   return(CurrentState);
}


unsigned char GetCurrentRound(void)
{
	return(CurrentRound);
}
/***************************************************************************
 private functions
 ***************************************************************************/

