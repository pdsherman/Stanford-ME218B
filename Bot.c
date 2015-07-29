/****************************************************************************
 Module
   Bot.c

 Revision
   1.0.1

 Description
   This is the source code the flat state machine of the jousting autonomous
   robot for ME 218B project. Project is divided into 3 different rounds (with
   a sudden death if necessary) and short recess between each round.

 History
 When           Who     What/Why
 -------------- ---     --------
 03/28/14 15:15 pds05    Revised template for use with project
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
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
//LED hardware pin/port
#define LED_ADDRESS DDRP
#define LED_PORT PTP
#define MATCH_LED BIT4HI
#define RELOAD_LED BIT5HI
#define RECESS_LED BIT3HI

//IR target frequencies
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

//This time assumes a 1.024mS/tick timing
#define ONE_SEC 976
/*---------------------------- Module Functions ---------------------------*/

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;
static BotState_t CurrentState;
static unsigned char CurrentRound;

static unsigned int RELOAD_STATUS = DARK_RELOAD_STATUS;
static bool DontChangeKnightFlag = false;

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
   Main code for control of robot to play jousting games. In charge of
   responding to different events depending on the state and round of
   the game play.

 Notes
   uses nested switch/case to implement the machine.

 Author
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunBot( ES_Event ThisEvent )
{
   ES_Event ReturnEvent, NewEvent;
   ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
  
   //Read switch to determine side of robot 
   if (RED_DARK_PORT & RED_DARK_PIN == RED_DARK_PIN) 
   {
      RELOAD_STATUS = RED_RELOAD_STATUS;  //RED KNIGHT
      DontChangeKnightFlag = true;
   }
   else if (DontChangeKnightFlag == false)
   {
      RELOAD_STATUS = DARK_RELOAD_STATUS; //DARK KNIGHT
   }	
  
   /*
   If Game ends: 
      Stop moving
      Turn off "Active Game" LED
      Turn off all motors/servos
      Set state to recess
   */
   if(ThisEvent.EventType == NEW_COMMAND_RECEIVED && ThisEvent.EventParam == END)
   {
      translateMotor(0);
      LED_PORT &= ~MATCH_LED; 
      CurrentState = Recess; 
      NewEvent.EventType = StopAligning;
      PostIR_Detect(NewEvent);
      NewEvent.EventType = StopShootingMotors;
      PostShoot(NewEvent);
   }
 
   /*
   If Round ends:
      Stop moving
      Start motors for shooting foam balls
      Aim for goal
   */ 
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

   /*
   After shooting all foam balls at goal:
      Start moving backswards
      Begin searching for opponent
   */
   if(ThisEvent.EventType == ES_TIMEOUT){
      rightMotor(-75);
      leftMotor(74);
      
      NewEvent.EventType = StartAlign;
      NewEvent.EventParam = BOT_FREQ;
      PostIR_Detect(NewEvent); 
   }
 
   //Respond to all other events based on current robot state and game round 
   switch ( CurrentState )
   {
      case(Recess):
         //New Command from JSR
         if(ThisEvent.EventType == NEW_COMMAND_RECEIVED)
         {
            //New Round starts: Begin moving on game board
            if(ThisEvent.EventParam == START_ROUND || 
               ThisEvent.EventParam == SUDDEN_DEATH)
            {
               if(CurrentRound == 0)
	               LED_PORT |= MATCH_LED;//Turn on MatchIndicator LED
	            LED_PORT &= ~RECESS_LED; 
			   	CurrentRound++;
	            CurrentState = PasDArmes;

               //Start looking for green tape on game board
	            NewEvent.EventType = UpdateTargetColor;
	            NewEvent.EventParam = GREEN;
	            PostOrientation(NewEvent);
	          
	         
               //For Round 2, 4. Try to shoot foams balls in goal
               if(CurrentRound == 2 || CurrentRound == 4)
               {
                  ES_Timer_InitTimer(StopMoving_Timer, 25*ONE_SEC);
                  if(QueryIR_Detect() == Aligned)
                  {
                     //Shoot Balls
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
                     
                     NewEvent.EventType = Shoot_Ball; 
                     NewEvent.EventParam = 5;
			            PostShoot(NewEvent); 
                     
                     ES_Timer_InitTimer(Bot_Timer, 6*ONE_SEC);
                  }
               }

               //For Round 1, 3 start moving foward and start looking for
               //opponent robot 
               if(CurrentRound == 1 || CurrentRound ==3)
               {
                  NewEvent.EventType = StartShootingMotors;
                  PostShoot(NewEvent);
   
                  ES_Timer_InitTimer(StopMoving_Timer, 25*ONE_SEC);
 
                  NewEvent.EventType = StartAlign;
			         NewEvent.EventParam = BOT_FREQ;
                  PostIR_Detect(NewEvent);
			   
                  rightMotor(75);
                  leftMotor(-74);
               }
            }
	      }
         break; //End CurrentState = Recess

      case(PasDArmes):
         if(ThisEvent.EventType == NEW_COMMAND_RECEIVED && 
            ThisEvent.EventParam == RECESS)
         {
            LED_PORT |= RECESS_LED;
            //Try to align with goal. Aiming for foam balls
            if(CurrentRound == 1 || CurrentRound == 3)
            { 
               //Align with goal
               NewEvent.EventType = StartAlign;
               NewEvent.EventParam = GOAL_FREQ;
               PostIR_Detect(NewEvent);

               NewEvent.EventType = StartShootingMotors;
               PostShoot(NewEvent);
            }
       
            //Reload balls at reloading stations 
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
         break; 
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


/****************************************************************************
 Function
   GetCurrentRound

 Description
   Return the current round of the game.
****************************************************************************/
unsigned char GetCurrentRound(void)
{
   return(CurrentRound);
}
/***************************************************************************
 private functions
 ***************************************************************************/

