/****************************************************************************
 Module
   Shoot.c

 Revision
   1.0.1

 Description
   Source code for robot to shooting foam balls. A servo is used to push the
   foam ball into a pair of spinning wheels. The robot is able to hold a
   total of 5 balls. Once a shooting command is given, all 5 will be shot
   at the same time.   


 History
 When           Who     What/Why
 -------------- ---     --------
 03/01/14 14:40 pds05    converted file for use in project
 01/16/12 09:58 jec      began conversion from TemplateFSM.c
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "Servos.h"
#include "Shoot.h"

/*----------------------------- Module Defines ----------------------------*/
#define PWMSCALE    150 // 15 -> PWM 2KHZ
#define PWMPERIOD   100 // Maximum Ticks for PWM Period

#define FEEDER_SERVO 0
#define RETRACT_WIDTH 700
#define SHOOT_WIDTH 970
#define WAIT_TIME 700
#define MAX_NUM_BALLS 5
/*---------------------------- Module Functions ---------------------------*/

/*---------------------------- Module Variables ---------------------------*/
static uint8_t MyPriority;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitShoot

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
bool InitShoot ( uint8_t Priority )
{
   ES_Event ThisEvent;

   MyPriority = Priority;
   // Start my init
   PWME |= (_S12_PWME2 | _S12_PWME3); // Enable PWM 2 & 3
   PWMPRCLK |= _S12_PCKB1; // Clock B divide by 4, 6MHz clock
   PWMPRCLK &= ~(_S12_PCKB0 | _S12_PCKB2);
   PWMPOL |= (_S12_PPOL2 | _S12_PPOL3); // Set polarity output initially high
   PWMCLK |= (_S12_PCLK2 | _S12_PCLK3); // Enable Scaled clock for channel 0
   PWMSCLB = PWMSCALE; // Scale clock B
   PWMPER2 = PWMPERIOD; // Set PWM period Shoot Motor 1
   PWMPER3 = PWMPERIOD; // Set PWM period Shoot Motor 2
   PWMDTY2 = 0; // Shoot Motor 1
   PWMDTY3 = 0; // Shoot Motor 2 

   // post the initial transition event
   ThisEvent.EventType = ES_INIT;
   if (ES_PostToService( MyPriority, ThisEvent) == true)
      return true;
   else
      return false;
}

/****************************************************************************
 Function
     PostShoot

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
bool PostShoot( ES_Event ThisEvent )
{
   return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunShoot

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   When a shoot ball event is passed to the service, servo activates to push
   foam ball into spinning wheels. A balls left count is decremented until
   it reaches 0 and the servo and wheels stop actuating.   
 
 Author
   Patrick Sherman,   03/03/14, 10:30
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunShoot( ES_Event ThisEvent )
{
   ES_Event ReturnEvent;
   ES_Event NewEvent;
   static int ballsLeft = 5;
  
   unsigned int distance;
   ReturnEvent.EventType = ES_NO_EVENT; // assume no errors

   switch(ThisEvent.EventType)
   {
      case(Shoot_Ball):
         ballsLeft = ThisEvent.EventParam;
         if (ballsLeft >0)
         {
            SetServo(FEEDER_SERVO, SHOOT_WIDTH);
            ES_Timer_InitTimer(ShootTimer, WAIT_TIME);
            ballsLeft--;
         }

         if(ballsLeft == 0)
         {
            PWMDTY2 = 0; // Shoot Motor 1
            PWMDTY3 = 0; // Shoot Motor 2 
         }
         break;
      
      case(ES_TIMEOUT):
         if(ThisEvent.EventParam == ShootTimer)
         { 
            SetServo(FEEDER_SERVO, RETRACT_WIDTH);
            if (ballsLeft > 0)
            {
               ES_Timer_InitTimer(Feeder_Timer, WAIT_TIME); 
            }
         }
         else
         {
            NewEvent.EventType = Shoot_Ball;
            NewEvent.EventParam = ballsLeft;
            PostShoot(NewEvent); 
         }
         break;
         
      case(RELOAD_BALLS):
         ballsLeft = MAX_NUM_BALLS;
         break;
         
      case(StartShootingMotors):
         PWMDTY2 = 13; // Shoot Motor 1
         PWMDTY3 = 13; // Shoot Motor 2
         break;
         
      case(StopShootingMotors):
         PWMDTY2 = 0; // Shoot Motor 1
         PWMDTY3 = 0; // Shoot Motor 2
         break;      
    }
      
   return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

