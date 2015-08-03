/****************************************************************************
 Module
   IR_Detect.c

 Revision
   1.0.1

 Description
   Source code for IR Light detection state machine. Target Bot and Target
   Goal each have IR LED that flashes at known frequency. An IR sensing
   circit was built to input an analog voltage for different frequencies
   to detect targets. A high power servo is used to rotate the section of
   the robot.
  
 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 02/17/14 10:30 PS       Converted template for use with IR Dectection
 01/15/12 11:12 jec      revisions for Gen2 framework
 11/07/11 11:26 jec      made the queue static
 10/30/11 17:59 jec      fixed references to CurrentEvent in RunTemplateSM()
 10/23/11 18:20 jec      began conversion from SMTemplate.c (02/20/07 rev)
****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "IR_Detect.h"
#include "Servos.h"
#include "Shoot.h"
#include "Bot.h"
#include "LanceFSM.h"
#include "DCMotor.h"

#include <stdio.h>
#include <hidef.h>
#include <mc9s12e128.h>     /* derivative information for the E128 */
#include <S12E128bits.h>    /* bit definitions for the E128 */ 
#include <Bin_Const.h>
#include <termio.h>
#include "S12eVec.h"
#include "ads12.h"

/*----------------------------- Module Defines ----------------------------*/
//Hardware Pin/Port on micro-controllers for IR Sensors
#define IR_ADDRESS DDRT
#define IR_PORT PTT
#define IR_SESNOR_1 BIT0HI
#define IR_SENSOR_2 BIT1HI

#define SERVO 1
#define SERVO_WIDTH_MAX 1500
#define SERVO_WIDTH_MIN 590
#define SERVO_WIDTH_INIT 1485
#define SERVO_TIME 12
#define SERVO_DELTA 10

#define NONE 0
//Combined Sensor Options
#define BOTH 1
#define RIGHT 2
#define LEFT 3
//Single Sensor Options
#define BOT 1
#define GOAL 2

#define BOT_FREQ 1250 
#define BOT_THRESH_LO 240
#define BOT_THRESH_HI 320
#define GOAL_FREQ 2083
#define GOAL_THRESH_LO 425
#define GOAL_THRESH_HI 515 

/*---------------------------- Module Functions ---------------------------*/
static void UpdateServoWidth(IR_State_t CurrentState);

/*---------------------------- Module Variables ---------------------------*/
static IR_State_t CurrentState;
static uint8_t MyPriority;

static unsigned int TargetFreq;
static unsigned int CurrentServoWidth;
static int DeltaWidth;

/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitIR_Detect

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
bool InitIR_Detect( uint8_t Priority )
{
   MyPriority = Priority;
   CurrentServoWidth = SERVO_WIDTH_INIT;
   DeltaWidth = SERVO_DELTA;

   SetServo(SERVO, CurrentServoWidth);
   CurrentState = DeActivated; //Initial State
   ES_Timer_InitTimer(IR_Detect_Timer, SERVO_TIME);//Start timer
   TargetFreq = BOT_FREQ; //Target Frequency
   ADS12_Init("AAAAAAAA");//Setup analog inputs
  
   return true;
}

/****************************************************************************
 Function
     PostIR_Dectect

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
bool PostIR_Detect( ES_Event ThisEvent )
{
   return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunIR_Detect

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   To detect targets, two IR sensors are used side by side. Goal is to use
   servo to rotate until both sensor can sense the target. If only right 
   sensor sees target, it rotates to the left. 

 Notes
   uses nested switch/case to implement the machine.

 Author
   Patrick Sherman,   02/17/14, 09:45 
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunIR_Detect( ES_Event ThisEvent )
{
   static bool shootflag = false;
   ES_Event ReturnEvent, NewEvent;
   ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
   


   if(CurrentState != DeActivated)
   {
      switch (ThisEvent.EventType)
      {    
         case (StopAligning): //Deactive Align Mode
            CurrentState = DeActivated;
            TargetFreq = 0;
            if(ThisEvent.EventParam == 1)
            {
               CurrentServoWidth = SERVO_WIDTH_MIN; 
            }
            else
            {
               CurrentServoWidth = SERVO_WIDTH_INIT;  
            }
            SetServo(SERVO, CurrentServoWidth);
            break;

         case (ES_TIMEOUT): 	//Timeout - Keep turning 
            if(ThisEvent.EventParam == IR_Detect_Timer)
            {
               UpdateServoWidth(CurrentState);
               SetServo(SERVO, CurrentServoWidth);
               ES_Timer_InitTimer(IR_Detect_Timer, SERVO_TIME);  
            }
            if(ThisEvent.EventParam == ShootBotTimer)
            {
               rightMotor(75);
               leftMotor(-74);
            }
				break;

         case (LeftOnly):	//Left Sensor sees beacon
            ES_Timer_StartTimer(IR_Detect_Timer);
            CurrentState = LeftAligned;
            break;

         case (RightOnly): //Right Sensor Sees beacon                     
            ES_Timer_StartTimer(IR_Detect_Timer);
            CurrentState = RightAligned;                  
            break;

         case (SenseBoth): //Both Sensors See beacon
            CurrentState = Aligned;
            ES_Timer_StopTimer(IR_Detect_Timer);
				//Aligned: Shoot or Deploy Lance
            if(TargetFreq == BOT_FREQ)
            {
               NewEvent.EventType = Deploy_Lance;
               PostLance(NewEvent);

               if(GetCurrentRound() == 3 && shootflag == false)
               {
					   translateMotor(0);
					   shootflag = true;
                  NewEvent.EventType = Shoot_Ball;
                  NewEvent.EventParam = 5;
                  PostShoot(NewEvent);
                  ES_Timer_InitTimer(ShootBotTimer, 3000);				
               }
				}	
				break;

         case (SenseNone): //Neither Sensor sees beacon
            CurrentState = Active;
            ES_Timer_StartTimer(IR_Detect_Timer);
            break;
				
         case (StartAlign): //Start Align reposted
            TargetFreq = ThisEvent.EventParam;
            CurrentState = Active;
            ES_Timer_InitTimer(IR_Detect_Timer, SERVO_TIME);
            break;
				   
      } /* End Switch(EventType) */

   } 
   else //Current State is Deactivated
   { 
      if(ThisEvent.EventType == StartAlign)
      {
         TargetFreq = ThisEvent.EventParam;
         CurrentState = Active;
         ES_Timer_InitTimer(IR_Detect_Timer, SERVO_TIME);
      }
   }
   
   return ReturnEvent;
}

/****************************************************************************
 Function
     QueryTemplateSM

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
IR_State_t QueryIR_Detect ( void )
{
   return(CurrentState);
}

/****************************************************************************
 Function
	CheckIRSensor     

 Description
   Event checker to see if the photo-transistors are detecting a signal.
	The input to the A/D converter is an analog voltage that is proportional
	to the frequency of the signal seen by the IR detector.

 Author
     Patrick Sherman, 02/19/2014, 12:43
****************************************************************************/
bool CheckIRSensor(void)
{
   bool ReturnVal = false;
   ES_Event NewEvent;

   static short LastLeftState = NONE;
   static short LastRightState = NONE;
   static short LastCombinedState = NONE;
	
   unsigned int leftFreq, rightFreq;
   short leftState, rightState, CombinedState;	
   short leftPin = ADS12_ReadADPin(0);
   short rightPin = ADS12_ReadADPin(1);
	
	//What does Left IR Sensor See
   if(leftPin >= BOT_THRESH_LO && leftPin <= BOT_THRESH_HI)
   {
      leftState = BOT;
      leftFreq = BOT_FREQ;	
	} 
   else if(leftPin >= GOAL_THRESH_LO && leftPin <= GOAL_THRESH_HI)
   {
      leftState = GOAL;
      leftFreq = GOAL_FREQ;	
	}
   else 
   {
		leftState = NONE;
		leftFreq = 0;
	}
	
	//What Does Right IR Sensor See
   if(rightPin >= BOT_THRESH_LO && rightPin <= BOT_THRESH_HI)
   {
      rightState = BOT;
      rightFreq = BOT_FREQ;
	} 
   else if(rightPin >= GOAL_THRESH_LO && rightPin <= GOAL_THRESH_HI)
   {
      rightState = GOAL;
      rightFreq = GOAL_FREQ;
	} 
   else
   {
      rightState = NONE;
      rightFreq = 0;	
   }

   //Look at state at both sensor to see if one/both/none see target
   if(rightState == leftState && rightFreq == TargetFreq)
		CombinedState = BOTH;
   else if (leftState != NONE && leftFreq == TargetFreq)
	   CombinedState = LEFT;
   else if (rightState != NONE && rightFreq == TargetFreq)
      CombinedState = RIGHT;
   else
	   CombinedState = NONE;  

   //Use combined state to choose next event	
   if(CombinedState != LastCombinedState)
   {
      switch(CombinedState)
      {
         case (BOTH):
            NewEvent.EventType = SenseBoth;
            NewEvent.EventParam = leftFreq;
            break;
	       
         case(LEFT):
            NewEvent.EventType = LeftOnly;
            NewEvent.EventParam = leftFreq;
            break;
	         
         case(RIGHT):
            NewEvent.EventType = RightOnly;
            NewEvent.EventParam = rightFreq;
            break;
	         
         case(NONE):
            NewEvent.EventType = SenseNone;
            NewEvent.EventParam = 0;
            break;
      }
      
      PostIR_Detect(NewEvent);
      ReturnVal = true;
	}

   LastRightState = rightState;
   LastLeftState = leftState;
   LastCombinedState = CombinedState;

   return ReturnVal;
} /* End CheckIRSensor */

/***************************************************************************
 private functions
 ***************************************************************************/
/****************************************************************************
 Function
	UdateServoWidth

 Description
	Adjusts the servo width based on the state of the IR sensors. The goal
   is for the robot to be able to turn in the correct direction based on
	which IR sensors is currently detecting the beacon.	

 Author
     Patrick Sherman, 02/19/2014, 18:43
****************************************************************************/
static void UpdateServoWidth(IR_State_t CurrentState)
{
   //If only one beacon is detecting target
   if(LeftAligned == CurrentState)
      DeltaWidth = -SERVO_DELTA;
	else if (RightAligned == CurrentState)
      DeltaWidth = SERVO_DELTA;
	
   //Update Servo Width
   CurrentServoWidth += DeltaWidth;
	
   //If width reaches limit of servo.
   if(CurrentServoWidth >= SERVO_WIDTH_MAX)
   {
		DeltaWidth = -SERVO_DELTA;
      CurrentServoWidth = SERVO_WIDTH_MAX;
   } 
   else if (CurrentServoWidth <= SERVO_WIDTH_MIN)
   {
      DeltaWidth = SERVO_DELTA; 
      CurrentServoWidth = SERVO_WIDTH_MIN;
   }

} /* End UpdateServoWidth */

/****************** End of File **********************/
