/****************************************************************************
 Module
   DC_Motor.c
                                                                                          
 Revision
   1.0.1

 Description
   This is a template file for implementing a simple service under the 
   Gen2 Events and Services Framework. Service handles the generation of
   PWM Signal for a DC Motor as required by Lab 7. Additionally, handles
   the reading of the encoder and implements a simple PI controller.

 Notes

 History
 When           Who     What/Why
 -------------- ---     --------
 01/25/14 16:09 pds05    Changed file for use with DC Motor for lab 7
 01/10/14 17:26 pds05    Converted template for use in Lab 5
 01/16/12 09:58 jec      began conversion from TemplateFSM.c

****************************************************************************/
/*----------------------------- Include Files -----------------------------*/
/* include header files for this state machine as well as any machines at the
   next lower level in the hierarchy that are sub-machines to this machine
*/
#include "ES_Configure.h"
#include "ES_Framework.h"
#include "ES_DeferRecall.h"
#include "DCMotor.h"


#include <stdio.h>
#include "ADS12.h"
#include <hidef.h>
#include <mc9s12e128.h>     /* derivative information for the E128 */
#include <S12E128bits.h>    /* bit definitions for the E128 */ 
#include <Bin_Const.h>
#include <termio.h>
#include "S12eVec.h"

/*----------------------------- Module Defines ----------------------------*/
#define MOTOR_PORT PTU
#define MOTOR_PORT_ADDRESS DDRU
#define MOTOR_1_PWM BIT0HI
#define MOTOR_1_DIR BIT7HI
#define MOTOR_2_PWM BIT1HI
#define MOTOR_2_DIR BIT6HI

#define ENCODER_PORT PTT
#define ENCODER_PORT_ADDRESS DDRT
#define ENCODER1 BIT0HI
#define ENCODER2 BIT1HI
#define ENCODERCHECK_PORT PTT
#define ENCODERCHECK_PORT_ADDRESS DDRT
#define ENCODERCHECK1 BIT7HI


#define ONE_SEC 976
#define CHCK_SPEED_TIME ONE_SEC/2

#define _ms_ *750
#define Period (60 _ms_)
#define PGain1 0.2
#define IGain1 0.008
#define PGain2 0.2                      
#define IGain2 0.008
#define RPM_Period 200

#define SCALE 8
/*---------------------------- Module Functions ---------------------------*/
void InitializeTimer(void);    
void angleMotor(unsigned int Deg, unsigned int RPM5);
void rotateMotor(signed int RPMdir1);
void translateMotor(signed int RPMdir);
void positionMotor(unsigned int distInInches, unsigned int RPM);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static ES_Event DeferralQueue[3+1];
static unsigned char i=0;

static unsigned long period1=0;
static unsigned long period2=0;
static unsigned long uPeriod1=0;
static unsigned long uPeriod2=0;
static unsigned long LastPeriod1 = 0;
static unsigned long LastPeriod2 = 0;
static unsigned long uLastTime1=0;
static unsigned long uLastTime2=0;
static unsigned int OverFlows1=0;
static unsigned int OverFlows2=0;
static float RPM1=0;
static unsigned char count1=0;
static float ReqRPM1 = 0;
static float RPMError1 = 0;
static float SumError1 = 0;
static float ReqDuty1 = 0;
static float SetDuty1 = 0;
static float LastDuty1 = 0;
static float RPM2=0;
static unsigned char count2=0;
static float ReqRPM2 = 0;
static float RPMError2 = 0;
static float SumError2 = 0;
static float ReqDuty2 = 0;
static float SetDuty2 = 0;
static float LastDuty2 = 0;
static float LastRPM1 = 0;
static float LastRPM2 = 0;
   
/*------------------------------ Module Code ------------------------------*/
/****************************************************************************
 Function
     InitDCMotor

 Parameters
     uint8_t : the priorty of this service

 Returns
     bool, false if error in initialization, true otherwise

 Description
     Saves away the priority, sets all the necessary bits for input capture
     output compare, PWM setup and enabling interrupts.

 Author
     J. Edward Carryer, 01/16/12, 10:00
****************************************************************************/
bool InitDCMotor ( uint8_t Priority )
{
  ES_Event ThisEvent;
  MyPriority = Priority;
  
   
  /* Turn on Port U for PWM outputs */
  MOTOR_PORT_ADDRESS |= (MOTOR_1_DIR | MOTOR_2_DIR | MOTOR_1_PWM | MOTOR_2_PWM);
  
  /* Initialize Registers for PWM Capability */
  PWME |= _S12_PWME0 | _S12_PWME1;    //Enable PWM 0,1
  PWMPOL |= (_S12_PPOL0 | _S12_PPOL1); //Set Output Polarity LO
  PWMCLK  |= (_S12_PCLK0 | _S12_PCLK1);  //Enable Clock A Scaling
  PWMPRCLK &= ~(_S12_PCKA2|_S12_PCKA1|_S12_PCKA0);//Set A Clock prescale /1
  PWMSCLA = 4;         //Clock Scaler
  PWMCAE &= ~_S12_CAE0;  //Set to Left Align
  PWMCAE |= _S12_CAE1;  //Set to Center Align
  MODRR |= _S12_MODRR0 | _S12_MODRR1;  //Port U Mapping - Pin(s) 0,1 to PWM
 
  /* Set Period/Initial Duty Cycle/Initial Dir */ 
  PWMPER0 = 100; //Set PWM freq to 30 kHz
  PWMPER1 = 50;
  PWMDTY0 = 0;
  PWMDTY1 = 0;
  
  MOTOR_PORT &= ~MOTOR_1_DIR & ~MOTOR_2_DIR;
  
  ES_InitDeferralQueueWith( DeferralQueue, ARRAY_SIZE(DeferralQueue) );
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
     PostDCMotor

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
bool PostDCMotor( ES_Event ThisEvent )
{
  return ES_PostToService( MyPriority, ThisEvent);
}

/****************************************************************************
 Function
    RunDCMotor

 Parameters
   ES_Event : the event to process

 Returns
   ES_Event, ES_NO_EVENT if no error ES_ERROR otherwise

 Description
   Updates the current RPM and desired RPM. This update is triggered 10 times a second.
   Changes the direction of the motor turning if button event is posted. 
   
 Author
   P. Sherman,        01/11/14, 14:04 - Converted for use with DC Motor
   J. Edward Carryer, 01/15/12, 15:23
****************************************************************************/
ES_Event RunDCMotor( ES_Event ThisEvent )
{
   ES_Event ReturnEvent;
   
   ReturnEvent.EventType = ES_NO_EVENT; // assume no errors
   //printf("EventType: %d \n\r",ThisEvent.EventType);
   //printf("PWMDTY0: %d \n\r",PWMDTY0);
   switch(ThisEvent.EventType){
   
            
      case MotorRightC:
   			//printf("MotorRightC\r\n");
   			MOTOR_PORT &= ~MOTOR_1_DIR;
   			PWMDTY0 = ThisEvent.EventParam;
   			//ReqRPM1 = ThisEvent.EventParam;
   			break;
   			
      case MotorRightCC:
   			//printf("MotorRightCC\r\n");
   			MOTOR_PORT |= MOTOR_1_DIR;	
   			PWMDTY0 = ThisEvent.EventParam; 
   			//ReqRPM1 = ThisEvent.EventParam;   				
         	break; 
   	
   	case MotorLeftC:
   			MOTOR_PORT &= ~MOTOR_2_DIR;
   			PWMDTY1 = ThisEvent.EventParam/2;
   		   //ReqRPM2 = ThisEvent.EventParam;
   			break;
                                                                    
      case MotorLeftCC: 
   			MOTOR_PORT |= MOTOR_2_DIR;
   			PWMDTY1 = ThisEvent.EventParam/2;
   			//ReqRPM2 = ThisEvent.EventParam;   				
         	break;
         	
      case ES_TIMEOUT:
            if(ThisEvent.EventParam==DC_TIMER){
               printf("DC_TIMER\n\r");
               translateMotor(0);
            }
            break;
   }
  return ReturnEvent;
}
/*-------------------------- Public Functions ---------------------------*/
/* Function: translateMotor
  -------------------------
  Move Bot straight forward/backwards */
void translateMotor(signed int RPMdir){
      leftMotor(-RPMdir);
      rightMotor(RPMdir);   
}

/* Function: rotateMotor
  ------------------------
  Rotate Bot clockwise or counter-clockwise */
void rotateMotor(signed int RPMdir1)
{
   leftMotor(RPMdir1);
   rightMotor(RPMdir1);   
}

/* Function: timedTranslate
  --------------------------
  translate the bot for a specified amount of time */
void timedTranslate(int RPM, unsigned int move_time)
{
   translateMotor(RPM);
   ES_Timer_InitTimer(DC_TIMER, move_time);   
}

void leftMotor(signed int PWMDCdir){
   ES_Event motorEvent;
   
   if (PWMDCdir < 0){
      motorEvent.EventType = MotorLeftC;
      motorEvent.EventParam = -PWMDCdir;
   } else {
      motorEvent.EventType = MotorLeftCC;
      motorEvent.EventParam = PWMDCdir;      
   }
   PostDCMotor(motorEvent); 

}
void rightMotor(signed int PWMDCdir){
   ES_Event motorEvent;
   
   if (PWMDCdir < 0){
      motorEvent.EventType = MotorRightC;
      motorEvent.EventParam = -PWMDCdir;
   } else {
      motorEvent.EventType = MotorRightCC;
      motorEvent.EventParam = PWMDCdir;      
   }
   PostDCMotor(motorEvent);   
}

/*------------------------------ End of file ------------------------------*/

