/****************************************************************************
 Module
   EventCheckers.c

 Revision
   1.0.1 

 Description
   This is the sample for writing event checkers along with the event 
   checkers used in the basic framework test harness.

 Notes
   Note the use of static variables in sample event checker to detect
   ONLY transitions.
   
 History
 When           Who     What/Why
 -------------- ---     --------
 08/06/13 13:36 jec     initial version
****************************************************************************/

// this will pull in the symbolic definitions for events, which we will want
// to post in response to detecting events
#include "ES_Configure.h"
// this will get us the structure definition for events, which we will need
// in order to post events in response to detecting events
#include "ES_Events.h"
// if you want to use distribution lists then you need those function 
// definitions too.
#include "ES_PostList.h"       
// This include will pull in all of the headers from the service modules
// providing the prototypes for all of the post functions
#include "ES_ServiceHeaders.h"
// this test harness for the framework references the serial routines that
// are defined in ES_Port.c
#include "ES_Port.h"
// include our own prototypes to insure consistency between header & 
// actual functionsdefinition
#include "EventCheckers.h"

#include "Bot.h"
#include "Servos.h"
#include "Ultrasonic.h"
#include "Orientation.h"
#include "DCMotor.h"
#include "ADS12.h"
#include "JSRcommand.h"

#include <hidef.h>
#include <mc9s12e128.h>     /* derivative information for the E128 */
#include <S12E128bits.h>    /* bit definitions for the E128 */
#include <Bin_Const.h>
#include <termio.h>
#include "S12eVec.h"

//#define START_ROUND  0x01
//#define RECESS       0x03a
//#define END	         0x05
//#define SWITCH_ADDRESS DDRT
#define SWITCH_PORT PTIAD
#define FRONT BIT1HI
#define BACK BIT4HI


/****************************************************************************
 Function
   Check4Flag
 Parameters
   
 Returns
   bool: true if a new event was detected
 Description
   Sample event checker for JSR with ME128B Project 
 Notes
   will not compile, sample only
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Flag (void)
{
   ES_Event ThisEvent;
   
   if ((SPISR & _S12_SPIF) == _S12_SPIF)
   {
      ThisEvent.EventType = FLAGSET;
      PostJSRcommand(ThisEvent);
      return true;
   }
   return false;
}


/****************************************************************************
 Function
   Check4Keystroke
 Parameters
   None
 Returns
   bool: true if a new key was detected & posted
 Description
   checks to see if a new key from the keyboard is detected and, if so, 
   retrieves the key and posts an ES_NewKey event to TestHarnessService1
 Notes
   The functions that actually check the serial hardware for characters
   and retrieve them are assumed to be in ES_Port.c
   Since we always retrieve the keystroke when we detect it, thus clearing the
   hardware flag that indicates that a new key is ready this event checker 
   will only generate events on the arrival of new characters, even though we
   do not internally keep track of the last keystroke that we retrieved.
 Author
   J. Edward Carryer, 08/06/13, 13:48
****************************************************************************/
bool Check4Keystroke(void)
{
  if ( IsNewKeyReady() ) // new key waiting?
  {
    ES_Event ThisEvent;
    ES_Event TestEvent;
    ThisEvent.EventType = ES_NEW_KEY;
    ThisEvent.EventParam = GetNewKey();
    // test distribution list functionality by sending the 'L' key out via
    // a distribution list.
    //printf("ES_NEW_KEY: received ->%c<- in Service \r\n", ThisEvent.EventParam);
    switch ( ThisEvent.EventParam){
      case '1':
         TestEvent.EventType = QUERY4STATUS;
         TestEvent.EventParam = STATUS_QUERY;
         PostJSRcommand(TestEvent);
         printf("QUERY4STATUS Event posted \r\n");
         break;
      case '2':
         TestEvent.EventType = QUERY4SCORE;
         TestEvent.EventParam = SCORE_QUERY;
         PostJSRcommand(TestEvent);
         printf("QUERY4SCORE Event posted \r\n");
         break;
      case '3':
         TestEvent.EventType = RELOAD_BALLS;
         //TestEvent.EventParam = SCORE_QUERY;
         PostIRemitter(TestEvent);
         printf("RELOAD_BALLS Event posted \r\n");
         break;
         
      //Test Servos
      case '4':
         rightMotor(74);                    
         break;
      case '5':
         rightMotor(76);
         break;
      case '6':
         rightMotor(78);
         break;
      case '7':
         leftMotor(-76);
         break;
      case '8':
         leftMotor(-74);
         break;
      case '9':
         leftMotor(-78);
         break;
         
      case '0':
         TestEvent.EventType = Deploy_Lance;
         PostLance(TestEvent);
         break;

    //Test IR_Detect Code     
      case 'a':
         TestEvent.EventType = StartAlign;
         TestEvent.EventParam = 1250;
         PostIR_Detect(TestEvent);
         break;
         
      case 's':
         TestEvent.EventType = StopAligning;
         PostIR_Detect(TestEvent);
         break;
      
      case 'd':
         TestEvent.EventType = SenseBoth;
         TestEvent.EventParam = 1250;
         PostIR_Detect(TestEvent);
         break;
         
      case 'f':
         TestEvent.EventType = SenseNone;
         PostIR_Detect(TestEvent);
         break;         

      case 'g':
         TestEvent.EventType = LeftOnly;
         TestEvent.EventParam = 1250;
         PostIR_Detect(TestEvent);
         break;
         
      case 'h':
         TestEvent.EventType = RightOnly;
         TestEvent.EventParam = 1250;
         PostIR_Detect(TestEvent);
         break; 
               
      case 'u':
         TestEvent.EventType = StartShootingMotors;
         PostShoot(TestEvent);
         puts("ChenKey");
         break;
         
      case 't':
         TestEvent.EventType = Shoot_Ball;
         PostShoot(TestEvent);
         break;
                                                    
      
      case 'l':
         translateMotor(70);
         printf("angle");
         break;   
      case 'r':
         translateMotor(80);
         printf("angle");
         break;
         
      case 'z':
         translateMotor(60);
         printf("angle");
         break;
      case 'x':
         translateMotor(75);
         //rotateMotor(70);
         printf("translate");
         break;
      case 'c':
         rightMotor(75);
         leftMotor(-74);
         printf("forward\n");
         break;      
      case 'v':
         translateMotor(0);
         //rotateMotor(0);
         printf("stop\n");
         break;
         
      case 'Q':
         TestEvent.EventType = NEW_COMMAND_RECEIVED;
         TestEvent.EventParam = WAIT;
         PostBot(TestEvent);
         break;
      case 'q':
         TestEvent.EventType = NEW_COMMAND_RECEIVED;
         TestEvent.EventParam = START_ROUND;
         PostBot(TestEvent);
         break;
         
      case 'w':
         TestEvent.EventType = NEW_COMMAND_RECEIVED;
         TestEvent.EventParam = RECESS;
         PostBot(TestEvent);
         break;
         
      case 'e':
         TestEvent.EventType = NEW_COMMAND_RECEIVED;
         TestEvent.EventParam = END;
         PostBot(TestEvent);
         break; 
         
         
      case 'A':
         TestEvent.EventType = ES_TIMEOUT;
         TestEvent.EventParam = Pause_Timer;
         PostOrientation(TestEvent);
         break;
               
      default: 
         break;
    }
    
    return true;
  }
  return false;
}
