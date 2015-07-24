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
#include "Bot.h"
#include <hidef.h>
#include <mc9s12e128.h>     /* derivative information for the E128 */
#include <S12E128bits.h>    /* bit definitions for the E128 */
#include <Bin_Const.h>
#include <termio.h>
#include "S12eVec.h"

/*----------------------------- Module Defines ----------------------------*/
// these times assume a 1.024mS/tick timing
#define ONE_SEC 976
#define HALF_SEC (ONE_SEC/2)
#define TWO_SEC (ONE_SEC*2)
#define FIVE_SEC (ONE_SEC*5)


/*---------------------------- Module Functions ---------------------------*/
/* prototypes for private functions for this service.They should be functions
   relevant to the behavior of this service
*/
static void InitSPI(void);
static void InitVariables(void);
//static bool QueryCommand(void);
//static bool RecieveCommand(unsigned char j);
/*---------------------------- Module Variables ---------------------------*/
// with the introduction of Gen2, we need a module level Priority variable
static uint8_t MyPriority;
static unsigned char ByteInfo;
static unsigned int ReloadStatus=0;
static unsigned int RELOAD_STATUS = DARK_RELOAD_STATUS;
static unsigned int HEAD_STATUS; 
static bool DontChangeKnightFlag = false;

static unsigned char HeadStatus;
static unsigned char command1;
static unsigned char command2;
static unsigned char LastCommand1 = 0xFF;
static unsigned char LastCommand2 = 0xFF;
static unsigned char QueryCommand;
static unsigned int i = 0;
static unsigned int j = 0;
static unsigned char readFrom = 4; // The default is for Query for status

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
bool InitJSRcommand ( uint8_t Priority )
{
  ES_Event ThisEvent;

  MyPriority = Priority;
  /********************************************
   in here you write your initialization code
   *******************************************/
  // post the initial transition event
  DDRS |= BIT7HI; // To control SS line manually
  RED_DARK_ADDRESS &= ~RED_DARK_PIN; // Set pin as input
  RED_DARK_PORT &= ~RED_DARK_PIN; //Set button low
  InitSPI();
  readFrom = 4;// Looking for the 4th command from JSR
  InitVariables();
  QueryCommand =  STATUS_QUERY;
  ES_Timer_InitTimer(JSRtimer, 3); // Start JSRtimer for 2ms.
 
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
bool PostJSRcommand( ES_Event ThisEvent )
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
ES_Event RunJSRcommand( ES_Event ThisEvent )
{
  static unsigned char dummy;
  ES_Event ReturnEvent;
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
  
  //printf("RELOAD_STATUS = %u \n\r", RELOAD_STATUS);
  

  switch (ThisEvent.EventType){
  
    case QUERY4STATUS:
        readFrom = 4;// Looking for the 4th command from JSR
        InitVariables();
        QueryCommand =  STATUS_QUERY;
        
    break;
    
    case QUERY4SCORE :
        readFrom = 3;// Looking for the 3rd and 4th command from JSR
        InitVariables();
        QueryCommand =  SCORE_QUERY;
    break;
        
    case ES_TIMEOUT :  // re-start timer & announce
         if ((SPISR & _S12_SPTEF) == _S12_SPTEF)  //if query can be sent
    		{
    			i++;
    			if (i==1)
    			{ 
    			   PTS &= BIT7LO;  // Set SS line low to recieve 4 bytes
    			   SPIDR = QueryCommand; //Send Status Query
    			   //printf("Querycommand Sent!! \r\n");
    			} else
    			{
    			  	SPIDR = 0x00;
    			   //printf("0x00 Sent!!   i =  %d\r\n",i);
    			}
    			//printf("SPTEF SET\r\n");		
    		} 

    break;
    
   case FLAGSET:
      if ((SPISR & _S12_SPIF) == _S12_SPIF) //If Data can be received
        	{
        		 ByteInfo = SPIDR;
        		 dummy = (BIT0HI|BIT1HI|BIT2HI)&(ByteInfo) ;  //Reading Bit 0,1,2
        		 
        		 if (i<readFrom)
        		 {
        		    //dummy = SPIDR;
        		    //printf("---DummyCommand = %d,    i = %d\r\n",dummy, i);
        		    
        		 }
        		 else if (i == readFrom)
        		 {
        		    HeadStatus = ByteInfo & HEAD_STATUS;
        		    ReloadStatus = ByteInfo & RELOAD_STATUS;
        		    command1 = dummy;//SPIDR;
        		    //printf("ByteInfo = %d,  ReloadStatus = %d,  command1 = %d \r\n",ByteInfo, ReloadStatus, command1);
        		    //printf("*******   Command1 = %d,    i = %d    *******\r\n",command1, i);
        		    if (command1 != LastCommand1)
            		 {
            		   	ES_Event ThisEvent;
            		      LastCommand1 = command1;
            		   	//printf("*******   Command1 = %#x,    i = %d    *******\r\n",command1, i);
            		      printf("ByteInfo = %d,  ReloadStatus = %d,  command1 = %d \r\n",ByteInfo, ReloadStatus, command1);
            		   	ThisEvent.EventType = NEW_COMMAND_RECEIVED;
            		   	ThisEvent.EventParam = command1;
            		   	PostBot(ThisEvent);
                	}
        		 }
        		 else if (i > readFrom)
        		 {
        		    command2 = dummy;//SPIDR;
        		    //printf("**************   Command2 = %#x.    i = %d ***********\r\n",command2,i);
        		    if (command2 != LastCommand2)
            		 {
            		   	ES_Event ThisEvent;
            		    LastCommand2 = command2;
            		   //	printf("**************   Command2 = %d.    i = %d \r\n   ***********",command2,i);
            		   //   printf("Reload Status = %d \r\n",ReloadStatus);
            		   	ThisEvent.EventType = NEW_COMMAND_RECEIVED;
            		   	ThisEvent.EventParam = command2;
            		    PostBot(ThisEvent);
                	}
        		 }
        		 
        		if (ES_Timer_InitTimer(JSRtimer, 3) == ES_Timer_OK) // Start JSRtimer 3ms
            	{ 	//puts(" initialized successfully!\n\r");
            	} 
        		if (i==4){i=0;PTS |= BIT7HI;}  // Set SS line high after receiving 4 bytes
           }
    break;
   
    }
    return ReturnEvent;
}

/***************************************************************************
 private functions
 ***************************************************************************/
static void InitSPI(void) {
    /* //Enable SPI, as Master, even rising edges, manual control for SS line */
    SPICR1 = (_S12_SPE | _S12_MSTR | _S12_CPHA | _S12_CPOL); // | _S12_SSOE);   
    SPICR2 &= (~_S12_MODFEN);  //Setting for manual control for SS line
    SPIBR = (_S12_SPPR2 |_S12_SPPR1 | _S12_SPPR0 | _S12_SPR1 |_S12_SPR0); //Set baud rate as , which is smaller than 1.43 MHz (based on Command Genrator Specs (High and Low).
}

static void InitVariables(void)
{
    i = 0;
    j = 0;
    LastCommand1 = 0xFF;
    LastCommand2 = 0xFF;
}
    
    
unsigned int GetReloadStatus(void)
{
   printf("ReloadStatus = %u\n\r", ReloadStatus);
   return ReloadStatus;
}

/*------------------------------- Footnotes -------------------------------*/
/*------------------------------ End of file ------------------------------*/

