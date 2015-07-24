/****************************************************************************
 
  Header file for IR Dectection Flat Sate Machine 
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef IR_Detect_H
#define IR_Detect_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum {Aligned, LeftAligned, RightAligned, Active, DeActivated} IR_State_t ;


// Public Function Prototypes
bool CheckIRSensor(void);
bool InitIR_Detect ( uint8_t Priority );
bool PostIR_Detect( ES_Event ThisEvent );
ES_Event RunIR_Detect( ES_Event ThisEvent );
IR_State_t QueryIR_Detect( void );


//For Testing IR Emitter
void ChangeTestFreq(int flag);

#endif /* IR_Detect_H*/

