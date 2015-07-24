/****************************************************************************
 
  Header file for template Flat Sate Machine 
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef Bot_H
#define Bot_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum {Waiting, Recess, PasDArmes, SuddenDeath
              } BotState_t ;


// Public Function Prototypes

bool InitBot ( uint8_t Priority );
bool PostBot( ES_Event ThisEvent );
ES_Event RunBot( ES_Event ThisEvent );
BotState_t QueryBot( void );
unsigned char GetCurrentRound(void);


#endif /* Bot_H */

