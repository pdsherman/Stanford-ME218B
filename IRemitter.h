/****************************************************************************
 
  Header file for reloading service 
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef IRemitter_H
#define IRemitter_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */


// Public Function Prototypes
bool InitIRemitter ( uint8_t Priority );
bool PostIRemitter( ES_Event ThisEvent );
ES_Event RunIRemitter( ES_Event ThisEvent );
IRemitterState_t QueryIRemitter ( void );


#endif /* IRemitter_H */

