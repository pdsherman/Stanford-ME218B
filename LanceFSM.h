/****************************************************************************
 
  Header file for template Flat Sate Machine 
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef Lance_H
#define Lance_H

// Event Definitions
#include "ES_Configure.h" /* gets us event definitions */
#include "ES_Types.h"     /* gets bool type for returns */

// typedefs for the states
// State definitions for use with the query function
typedef enum {Deployed, Retracted, Inactive} LanceState_t ;


// Public Function Prototypes

bool InitLance ( uint8_t Priority );
bool PostLance( ES_Event ThisEvent );
ES_Event RunLance( ES_Event ThisEvent );
LanceState_t QueryLance ( void );


#endif /* Lance_H */

