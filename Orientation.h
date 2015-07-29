/****************************************************************************
 
  Header file for service to determine what color
  on the gameboard the robot is standing above.

 ****************************************************************************/

#ifndef Orientation_H
#define Orientation_H

#include "ES_Configure.h"
#include "ES_Types.h"

// Public Function Prototypes
bool InitOrientation ( uint8_t Priority );
bool PostOrientation( ES_Event ThisEvent );
ES_Event RunOrientation( ES_Event ThisEvent );

// Event Checker
bool Check4RightTape(void);

#endif /* ServTemplate_H */

