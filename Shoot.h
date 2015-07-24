/****************************************************************************
 
  Header file for template service 
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef Shoot_H
#define Shoot_H

#include "ES_Types.h"

// Public Function Prototypes

bool InitShoot ( uint8_t Priority );
bool PostShoot( ES_Event ThisEvent );
ES_Event RunShoot( ES_Event ThisEvent );


#endif /* Shoot_H */

