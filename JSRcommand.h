/****************************************************************************
 
  Header file for Joust Status Reporter (JSR) service
  based on the Gen2 Events and Services Framework

 ****************************************************************************/

#ifndef JSRcommand_H
#define JSRcommand_H


#define STATUS_QUERY 0x3F
#define SCORE_QUERY  0xC3
#define NOTHING      0xFF
#define WAIT         0x00
#define START_ROUND  0x01
#define RECESS       0x03
#define SUDDEN_DEATH 0x04
#define END	         0x05

#define RED_DARK_ADDRESS DDRE
#define RED_DARK_PORT PORTE
#define RED_DARK_PIN BIT0HI

#define RED_RELOAD_STATUS BIT4HI
#define DARK_RELOAD_STATUS BIT6HI

// Event Definitions
#include "ES_Configure.h"
#include "ES_Types.h"

// Public Function Prototype
bool InitJSRcommand( uint8_t Priority );
bool PostJSRcommand( ES_Event ThisEvent );
ES_Event RunJSRcommand( ES_Event ThisEvent );
unsigned int GetReloadStatus(void);


#endif /* JSRcommand_H */

