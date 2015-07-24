/****************************************************************************
 
  Header file for Step service 
  based on the Gen 2 Events and Services Framework

 ****************************************************************************/

#ifndef DCMotor_H
#define DCMotor_H

#include "ES_Types.h"

// Public Function Prototypes
bool Check4Encoder1(void);

bool InitDCMotor ( uint8_t Priority );
bool PostDCMotor( ES_Event ThisEvent );
ES_Event RunDCMotor( ES_Event ThisEvent );
void leftMotor(signed int PWMDCdir);
void rightMotor(signed int PWMDCdir);

void InitializeTimer(void);    
void angleMotor(unsigned int Deg, unsigned int RPM5);
void rotateMotor(signed int RPMdir1);
void translateMotor(signed int RPMdir);
void positionMotor(unsigned int distInInches, unsigned int RPM);
void timedTranslate(int RPM, unsigned int move_time);

#endif /* DC_Motor_H */
