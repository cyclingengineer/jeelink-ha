// This file is published under http://opensource.org/licenses/mit-license.php
// 2013-10-27 Paul Hampson

#ifndef JeelinkHaSalusRT500_h
#define JeelinkHaSalusRT500_h

#include "JeelinkHa.h"

/// @details
/// Turn Salus 500 boiler controller On/Off. 
/// @param state Boiler state. If state != 0 then state is ON.
void jeelinkha_salusrt500_onOff( uint8_t state );

#endif