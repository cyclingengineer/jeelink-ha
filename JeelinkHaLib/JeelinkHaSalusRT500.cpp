// This file is published under http://opensource.org/licenses/mit-license.php
// 2013-10-27 Paul Hampson

#include "JeelinkHaSalusRT500.h"
#include "JeelinkHaRF12ConfControl.h"

#define SALUS_CMD_LEN_BYTES         4
#define SALUS_CMD_REPEAT_COUNT      30
#define SALUS_CMD_INTER_PKT_GAP_MS  30

static uint8_t salus_addr123_boiler_on_buf[SALUS_CMD_LEN_BYTES]   = { 0x18, 0x01, 0x19, 0x5A };
static uint8_t salus_addr123_boiler_off_buf[SALUS_CMD_LEN_BYTES] = { 0x18, 0x02, 0x1A, 0x5A };


void jeelinkha_salusrt500_onOff( uint8_t state )
{
    uint8_t *data = NULL;
    
    /* configure radio */
    jeelinkha_confctrl_set( SALUS_RT500RF_CFG );
    
    if (state)
        data = (uint8_t *)&salus_addr123_boiler_on_buf;
    else
        data = (uint8_t *)&salus_addr123_boiler_off_buf;
        
    for (int n=0; n < SALUS_CMD_REPEAT_COUNT; n++)
    {
        rf12_sendNow(data, SALUS_CMD_LEN_BYTES);
        delay(SALUS_CMD_INTER_PKT_GAP_MS);
    }
    
    /* superfluous for now - no other modes*/
    //jeelinkha_confctrl_restore( )
}