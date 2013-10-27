// This file is published under http://opensource.org/licenses/mit-license.php
// 2013-10-27 Paul Hampson

#include "JeelinkHaRF12ConfControl.h"
#include "JeelinkHa.h"
#include <assert.h>

static radio_cfg prev_cfg = DEFAULT_CFG;
static radio_cfg curr_cfg = DEFAULT_CFG;

void jeelinkha_confctrl_set( radio_cfg cfg )
{
    assert(cfg < MAX_RADIO_CFG);
    
    /* don't reconfigure if the new one is the same as the last */
    if (curr_cfg == cfg) return; 
    
    /* save history */    
    prev_cfg = curr_cfg;
    curr_cfg = cfg;
    
    /* clear out to defaults */
    rf12_initialize(1, RF12_868MHZ, 0xD4);
    
    /* set new cfg */
    switch (cfg)
    {
        case DEFAULT_CFG:            
            break;
        case SALUS_RT500RF_CFG:
            rf12_control(RF12_DATA_RATE_2); // 2.4kbps
            rf12_control(0xA674); // 868.260khz
            rf12_control(0x9840); // 75khz freq shift
            break;
        case LWRF_868_CFG:
            break;
    }
}

void jeelinkha_confctrl_restore( )
{
    jeelinkha_confctrl_set( prev_cfg );
}

void jeelinkha_confctrl_init( radio_cfg cfg )
{
    jeelinkha_confctrl_set( cfg );
    
    /* now override history to initial config */
    prev_cfg = cfg;
}