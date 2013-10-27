// This file is published under http://opensource.org/licenses/mit-license.php
// 2013-10-27 Paul Hampson

#ifndef JeelinkHaRF12ConfControl_h
#define JeelinkHaRF12ConfControl_h

/// @file
/// RFM12B Configuration Controller 

typedef enum {
    DEFAULT_CFG,
    SALUS_RT500RF_CFG,
    LWRF_868_CFG,
    MAX_RADIO_CFG
} radio_cfg;

/// @details
/// Set initial radio configuration value - only call once at startup, erases history
void jeelinkha_confctrl_init( radio_cfg cfg );

/// @details
/// Set radio configuration value
void jeelinkha_confctrl_set( radio_cfg cfg );

/// @details
/// Restore radio configuration value to previous
void jeelinkha_confctrl_restore( );

#endif