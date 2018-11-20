// Copyright 2018 Silicon Laboratories, Inc.
//
//

/***************************************************************************//**
 * @file rail_config.h
 * @brief RAIL Configuration
 * @copyright Copyright 2015 Silicon Laboratories, Inc. http://www.silabs.com
 ******************************************************************************/
// =============================================================================
//
//  WARNING: Auto-Generated Radio Config Header  -  DO NOT EDIT
//  Radio Configurator Version: 3.4.3
//  RAIL Adapter Version: 2.0.7
//  RAIL Compatibility: 1.x
//
// =============================================================================

#ifndef __RAIL_CONFIG_H__
#define __RAIL_CONFIG_H__

#include <stdint.h>
#include "rail_types.h"

extern const uint32_t generated[];

extern const uint32_t *configList[];
extern const char *configNames[];
extern const uint8_t irCalConfig[];

#define NUM_RAIL_CONFIGS 1
extern RAIL_ChannelConfigEntry_t generated_channels[];
extern const RAIL_ChannelConfig_t generated_channelConfig;
extern const RAIL_ChannelConfig_t *channelConfigs[];
extern RAIL_FrameType_t *frameTypeConfigList[];

extern const uint32_t generated_phyInfo[];

#define RADIO_CONFIG_BASE_FREQUENCY 169000000UL
#define RADIO_CONFIG_XTAL_FREQUENCY 38400000UL
#define RADIO_CONFIG_BITRATE "2.4kbps"
#define RADIO_CONFIG_MODULATION_TYPE "FSK2"
#define RADIO_CONFIG_DEVIATION "1.2kHz"
#define RADIO_CONFIG_SYNTHESIZER_RESOLUTION "4.8828125"

#endif // __RAIL_CONFIG_H__


//        _  _                          
//       | )/ )         Wireless        
//    \\ |//,' __       Application     
//    (")(_)-"()))=-    Software        
//       (\\            Platform        
