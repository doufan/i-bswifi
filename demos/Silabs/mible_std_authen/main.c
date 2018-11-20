/***********************************************************************************************//**
 * \file   main.c
 * \brief  Silicon Labs Empty Example Project
 *
 * This example demonstrates the bare minimum needed for a Blue Gecko C application
 * that allows Over-the-Air Device Firmware Upgrading (OTA DFU). The application
 * starts advertising after boot and restarts advertising after a connection is closed.
 ***************************************************************************************************
 * <b> (C) Copyright 2016 Silicon Labs, http://www.silabs.com</b>
 ***************************************************************************************************
 * This file is licensed under the Silabs License Agreement. See the file
 * "Silabs_License_Agreement.txt" for details. Before using this software for
 * any purpose, you must agree to the terms of that agreement.
 **************************************************************************************************/

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"

/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"

/* Device initialization header */
#include "hal-config.h"

#if defined(HAL_CONFIG)
#include "bsphalconfig.h"
#else
#include "bspconfig.h"
#endif

/*mible std authen*/
#include "mible_type.h"
#include "mible_server.h"
#include "mible_beacon.h"

/*uart driver*/
#include "uartdrv.h"
#include "sleep.h"
//#include "ncp_usart.h"
#include "app_uart.h"
#include "em_core.h"
/***********************************************************************************************//**
 * @addtogroup Application
 * @{
 **************************************************************************************************/

/***********************************************************************************************//**
 * @addtogroup app
 * @{
 **************************************************************************************************/

#ifndef MAX_CONNECTIONS
#define MAX_CONNECTIONS 1
#endif
uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS)];

// Gecko configuration parameters (see gecko_configuration.h)
static const gecko_configuration_t config = {
  .config_flags = 0,
  .sleep.flags = 0,//SLEEP_FLAGS_DEEP_SLEEP_ENABLE,
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap),
  .bluetooth.sleep_clock_accuracy = 100, // ppm
  .gattdb = &bg_gattdb_data,
  .max_timers = 16,
#if (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
  .pa.config_enable = 1, // Enable high power PA
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#endif // (HAL_PA_ENABLE) && defined(FEATURE_PA_HIGH_POWER)
};

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;
//

device_info dev_info = {
  .bonding = WEAK_BONDING, // can be modified according to product
  .pid = 0x03FE,//0x036A, // product id, can be modified according to product
  .version = "0000",	// can be modified according to product
};


static void advertising_init(void);
 void advertising_start(void);


/**
 * @brief  Main function
 */
void main(void)
{
  // Initialize device
  initMcu();
  // Initialize board
  initBoard();
  // Initialize application
  initApp();

  // Initialize stack
  gecko_init(&config);

//  SLEEP_Init_t sleepConfig = {
//     .sleepCallback = beforeSleep,
//     .wakeupCallback = afterWakeup
//   };
  // SLEEP_InitEx(&sleepConfig);
//  SLEEP_SleepBlockBegin(sleepEM2);  // do not allow EM2 or higher

  // NCP USART init
//  ncp_usart_init();
  app_uart_init();
  SLEEP_SleepBlockBegin(sleepEM2);  // do not allow EM2 or higher

  while (1) {
    /* Event pointer for handling events */
    struct gecko_cmd_packet* evt;

    /* Check for stack event. */
    evt = gecko_wait_event();

    app_handle_event(evt);

    mible_stack_event_handler(evt);
    mible_tasks_exec();
    /* Handle events */
    switch (BGLIB_MSG_ID(evt->header)) {
      /* This boot event is generated when the system boots up after reset.
       * Do not call any stack commands before receiving the boot event.
       * Here the system is set to start advertising immediately after boot procedure. */
      case gecko_evt_system_boot_id:

        /* Set advertising parameters. 100ms advertisement interval.
         * The first parameter is advertising set handle
         * The next two parameters are minimum and maximum advertising interval, both in
         * units of (milliseconds * 1.6).
         * The last two parameters are duration and maxevents left as default. */
    	mible_server_info_init(&dev_info);
    	mible_server_miservice_init();

        //gecko_cmd_le_gap_set_advertise_timing(0, 160, 160, 0, 0);
        /* Start general advertising and enable connections. */
        //gecko_cmd_le_gap_start_advertising(0, le_gap_general_discoverable, le_gap_connectable_scannable);
        advertising_init();
        advertising_start();
        break;
      default: break;
    }
  }
}
static void advertising_init(void)
{
  MI_LOG_INFO("advertising init...\n");
  // MI adv and resp data
  mibeacon_frame_ctrl_t frame_ctrl = {
    .time_protocol = 0,
    .is_encrypt = 0,
    .mac_include = 1,
    .cap_include = 1,
    .obj_include = 0,
    .bond_confirm = 0,
    .version = 0x03,
  };
  mibeacon_capability_t cap = {
    .connectable = 1,
    .encryptable = 1,
    .bondAbility = 1
  };
  mible_addr_t dev_mac;
  mible_gap_address_get(dev_mac);
  mibeacon_config_t mibeacon_cfg = {
    .frame_ctrl = frame_ctrl,
    .pid =dev_info.pid,
    .p_mac = (mible_addr_t*)dev_mac,
    .p_capability = &cap,
    .p_obj = NULL,
  };

  uint8_t service_data[31];
  uint8_t service_data_len=0;

  if(MI_SUCCESS != mible_service_data_set(&mibeacon_cfg, service_data, &service_data_len)){
    MI_LOG_ERROR("mibeacon_data_set failed. \r\n");
    return;
  }
  uint8_t adv_data[23]={0};
  uint8_t adv_len=0;
  // add flags
  adv_data[0] = 0x02;
  adv_data[1] = 0x01;
  adv_data[2] = 0x06;
  memcpy(adv_data+3, service_data, service_data_len);
  adv_len = service_data_len + 3;
  int ret,i;
  for(i=0;i<31;i++)
    printf("adv_data[%d] = 0x%x\r\n",i,adv_data[i]);
  printf("adv_len = %d\r\n",adv_len);
  //advertising_start();
  ret = mible_gap_adv_data_set(adv_data,adv_len,NULL,0);
  printf("mible_gap_adv_data_set=%d\r\n",ret);

  MI_LOG_INFO("adv mi service data:");
  MI_LOG_HEXDUMP(adv_data, adv_len);

  // iHealth adv and resp data
  /*// adv data
  uint8_t adv_data[31]={0};
   uint8_t adv_len=0;
   // add flags
   adv_data[adv_len++] = 0x02;
   adv_data[adv_len++] = 0x01;
   adv_data[adv_len++] = 0x06;
   // service UUID
   adv_data[adv_len++] = 3;
   adv_data[adv_len++] = 2;
   adv_data[adv_len++] = 0x11;
   adv_data[adv_len++] = 0xFF;
   // manufacture data
   adv_data[adv_len++] = 9;
   adv_data[adv_len++] = 0xFF;
   adv_data[adv_len++] = 0xFF;
   adv_data[adv_len++] = 0x02;
   mible_addr_t dev_mac;
   mible_gap_address_get(dev_mac);
   adv_data[adv_len++] = dev_mac[5];
   adv_data[adv_len++] = dev_mac[4];
   adv_data[adv_len++] = dev_mac[3];
   adv_data[adv_len++] = dev_mac[2];
   adv_data[adv_len++] = dev_mac[1];
   adv_data[adv_len++] = dev_mac[0];
  // scan resp data
  uint8_t scan_resp_data[31] = {0};
  uint8_t scan_resp_len = 0;
  //TX power
  scan_resp_data[scan_resp_len++] = 0x02;
  scan_resp_data[scan_resp_len++] = 0x0A;
  scan_resp_data[scan_resp_len++] = 0;
  //local name
  scan_resp_data[scan_resp_len++] = 0x09;
  scan_resp_data[scan_resp_len++] = 0x09;
  scan_resp_data[scan_resp_len++] = 'i';
  scan_resp_data[scan_resp_len++] = '-';
  scan_resp_data[scan_resp_len++] = 'b';
  scan_resp_data[scan_resp_len++] = 's';
  scan_resp_data[scan_resp_len++] = 'w';
  scan_resp_data[scan_resp_len++] = 'i';
  scan_resp_data[scan_resp_len++] = 'f';
  scan_resp_data[scan_resp_len++] = 'i';
  mible_gap_adv_data_set(adv_data,adv_len,scan_resp_data,scan_resp_len);
  MI_LOG_INFO("adv data:");
  MI_LOG_HEXDUMP(adv_data, adv_len);
  MI_LOG_INFO("scan resp data:");
  MI_LOG_HEXDUMP(scan_resp_data, scan_resp_len);
  */
  return;
}

 void advertising_start(void)
{
	MI_LOG_INFO("advertising start...\n");
  mible_gap_adv_param_t adv_param =(mible_gap_adv_param_t){
    .adv_type = MIBLE_ADV_TYPE_CONNECTABLE_UNDIRECTED,
    .adv_interval_min = 0X0020,
    .adv_interval_max = 0X0020,
    .ch_mask = {0},
  };

  if(MI_SUCCESS != mible_gap_adv_start(&adv_param)){
      MI_LOG_ERROR("mible_gap_adv_start failed. \r\n");
      return;
  }
}

void mible_service_init_cmp(void)
{
  MI_LOG_INFO("mible_service_init_cmp\r\n");
//  MI_LOG_DEBUG("",service_handle.)
}

void mible_connected(void)
{
  MI_LOG_INFO("mible_connected \r\n");
  uint8 buffer[] = {"mible_connected \r\n"};
//  UARTDRV_TransmitB(handle, buffer, sizeof(buffer));
  ble_conn_state = 1;
  ble_state_notify(ble_conn_state);
  app_send_timer_start();
}

void mible_disconnected(void)
{
  MI_LOG_INFO("mible_disconnected \r\n");
  uint8 buffer[] = {"mible_disconnected \r\n"};
//   UARTDRV_TransmitB(handle, buffer, sizeof(buffer));
  advertising_start();
  ble_conn_state = 0;
  ble_state_notify(ble_conn_state);
}

void mible_bonding_evt_callback(mible_bonding_state state)
{
  if(state == BONDING_FAIL){
          MI_LOG_INFO("BONDING_FAIL\r\n");
          uint8 buffer[] = {"BONDING_FAIL \r\n"};
//             UARTDRV_TransmitB(handle, buffer, sizeof(buffer));
          //mible_gap_disconnect(mible_server_connection_handle);
  }else if(state == BONDING_SUCC){
          MI_LOG_INFO("BONDING_SUCC\r\n");
          uint8 buffer[] = {"BONDING_SUCC \r\n"};
//             UARTDRV_TransmitB(handle, buffer, sizeof(buffer));
//          app_send_timer_start();
  }else if(state == LOGIN_FAIL){
          MI_LOG_INFO("LOGIN_FAIL\r\n");
          uint8 buffer[] = {"LOGIN_FAIL \r\n"};
//             UARTDRV_TransmitB(handle, buffer, sizeof(buffer));
          //mible_gap_disconnect(mible_server_connection_handle);
  }else{
          MI_LOG_INFO("LOGIN_SUCC\r\n");
          uint8 buffer[] = {"LOGIN_SUCC \r\n"};
//             UARTDRV_TransmitB(handle, buffer, sizeof(buffer));
//          app_send_timer_start();
  }
  mi_auth_state = state;
}
/** @} (end addtogroup app) */
/** @} (end addtogroup Application) */
