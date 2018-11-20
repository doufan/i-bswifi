/*
 * app_uart.c
 *
 *  Created on: Oct 18, 2018
 *      Author: root
 */

#include "uartdrv.h"
#include "app_uart.h"
#include "gatt_db.h"
#include "efr32_api.h"

// add by hdf
unsigned int volatile p_in = 0, p_out = 0;//接收循环buffer下标
unsigned char uartRecvBuf[BUFF_SIZE] = {0};
unsigned int volatile ps_in = 0, ps_out = 0;//发送循环buffer下标
unsigned char uartSendBuf[BUFF_SIZE] = {0};
unsigned char seq_id = 0;

// ble conn state
uint8_t ble_conn_state = 0;// 0: disconnect, 1: connected
uint8 ble_conn_handle = DISCONNECTION;
uint16 ble_mtu = 20;
// mi auth state
mible_bonding_state mi_auth_state = BONDING_FAIL;
// ble send buffer
static uint32_t pb_in = 0, pb_out = 0;
static uint8_t send_to_app_array[BUFF_SIZE];

// temporary buffer for receiving data from UART
static uint8_t rxbuf[BUFF_SIZE];

 // Define receive/transmit operation queues
 DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_RX_BUFS, rxBufferQueue);
 DEFINE_BUF_QUEUE(EMDRV_UARTDRV_MAX_CONCURRENT_TX_BUFS, txBufferQueue);
 // Configuration for USART0, location 0
 #define MY_UART                                   \
   {                                               \
     USART0,                                       \
     115200,                                       \
     _USART_ROUTELOC0_TXLOC_LOC0,                  \
     _USART_ROUTELOC0_RXLOC_LOC0,                  \
     usartStopbits1,                               \
     usartNoParity,                                \
     usartOVS16,                                   \
     false,                                        \
     uartdrvFlowControlNone,                       \
     gpioPortA,                                    \
     4,                                            \
     gpioPortA,                                    \
     5,                                            \
     (UARTDRV_Buffer_FifoQueue_t *)&rxBufferQueue, \
     (UARTDRV_Buffer_FifoQueue_t *)&txBufferQueue, \
     _USART_ROUTELOC1_CTSLOC_LOC0,                 \
     _USART_ROUTELOC1_RTSLOC_LOC0                  \
   }
 UARTDRV_HandleData_t handleData;
 UARTDRV_Handle_t handle = &handleData;
 uint8_t buffer[400];

 static void uart_rx_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data, UARTDRV_Count_t transferCount);
 static uint32_t app_usart_transmit(uint8_t* data, uint8_t len);
 static void uart_tx_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data,
                              UARTDRV_Count_t transferCount);
 void APP_USART_IRQ_NAME(void);
 void APP_USART_TX_IRQ_NAME(void);

 static unsigned int ptr_plus(unsigned int ptr, unsigned int off);
 static unsigned int ptr_minus(unsigned int ptr, unsigned int off);
 static unsigned int hex2str( unsigned char *phex, unsigned int len, char *pstr );
 static void praseAppCmd(void);
 static void send_to_app_timeout_handler(void);
 static void send_to_uart_timeout_handler(void);
 static void packDeviceCmd(STRU_BLE_DATA *data);


 void callback(UARTDRV_Handle_t handle,
               Ecode_t transferStatus,
               uint8_t *data,
               UARTDRV_Count_t transferCount)
 {
   (void)handle;
   (void)transferStatus;
   (void)data;
   (void)transferCount;
//   DBG(YELLOW"send 400 bytes OK\r\n"NONE);
 }
 void app_uart_init(void)
 {
   // Initialize driver handle
   UARTDRV_InitUart_t initData = MY_UART;
   UARTDRV_InitUart(handle, &initData);

   CORE_SetNvicRamTableHandler(APP_USART_IRQn, (void *)APP_USART_IRQ_NAME);
   CORE_SetNvicRamTableHandler(APP_USART_TX_IRQn, (void *)APP_USART_TX_IRQ_NAME);
   NVIC_ClearPendingIRQ(APP_USART_IRQn);           // Clear pending RX interrupt flag in NVIC
   NVIC_ClearPendingIRQ(APP_USART_TX_IRQn);        // Clear pending TX interrupt flag in NVIC
   NVIC_EnableIRQ(APP_USART_IRQn);
   NVIC_EnableIRQ(APP_USART_TX_IRQn);

   //Setup RX timeout
     handle->peripheral.uart->TIMECMP1 = USART_TIMECMP1_RESTARTEN
                                         | USART_TIMECMP1_TSTOP_RXACT
                                         | USART_TIMECMP1_TSTART_RXEOF
                                         | (0x30 << _USART_TIMECMP1_TCMPVAL_SHIFT);

   //IRQ
    USART_IntClear(handle->peripheral.uart, _USART_IF_MASK);       // Clear any USART interrupt flags
    USART_IntEnable(handle->peripheral.uart, USART_IF_TXIDLE | USART_IF_TCMP1); // USART_IF_TCMP1    USART_IF_RXDATAV
    /* RX the next command header*/
    UARTDRV_Receive(handle, rxbuf, BUFF_SIZE, uart_rx_callback);// BUFF_SIZE

    gecko_cmd_hardware_set_lazy_soft_timer(TIMER_MS_2_TIMERTICK(20),TIMER_MS_2_TIMERTICK(5),UART_PRASE_CMD,0);
    gecko_cmd_hardware_set_lazy_soft_timer(TIMER_MS_2_TIMERTICK(20),TIMER_MS_2_TIMERTICK(5),UART_SEND,0);
//    gecko_cmd_hardware_set_lazy_soft_timer(TIMER_MS_2_TIMERTICK(5000),TIMER_MS_2_TIMERTICK(5),UART_TEST,0);


//   int i = 0;
//   for(i = 0; i < 400; i++)
//   {
// 	  buffer[i] = i;
//   }
//   // Transmit data using a non-blocking transmit function
//   UARTDRV_Transmit(handle, buffer, 400, callback);
 }

 static void recv(uint8_t* data, uint32_t len)
 {
		uint32_t i = 0;
		for(i = 0; i < len; i++){
			uartRecvBuf[p_in] = data[i];
			p_in++;
			  if(p_in >= BUFF_SIZE)
			  {
				  p_in = 0;
			  }
		  }
 }

 void APP_USART_IRQ_NAME()
 {
  // DBG(L_RED"before.uart->IF: %04X\r\n"NONE,handle->peripheral.uart->IF);
   if(handle->peripheral.uart->IF & USART_IF_FERR)
   {
	  DBG(L_RED"before.uart->IF: %04X\r\n"NONE,handle->peripheral.uart->IF);
 	  USART_IntClear(handle->peripheral.uart, USART_IF_FERR);
 	 //stop the timer
 		     handle->peripheral.uart->TIMECMP1 &= ~_USART_TIMECMP1_TSTART_MASK;
 		     handle->peripheral.uart->TIMECMP1 |= USART_TIMECMP1_TSTART_RXEOF;
 		     USART_IntClear(handle->peripheral.uart, USART_IF_TCMP1);
   }

   if(handle->peripheral.uart->IF & USART_IF_RXOF)
     {
  	  DBG(L_RED"before.uart->IF: %04X\r\n"NONE,handle->peripheral.uart->IF);
   	  USART_IntClear(handle->peripheral.uart, USART_IF_RXOF);
   	 //stop the timer
   		     handle->peripheral.uart->TIMECMP1 &= ~_USART_TIMECMP1_TSTART_MASK;
   		     handle->peripheral.uart->TIMECMP1 |= USART_TIMECMP1_TSTART_RXEOF;
   		     USART_IntClear(handle->peripheral.uart, USART_IF_TCMP1);
     }
   else if (handle->peripheral.uart->IF & USART_IF_TCMP1)
   {
	   //stop the timer
	     handle->peripheral.uart->TIMECMP1 &= ~_USART_TIMECMP1_TSTART_MASK;
	     handle->peripheral.uart->TIMECMP1 |= USART_TIMECMP1_TSTART_RXEOF;
	     USART_IntClear(handle->peripheral.uart, USART_IF_TCMP1);

		uint8_t* buffer = NULL;
		uint32_t received = 0;
		uint32_t remaining = 0;
		UARTDRV_GetReceiveStatus(handle, &buffer, &received, &remaining);
		DBG(YELLOW"recv: %d, %d\r\n"NONE,received,remaining);
		if (buffer && received >= 1) {
			recv(buffer, received);
			//abort receive operation
			DBG(L_RED"111 handle->rxDmaActive: %04X\r\n"NONE,handle->rxDmaActive);
			UARTDRV_Abort(handle, uartdrvAbortReceive);
			DBG(L_RED"222 handle->rxDmaActive: %04X\r\n"NONE,handle->rxDmaActive);
			//enqueue next receive buffer
			Ecode_t ret = 0;
			ret = UARTDRV_Receive(handle, rxbuf, BUFF_SIZE, uart_rx_callback); // BUFF_SIZE
			DBG(YELLOW"ret: %d\r\n"NONE,ret);
			DBG(L_RED"333 handle->rxDmaActive: %04X\r\n"NONE,handle->rxDmaActive);
		}
   }
  // DBG(L_RED"after.uart->IF: %04X\r\n"NONE,handle->peripheral.uart->IF);
 }

 void APP_USART_TX_IRQ_NAME()
 {
// 	DBG(L_RED"TX: before.uart->IF: %04X\r\n"NONE,handle->peripheral.uart->IF);
   if (handle->peripheral.uart->IF & USART_IF_TXIDLE) {
 //    gecko_external_signal(NCP_USART_UPDATE_SIGNAL);
     USART_IntClear(handle->peripheral.uart, USART_IF_TXIDLE);
   }
//   DBG(L_RED"TX: after.uart->IF: %04X\r\n"NONE,handle->peripheral.uart->IF);
 }

 static void uart_rx_callback(UARTDRV_Handle_t handle, Ecode_t transferStatus, uint8_t *data, UARTDRV_Count_t transferCount)
 {
   // handle it
	 DBG(L_RED"uart_rx_callback: transferCount: %d, transferStatus: %04X\r\n"NONE,transferCount,transferStatus);
	 if(transferStatus == ECODE_EMDRV_UARTDRV_OK)// ECODE_EMDRV_UARTDRV_ABORTED
	 {
		 int i = 0;
		 DBG(YELLOW);
		 for(i = 0; i < transferCount; i++)
		 {
			 DBG(" %02X",data[i]);
		 }
		 DBG("\r\n"NONE);
		 recv(data, transferCount);
		 //stop the timer
		 handle->peripheral.uart->TIMECMP1 &= ~_USART_TIMECMP1_TSTART_MASK;
		 handle->peripheral.uart->TIMECMP1 |= USART_TIMECMP1_TSTART_RXEOF;
		 USART_IntClear(handle->peripheral.uart, USART_IF_TCMP1);

		 UARTDRV_Receive(handle, rxbuf, BUFF_SIZE, uart_rx_callback);// BUFF_SIZE
	 }
	 if(transferStatus == ECODE_EMDRV_UARTDRV_FRAME_ERROR)// ECODE_EMDRV_UARTDRV_ABORTED
	 {
		 //stop the timer
		 handle->peripheral.uart->TIMECMP1 &= ~_USART_TIMECMP1_TSTART_MASK;
		 handle->peripheral.uart->TIMECMP1 |= USART_TIMECMP1_TSTART_RXEOF;
		 USART_IntClear(handle->peripheral.uart, USART_IF_TCMP1);

		 UARTDRV_Receive(handle, rxbuf, BUFF_SIZE, uart_rx_callback);// BUFF_SIZE
	 }
 }

 static uint32_t app_usart_transmit(uint8_t* data, uint8_t len)
 {
 //  return UARTDRV_Transmit(handle, data, len, uart_tx_callback);
   return UARTDRV_TransmitB(handle, data, len);
 }

 bool app_handle_event(struct gecko_cmd_packet *evt)
 {
   bool evt_handled = false;
   switch (BGLIB_MSG_ID(evt->header)) {
   	  case gecko_evt_le_connection_opened_id:
           ble_conn_handle = evt->data.evt_le_connection_opened.connection;
           break;
 	  case gecko_evt_le_connection_closed_id:
 		  ble_conn_handle = DISCONNECTION;
 		  break;
 	  case gecko_evt_gatt_mtu_exchanged_id:
 		{
 		  uint8 connection = evt->data.evt_gatt_mtu_exchanged.connection;
 		  uint16 mtu = evt->data.evt_gatt_mtu_exchanged.mtu;
 		  ble_mtu = mtu - 3;
 		  MI_LOG_WARNING("connection: %d\n",connection);
 		  MI_LOG_WARNING("mtu: %d\n",mtu);
 		}break;
 	  case gecko_evt_gatt_server_attribute_value_id:
 	  case gecko_evt_gatt_server_user_write_request_id:
 		{
 		  int i = 0, len = evt->data.evt_gatt_server_attribute_value.value.len;
 		  if(evt->data.evt_gatt_server_attribute_value.attribute == gattdb_Recv)
 		  {
 			  DBG(YELLOW"gattdb_Recv len: %d\r\n ble recv: "NONE, len);
 //			  for(i = 0; i < len; i++)
 //				  DBG(YELLOW"%02X "NONE, evt->data.evt_gatt_server_attribute_value.value.data[i]);
 //			  DBG(YELLOW"\r\n "NONE);
 			  for(i = 0; i < len; i++)
 			  {
 				  uartSendBuf[ps_in] = evt->data.evt_gatt_server_attribute_value.value.data[i];
 				  ps_in++;
 				  if(ps_in >= BUFF_SIZE)
 				  {
 					  ps_in = 0;
 				  }
 			  }
 		  }
 		}break;
   	  case gecko_evt_hardware_soft_timer_id:
        {
        	  switch(evt->data.evt_hardware_soft_timer.handle)
        	  {
    			  case BLE_SEND :
    			  {
 //   				DBG(L_GREEN"BLE_SEND \r\n"NONE);
    				send_to_app_timeout_handler();
    			  }break;
    			  case UART_PRASE_CMD :
 			  {
 //				  DBG(L_GREEN"UART_PRASE_CMD \r\n"NONE);
 				  praseAppCmd();
 			  }break;
    			  case UART_SEND :
    			  {
 //   				DBG(L_GREEN"UART_SEND \r\n"NONE);
    				send_to_uart_timeout_handler();
    			  }break;
    			  case RESET:
    			  {
    				DBG(L_RED"RESET \r\n"NONE);
    				NVIC_SystemReset();
    			  }break;
    			  case UART_TEST:
    			  {
						DBG(L_GREEN"UART_TEST \r\n"NONE);
						STRU_BLE_DATA d;
						int i = 0;
						uint8_t buf[400] = { 0 };
						for (i = 0; i < 390; i++) {
							buf[i] = i;
						}
						d.state = 0;
						d.seq_id = 0;
						d.product_type = 0xF1;
						d.cmd_id = 0xDD;
						d.len = i;
						d.buff = buf;
						packDeviceCmd(&d);

						DBG(L_RED"before.uart->IF: %04X\r\n"NONE,handle->peripheral.uart->IF);
						DBG(L_RED"handle->rxDmaActive: %04X\r\n"NONE,handle->rxDmaActive);//handle->rxDmaActive
						DBG(L_RED"handle->peripheral.uart->CMD: %04X\r\n"NONE,handle->peripheral.uart->CMD);//handle->peripheral.uart->CMD
					}break;
					default: {
						// no this timer
					}
				}
			}break;
     default:
       break;
   }
   return evt_handled;
 }

 static unsigned int ptr_minus(unsigned int ptr, unsigned int off)
 {
 	if(ptr >= off)
 	{
 		return (ptr - off);
 	}
 	else
 	{
 		return (ptr + BUFF_SIZE - off);
 	}
 }

 static unsigned int ptr_plus(unsigned int ptr, unsigned int off)
 {
 	if((ptr + off)>= BUFF_SIZE)
 	{
 		return (ptr + off) - BUFF_SIZE;
 	}
 	else
 	{
 		return (ptr + off);
 	}
 }
 static unsigned int hex2str( unsigned char *phex, unsigned int len, char *pstr )
 {
   unsigned int       i;
   char        hex[] = "0123456789ABCDEF";

   for ( i = 0; i < len; i++ )
   {
     *pstr++ = hex[*phex >> 4];
     *pstr++ = hex[*phex++ & 0x0F];
   }

   *pstr = 0;

   return 2*len +1;
 }

 static void packDeviceCmd(STRU_BLE_DATA *data)
 {
 	unsigned int i = 0;
 	unsigned char checksum = 0x00;
 	unsigned int ps_in_tmp = 0;

 	ps_in_tmp = ps_in;
 	//打包头
 	uartSendBuf[ps_in] = WIFIAMP_APP_HEADER;
 	ps_in = ptr_plus(ps_in,1);
 	uartSendBuf[ps_in] = HI_UINT16(data->len + 4);
 	ps_in = ptr_plus(ps_in,1);
 	uartSendBuf[ps_in] = LO_UINT16(data->len + 4);
 	ps_in = ptr_plus(ps_in,1);
 	uartSendBuf[ps_in] = data->state;
 	ps_in = ptr_plus(ps_in,1);
 	//uartSendBuf[ps_in] = data->seq_id;
 	uartSendBuf[ps_in] = seq_id;
 	seq_id++;
 	if(seq_id >=255){seq_id = 0;}
 	ps_in = ptr_plus(ps_in,1);
 	uartSendBuf[ps_in] = data->product_type;
 	ps_in = ptr_plus(ps_in,1);
 	uartSendBuf[ps_in] = data->cmd_id;
 	ps_in = ptr_plus(ps_in,1);
 	//复制数据
 	for(i = 0; i < (data->len); i++)
 	{
 		uartSendBuf[ps_in] = data->buff[i];
 		ps_in = ptr_plus(ps_in,1);
 	}
 	//计算校验和
 	ps_in_tmp += 3;
 	for (i = 3; i < (data->len + 7); ++i)
 	{
 		checksum += uartSendBuf[ps_in_tmp];
 		ps_in_tmp = ptr_plus(ps_in_tmp,1);
 	}
 	//赋值校验和
 	uartSendBuf[ps_in] = checksum;
 	ps_in = ptr_plus(ps_in,1);

 }
 static void praseAppCmd(void)
 {
 	static unsigned char cmd_header_flag = 0x00;
 	static unsigned int cmd_header_loc = 0x00;
 	static unsigned int cmd_len = 0x00;

 	unsigned char cmd_checksum = 0x00;
 	unsigned int byte_cnt = 0x00;

 	unsigned int i = 0;
 	STRU_BLE_DATA d = {0};

 	if(p_out != p_in)
 		DBG(L_GREEN"START PRASE CMD\r\n"NONE);
 	while(p_out != p_in)
 	{
 		//DBG("p_out: %d, p_in: %d\n",p_out,p_in);
 		if(cmd_header_flag == 0x00)//寻找帧头
 		{
 			if(uartRecvBuf[ptr_minus(p_out,2)] == WIFIAMP_DEV_HEADER)
 			{
 				DBG(L_GREEN"FIND CMD HEAD\r\n"NONE);
 				byte_cnt = uartRecvBuf[ptr_minus(p_out,1)] << 8;
 				byte_cnt += uartRecvBuf[p_out];
 				if((byte_cnt >= 4) && (byte_cnt <= MAX_CMD_SIZE - 4))
 				{
 					cmd_len = byte_cnt;
           			cmd_header_flag = 0x01;
           			cmd_header_loc = ptr_minus(p_out,2);
 				}
 			}
 		}
 		else
 		{
 			if(ptr_minus(p_out,cmd_header_loc) == (cmd_len + 3))//帧长度达到
 			{
 				DBG(L_GREEN"CMD LEN OK\r\n"NONE);
 				cmd_checksum = 0x00;
 				for(i = 0; i < cmd_len; i++)//计算校验和
 				{
 					cmd_checksum +=  uartRecvBuf[ptr_minus(p_out,(i + 1))];
 				}
 				if(cmd_checksum == uartRecvBuf[p_out])
 				{
 					DBG(L_GREEN"CMD CRC OK\r\n"NONE);
 					switch(uartRecvBuf[ptr_plus(cmd_header_loc,5)])
 					{
 						case CMDTYPE_BLE_MODULE:
 						{
 							DBG(L_GREEN"CMDTYPE_BLE_MODULE\r\n"NONE);
 							//执行指令
 							switch(uartRecvBuf[ptr_plus(cmd_header_loc,6)])
 							{
 								case CMD_MODULE_HAND_SHAKE:
 								{
 									DBG(YELLOW"command: CMD_MODULE_HAND_SHAKE\r\n"NONE);
 									d.state = 0;
 									d.seq_id = 0;
 									d.product_type = CMDTYPE_BLE_MODULE;
 									d.cmd_id = CMD_MODULE_HAND_SHAKE;
 									d.len = 0;
 									d.buff = NULL;
 									packDeviceCmd(&d);
 								}break;
 								case CMD_SET_ADV_DATA:
 								{
 									DBG(YELLOW"command: CMD_SET_ADV_DATA\r\n"NONE);
 									uint8_t buf[2] = {0};
 									buf[0] = uartRecvBuf[ptr_plus(cmd_header_loc,7)];
 									d.state = 0;
 									d.seq_id = 0;
 									d.product_type = CMDTYPE_BLE_MODULE;
 									d.cmd_id = CMD_SET_ADV_DATA;
 									d.len = 1;
 									d.buff = buf;
 									packDeviceCmd(&d);
 								}break;
 								case CMD_ADV_CTRL:
 								{
 									DBG(YELLOW"command: CMD_ADV_CTRL\r\n"NONE);
 									uint8_t buf[2] = {0};
 									buf[0] = uartRecvBuf[ptr_plus(cmd_header_loc,7)];
 									d.state = 0;
 									d.seq_id = 0;
 									d.product_type = CMDTYPE_BLE_MODULE;
 									d.cmd_id = CMD_ADV_CTRL;
 									d.len = 1;
 									d.buff = buf;
 									packDeviceCmd(&d);
 									extern  void advertising_start(void);
 									 advertising_start();
 								}break;
 								default:
 									DBG(L_RED"No THIS CMD !!!!\r\n"NONE);
 							}
 						}break;
 						default:
 						{
 							DBG(L_GREEN"CMDTYPE_APP\r\n"NONE);
// 							if((ble_conn_state == 1)&&(mi_auth_state == LOGIN_SUCC))// mi auth success
 							if(ble_conn_state == 1)
 							{
 								for(i = 0; i < (cmd_len + 4); i++)
 								{
 									send_to_app_array[pb_in] = uartRecvBuf[ptr_plus(cmd_header_loc,i)];
 									pb_in = ptr_plus(pb_in,1);
 								}
 							}
 						}
 					}
 					cmd_header_flag = 0x00;
 				}
 				else
 				{
 					p_out = ptr_plus(cmd_header_loc,2);
 					cmd_header_flag = 0x00;
 					DBG(L_RED"CMD CRC ERROR!\r\n"NONE);
 				}
 			}
 		}
 		p_out = ptr_plus(p_out,1);
 	}
 }

 static void send_to_app_timeout_handler(void)
 {
   struct gecko_msg_gatt_server_send_characteristic_notification_rsp_t*  notify_rsp = NULL;
 	uint32_t len_remain = 0;
 	uint32_t p_out_tmp = 0;
 	uint8_t buf[255] = {0};
 	uint8_t tmp = 0;
// 	if((ble_conn_state == 0) || (mi_auth_state != LOGIN_SUCC))//free send buffer
// 	if(ble_conn_state == 1)
 	if(ble_conn_state == 0)
 	{
 		pb_out = pb_in;
 		app_send_timer_stop();
 	}
 	else
 	{
 		while(pb_out != pb_in)
 		{
 			len_remain = ptr_minus(pb_in, pb_out);
 			tmp = (len_remain > ble_mtu) ? ble_mtu : len_remain;
 			p_out_tmp = pb_out;
 			for(uint8_t i = 0; i < tmp; i++)
 			{
 				buf[i] = send_to_app_array[p_out_tmp];
 				p_out_tmp = ptr_plus(p_out_tmp,1);
 			}
 			notify_rsp =  gecko_cmd_gatt_server_send_characteristic_notification(ble_conn_handle,gattdb_Send,tmp,buf);
 			DBG(YELLOW"notify_rsp->result: %04X, notify_rsp->sent_len: %d\r\n"NONE,notify_rsp->result,notify_rsp->sent_len);
 //			if((err_code == NRF_SUCCESS)||(err_code == NRF_ERROR_INVALID_STATE))// not connected or not  enable notify or mi auth failed, free send buffer
 			if(notify_rsp->result == bg_err_success)
 			{
 				pb_out = p_out_tmp;
 			}
 			else
 			{
 				break;
 			}
 		}
 	}
 }

 void app_send_timer_start(void)
 {
 	struct gecko_msg_hardware_set_lazy_soft_timer_rsp_t* rsp = NULL;
 	rsp = gecko_cmd_hardware_set_lazy_soft_timer(TIMER_MS_2_TIMERTICK(20),TIMER_MS_2_TIMERTICK(5),BLE_SEND,0);
 	if(rsp->result != bg_err_success)
 	{
 		DBG(L_RED"app_send_timer_start(): ERROR  %04X\r\n"NONE,rsp->result);
 	}
 }

 void app_send_timer_stop(void)
 {
 	struct gecko_msg_hardware_set_lazy_soft_timer_rsp_t* rsp = NULL;
 	rsp = gecko_cmd_hardware_set_lazy_soft_timer(0,0,BLE_SEND,0);
 	if(rsp->result != bg_err_success)
 	{
 		DBG(L_RED"app_send_timer_stop(): ERROR  %04X\r\n"NONE,rsp->result);
 	}
 }

 static void send_to_uart_timeout_handler(void)
 {
 	if(ps_out != ps_in)
 	{
 #undef MAX_CMD_SIZE
 #define MAX_CMD_SIZE            400//140// after test, I found the max send is 142 byte.
 		unsigned int len_remain = 0;
 		unsigned int ps_out_tmp = 0;
 		unsigned char buf[MAX_CMD_SIZE] = {0};
 		unsigned int tmp = 0;
 		unsigned int i = 0;
 		char bufstr[2*MAX_CMD_SIZE + 1] = {0};
 		len_remain = ptr_minus(ps_in, ps_out);
 		tmp = (len_remain > MAX_CMD_SIZE) ? MAX_CMD_SIZE : len_remain;
 		ps_out_tmp = ps_out;
 		for(i = 0; i < tmp; i++)
 		{
 			buf[i] = uartSendBuf[ps_out_tmp];
 			ps_out_tmp = ptr_plus(ps_out_tmp,1);
 		}

 //	  for(i = 0; i < tmp; i++)
 //		  DBG(YELLOW"%02X "NONE, buf[i]);
 //	  DBG(YELLOW"\r\n "NONE);
 		 DBG(YELLOW"%d "NONE, tmp);
// 		if(app_usart_transmit(buf, tmp) == ECODE_EMDRV_UARTDRV_OK)
// 		if(UARTDRV_Transmit(handle, buf, tmp, callback) == ECODE_EMDRV_UARTDRV_OK)// UARTDRV_TransmitB
 		if(UARTDRV_TransmitB(handle, buf, tmp) == ECODE_EMDRV_UARTDRV_OK)// UARTDRV_TransmitB
 		{
 			 ps_out = ptr_plus(ps_out,tmp);
 //			 hex2str(buf,tmp,bufstr);
 //			 DBG(CYAN"%s\n"NONE,bufstr);
 		}
 		else
 		{
 			DBG(L_RED"uart send error !\r\n"NONE);
 		}
 	}
 }

 void ble_state_notify(uint8 state)
 {
 	STRU_BLE_DATA d = {0};
 	d.state = 0;
 	d.seq_id = 0;
 	d.product_type = CMDTYPE_BLE_MODULE;
 	d.cmd_id = CMD_CONN_STATE_NOTIFY;
 	d.buff = &state;
 	d.len = 1;
 	packDeviceCmd(&d);
 }










