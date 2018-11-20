/*
 * app_uart.h
 *
 *  Created on: Oct 18, 2018
 *      Author: root
 */

#ifndef DEMOS_SILABS_MIBLE_STD_AUTHEN_APP_UART_H_
#define DEMOS_SILABS_MIBLE_STD_AUTHEN_APP_UART_H_

#include "hal-config.h"
#include <stdint.h>
#include <stdbool.h>
#include "native_gecko.h"
#include "mible_server.h"

#if BSP_UARTNCP_USART_PORT == HAL_SERIAL_PORT_USART0
// USART0
#define APP_USART_UART        USART0
#define APP_USART_CLK         cmuClock_USART0
#define APP_USART_IRQ_NAME    USART0_RX_IRQHandler
#define APP_USART_IRQn        USART0_RX_IRQn
#define APP_USART_TX_IRQ_NAME   USART0_TX_IRQHandler
#define APP_USART_TX_IRQn       USART0_TX_IRQn
#define APP_USART_USART       1
#endif

/* 调试信息宏定义 */
#define _DEBUG

#define NONE                 "\x1B[0m"
#define BLACK                "\x1B[0;30m"
#define L_BLACK              "\x1B[1;30m"
#define RED                  "\x1B[0;31m"
#define L_RED                "\x1B[1;31m"
#define GREEN                "\x1B[0;32m"
#define L_GREEN              "\x1B[1;32m"
#define BROWN                "\x1B[0;33m"
#define YELLOW               "\x1B[1;33m"
#define BLUE                 "\x1B[0;34m"
#define L_BLUE               "\x1B[1;34m"
#define PURPLE               "\x1B[0;35m"
#define L_PURPLE             "\x1B[1;35m"
#define CYAN                 "\x1B[0;36m"
#define L_CYAN               "\x1B[1;36m"
#define GRAY                 "\x1B[0;37m"
#define WHITE                "\x1B[1;37m"

#define BOLD                 "\x1B[1m"
#define UNDERLINE            "\x1B[4m"
#define BLINK                "\x1B[5m"
#define REVERSE              "\x1B[7m"
#define HIDE                 "\x1B[8m"
#define CLEAR                "\x1B[2J"
#define CLRLINE              "\r\x1B[K" //or "\e[1K\r"

/* 输出调试信息 */
#ifdef _DEBUG
#include "third_party/SEGGER_RTT/SEGGER_RTT.h"
//#define DBG( format, args... )  printf( format, ##args)
#define DBG(format, args...)   SEGGER_RTT_printf(0, format, ##args)
#else
#define DBG( format, args... )
#endif

typedef enum
{
	BLE_SEND = 10,
	UART_PRASE_CMD,
	UART_SEND,
	RESET,
	UART_TEST,
}timerId;

//BLE命令数据
typedef struct stru_ble_data
{
	unsigned char state;		//状态ID，固定为0x00
	unsigned char seq_id;		//顺序ID
	unsigned char product_type;	//产品类型
	unsigned char cmd_id;		//命令ID
	unsigned char* buff;		//数据缓冲区
	unsigned int len;		        //数据长度
}STRU_BLE_DATA;

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)

//frame head
#define WIFIAMP_DEV_HEADER	                0xD0
#define WIFIAMP_APP_HEADER	                0xC0
// cmd
// 蓝牙模块
#define CMDTYPE_BLE_MODULE             0xF2
#define CMD_CONN_STATE_NOTIFY               0x10
#define CMD_ADV_CTRL                        0x11
#define CMD_SET_ADV_DATA                    0x12
#define CMD_MODULE_HAND_SHAKE               0x13

#define MAX_CMD_SIZE     400
#define BUFF_SIZE   1024
extern unsigned int volatile p_in, p_out;//接收循环buffer下标
extern unsigned char uartRecvBuf[BUFF_SIZE];
extern unsigned int volatile ps_in, ps_out;//发送循环buffer下标
extern unsigned char uartSendBuf[BUFF_SIZE];
extern unsigned char seq_id;

 void app_uart_init(void);
 bool app_handle_event(struct gecko_cmd_packet *evt);

 extern uint8_t ble_conn_state;
 extern mible_bonding_state mi_auth_state;
 extern  void app_send_timer_start(void);
 extern void app_send_timer_stop(void);
 extern void ble_state_notify(uint8 state);




#endif /* DEMOS_SILABS_MIBLE_STD_AUTHEN_APP_UART_H_ */
