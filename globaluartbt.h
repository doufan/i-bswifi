
#ifndef __UARTBT_H
#define __UARTBT_H

#ifdef __cplusplus
 extern "C" {
#endif

#define FALSE  -1  
#define TRUE   0


#define BAUDRATE        115200
#define UART_DEVICE     "/dev/ttyS1"

#define CLOUD_NAME "wifiServer"
#define CLOUD_DEFAULT_SERVER "wifibox.wanwudayin.com"
#define CLOUD_DEFAULT_PORT 60000
#define CLOUD_DEFAULT_PASSWORD "12345678"
#define DEVICE_DEFAULT_SN   "mybox000001"
#define SERIAL_DEFAULT_BAUDRATE "115200"
#define SERIAL_DEFAULT_PARITYBIT "N"
#define SERIAL_DEFAULT_DATABIT "8"
#define SERIAL_DEFAULT_STOPBIT "1"





#define DEVICE_ACK               "OK"
#define DEVICE_NACK              "NO"
#define SERVER_ACK               "OK"
#define SERVER_NACK              "NO"


#define NVRAM_AP_SN 			"myApSn"
#define NVRAM_CLOUD_ADDR  		"sniServerAddr"
#define NVRAM_CLOUD_PORT  		"sniServerPort"
#define NVRAM_CLOUD_PASSWORD  	"sniServerPwd"
#define NVRAM_SERIAL_BAUDRATE	"serialBaudrate"
#define NVRAM_SERIAL_PARITYBIT	"serialPartybit"
#define NVRAM_SERIAL_STOPBIT	"serialStopbit"
#define NVRAM_SERIAL_DATABIT	"serialDatabit"


#define SEND_PACKET_LENTH     1024
#define RECV_PACKET_LENTH  	  1024

#define HI_UINT16(a) (((a) >> 8) & 0xFF)
#define LO_UINT16(a) ((a) & 0xFF)
  
//WIFI
#define WIFIAMP_DEV_HEADER	                0xD0 
#define WIFIAMP_APP_HEADER	                0xC0

#define BLE_REC_BUFF_LEN	                	800 //环形缓冲区长度，至少为BLE_CMD_MAX_LEN的2倍
#define BLE_CMD_MAX_LEN							400 //单条BLE命令最大长度

#define CMD_PARSED_FINISHED 	                0x00
#define CMD_PARSED_UNFINISHED 	                0x01
#define CMD_PARSED_ERR_LENGTH 	                0x02
#define CMD_PARSED_ERR_CHECKSUM	                0x03
#define CMD_PARSED_ERR_OTHER	                0x04

// cmd
// 蓝牙模块
#define CMDTYPE_BLE_MODULE             0xF2
#define CMD_CONN_STATE_NOTIFY               0x10
#define CMD_ADV_CTRL                        0x11
#define CMD_SET_ADV_DATA                    0x12
#define CMD_MODULE_HAND_SHAKE               0x13

// 设置wifi
#define CMDTYPE_SET_WIFI               0xF1
#define CMD_HAND_SHAKE                      0xE0
#define CMD_REQ_ROUTE_LIST                  0xE1
#define CMD_SEND_ROUTE_LIST                 0xE2
#define CMD_SET_WIFI                        0xE3
#define CMD_SEND_WIFI_CONN_STATE            0xE4
#define CMD_CANCEL_SET_WIFI                 0xE5
#define CMD_TEST                            0xE6

// 读写放大器配置参数指令
#define CMDTYPE_WIFI_CONFIG_PARAM      0xFF
#define CMD_LOGIN                           0xF0
#define CMD_LOGOUT                          0xF1
#define CMD_IN_OUT_SET_WIFI_STATE           0xF2
#define CMD_READ_CONFIGER_PARAM             0xF3
#define CMD_WRITE_CONFIGER_PARAM            0xF4
#define CMD_START_SPEED_TEST                0xF5
#define CMD_SEND_SPEED_TEST_STATE           0xF6

// led
#define LED2      1
#define LED3      2
#define LED4      3


typedef enum
{
	F_2_4G = 0,
	F_5G = 1,
	F_2_4G_AND_5G = 2
}FREQ;

typedef enum
{
	SWL_FIRST_GET_ROUTE_LIST = 0,
	SWL_SECOND_GET_ROUTE_LIST = 1,
	SWL_GET_ROUTE_LIST_FROM_CATCH,
	SWL_GET_ROUTE_LIST_FROM_CATCH_2,
	SWL_SEND_ROUTE_LIST,
	SWL_GET_WIFI_CONN_STATE,
	SWL_SEND_WIFI_CONN_STATE,
	SWL_SPEED_TEST_LAN_DUAL,
	SWL_SPEED_TEST_LAN_SINGLE,
	SWL_SPEED_TEST_WAN,
	SWL_NOP_WAIT
}SET_WIFI_LOGIC_TE;

typedef enum
{
	TO_APP_CONN_SUCCESS = 0,
	TO_APP_CONN_PASSWORD_ERROR = 1,
	TO_APP_CONN_DHCP_ERROR,
	TO_APP_CONN_NOT_FIND_ROUTE,
	TO_APP_CONN_CONN_TIMEOUT,
	TO_APP_CONN_CONNECTING,
	TO_APP_CONN_OTHER_ERROR,
	TO_APP_CONN_NOT_SET
}TO_APP_CONN_STATUS_TE;

typedef enum
{
	WIFI_CONN_CONNECTING = 0,
	WIFI_CONN_SUCCESS,
	WIFI_CONN_DHCP_ERROR = -1,
	WIFI_CONN_NOT_FIND_ROUTE = -3,
	WIFI_CONN_PASSWORD_ERROR = -2,
	WIFI_CONN_FAILED = -4,
	WIFI_CONN_OTHER_ERROR = -5
}WIFI_CONN_STATUS_TE;

typedef enum
{
	WIFI_EXT_NOT_START = 0,
	WIFI_EXT_EXTENDING = 1,
	WIFI_EXT_SUCCESS,
	WIFI_EXT_ERROR
}WIFI_EXT_STATUS_TE;

typedef enum
{
	MK_HAND_SHAKE = 0,
	MK_HAND_SHAKE_TO,
	MK_SET_ADV_DATA,
	MK_SET_ADV_DATA_TO,
	MK_START_ADV,
	MK_START_ADV_TO,
	MK_NOP
}MK_CTRL_STEP_TE;

typedef enum
{
	ID_DEV_INFO = 0,
	ID_UPPER_NETWORK_INFO,
	ID_EXTEND_NETWORK_INFO,
	ID_DEV_LIST_CONN_NOW,
	ID_DEV_LIST_CONN_CURRENT,
	ID_SET_MAC_FILTER,
	ID_GET_MAC_FILTER_WRITE_LIST,
	ID_GET_MAC_FILTER_BLACK_LIST,
	ID_CTRL_LED,
	ID_EXTENDER_NET_CFG,
	ID_RW_DHCP_SERVICE,
	ID_EXTENDER_USER_MANAGE,
	ID_GET_SYSTEM_LOG
}CFG_PARAM_ID_TE;

//BLE命令数据
typedef struct stru_ble_data
{
	unsigned char is_resp;      //是否是回复指令
	unsigned char need_resend;  //是否启动重发
	unsigned char state;		//状态ID，固定为0x00
	unsigned char seq_id;		//顺序ID
	unsigned char product_type;	//产品类型
	unsigned char cmd_id;		//命令ID
	unsigned char* buff;		//数据缓冲区
	unsigned int len;		        //数据长度
}STRU_BLE_DATA;

//reSend cmd struct
typedef struct stru_resend_cmd
{
	unsigned char has_cmd;// have or not one cmd in this resend buffer
	unsigned char is_cmd_changed;
	unsigned char resend_cnt;// retransmission counts
	unsigned char resend_to;// timeout counter of each resend
	STRU_BLE_DATA cmd;
	unsigned char cmd_data[BLE_CMD_MAX_LEN];// buffer for cmd.buff
	
}RESEND_CMD_T;

// 路由器列表
typedef struct route
{
	char ssid[64];
	char ch[4];
	char security[32];
	char rssi[8];
	char bssid[32];
	unsigned char ssid_len;
	unsigned char ch_len;
	unsigned char security_len;
	unsigned char rssi_len;
	unsigned char bssid_len;
	unsigned int route_len;
}ROUTE_T;


#define MAX_APLIST_NUM 255

typedef struct mkos_aplist_entry
{
	char ssid[64];
	char ch[4];
	char security[32];
	char rssi[8];
	char bssid[32];
	char mode[16];
	char extch[8];
	unsigned char is_online;
	unsigned char is_new;
	unsigned char is_5g;
}T_MKOS_APLIST_ENTRY;

typedef struct mkos_aplist_fctl
{
	
	unsigned char is_enable;
	unsigned char rules;
	T_MKOS_APLIST_ENTRY all_aplist_entry_tbl[MAX_APLIST_NUM];
}T_MKOS_APLIST_FCTL,*T_MKOS_APLIST_FCTL_P ;


typedef struct mkos_apcli_para
{
	char enable[4];
	char ssid[64];
	char ch[4];
	char security[32];
	char key[64];
	unsigned int timeout;
}T_MKOS_APCLI_PARA;

typedef struct mkos_apcli_status_info
{
	char ssid[64];
	char ch[4];
	char ip[32];
	char nm[32];
	int is_enable;
	int status;
	int rssi;
	unsigned long rate;
}T_MKOS_APCLI_STATUS_INFO;


typedef struct mkos_apcli_wan_speedtest_info
{
	int sends;
	int losts;
	double maxRtt;
	double minRtt;
	double averRtt;
	int rssi;
}T_MKOS_APCLI_WAN_SPEEDTEST_INFO;

typedef struct mkos_ext_ap_info
{
	char ssid[64];
	char ch[4];
	char security[32];
	char pass[64];
	char enable[4];
	char hidden[4];
}T_MKOS_EXT_AP_INFO;

typedef struct mkos_ext_ap_info_set_from_app
{
	unsigned char id;
	unsigned char net_status;
	unsigned char ssid_broadcast_status;
	unsigned char ch;
	char security[32];
	char ssid[64];
	char pass[64];
}T_MKOS_EXT_AP_INFO_SET_FROM_APP;

typedef struct mkos_dev_list_now
{
	unsigned char ip[4];
	unsigned char mac[6];
	int rssi;
	int speed_tx;
	char name[64];
	int name_len;
	char remark_name[64];
	int remark_name_len;
	char band;
	char bandwidth[8];
	int time_online;
	int dev_len;
}T_MKOS_DEV_LIST_NOW;

typedef struct mkos_dev_list_now_from_app
{
	unsigned char id;
	unsigned char more;
	unsigned char ip[4];
	unsigned char mac[6];
	int rssi;
	int speed_tx;
	char name[64];
	int name_len;
	char remark_name[64];
	int remark_name_len;
}T_MKOS_DEV_LIST_NOW_FROM_APP;

typedef struct mkos_write_list
{
	unsigned char mac[6];
}T_MKOS_WRITE_LIST;

typedef struct mkos_black_list
{
	unsigned char mac[6];
}T_MKOS_BLACK_LIST;

typedef struct mkos_apcli_lan_speedtest_info
{
	int sends;
	int losts;
	double maxRtt;
	double minRtt;
	double averRtt;
	double allRtt;
	int rssi;
}T_MKOS_APCLI_LAN_SPEEDTEST_INFO;

/* 调试信息宏定义 */
#define _DEBUG
//#define _REPORT   
//#define _SYSLOG
//#define _RECODEG   

//#define _SDKARD

#ifdef _SDKARD
#define FILE_DIR    "/etc_ro/web/ss/"
#else
#define FILE_DIR    "/tmp/"
#endif

#define NONE                 "\e[0m"
#define BLACK                "\e[0;30m"
#define L_BLACK              "\e[1;30m"
#define RED                  "\e[0;31m"
#define L_RED                "\e[1;31m"
#define GREEN                "\e[0;32m"
#define L_GREEN              "\e[1;32m"
#define BROWN                "\e[0;33m"
#define YELLOW               "\e[1;33m"
#define BLUE                 "\e[0;34m"
#define L_BLUE               "\e[1;34m"
#define PURPLE               "\e[0;35m"
#define L_PURPLE             "\e[1;35m"
#define CYAN                 "\e[0;36m"
#define L_CYAN               "\e[1;36m"
#define GRAY                 "\e[0;37m"
#define WHITE                "\e[1;37m"

#define BOLD                 "\e[1m"
#define UNDERLINE            "\e[4m"
#define BLINK                "\e[5m"
#define REVERSE              "\e[7m"
#define HIDE                 "\e[8m"
#define CLEAR                "\e[2J"
#define CLRLINE              "\r\e[K" //or "\e[1K\r"


/* 输出调试信息 */
#ifdef _DEBUG
#define DBG( format, args... )  printf( format, ##args)
#else
#define DBG( format, args... )
#endif

//打印到终端
#ifdef _REPORT
#define REPORT( format, args... )  fprintf( format,##args)
#else
#define REPORT( format, args... )
#endif

//打印到终端
#ifdef _SYSLOG
#define SYSLOG( format, args... )  syslog("[NETWORK]" format "Function:%s",##args,__FILE__)
#else
#define SYSLOG( format, args... )
#endif


//记录打印GCODE文件
#ifdef _RECODEG
#define RECODEG( format, args... )  fprintf(recodegFp, format,##args)
#else
#define RECODEG( format, args... )
#endif


typedef enum{
	NO_INSERT=0,
	INSERT,
	READY,
	IDLE,
	BUSY,
	ERRO
}TTYUSB0_STATUS;

typedef enum
{
	UARTBT_IDLE=0,
	UARTBT_PRINTING,
	UARTBT_DOWNLINE,//
	UARTBT_BUSY,
	UARTBT_PAUSING,
	UARTBT_NOINSERT,
	UARTBT_ERROR	
}UARTBT_STATE;


#ifdef __cplusplus
}
#endif


#endif /* __UARTBT_H */

