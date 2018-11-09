
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
//#include <string.h>

#include <sys/time.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>


#include <sys/types.h>  
#include <sys/stat.h>  
#include <fcntl.h>  
#include <termios.h>  


#include <errno.h> 

#include <pthread.h>
#include "globaluartbt.h"
#include "rosPlatformCommonLib.h"





#define BUFFER_SIZE		64*1024
#define WAIT_MAX_TIMES	6

static unsigned char idps[] = 
{
	'c','o','m','.','j','i','u','a','n','.','W','E','1','0',0x00,0x00,//protocol string
	'W','i','F','i',' ','E','x','t','e','n','d','e','r',0x00,0x00,0x00,//accessory name
	'0','0','1',// firmware version
	'1','0','0',// hardware version
	'W','y','z','e',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//manufacture name
	'W','E','3',0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,//model number
	'1','2','3','4','5','6','7','8','9','0','a','b','\0','\0','\0',0x00//serial number
}; 

static char gCloudServerAddr[128]={0};
static unsigned short int gCloudServerPort=0;
static char gCloudServerPwd[64]={0};
static struct hostent *gCloudHostinfo;
struct sockaddr_in gServerIp;
static int gServerIsIp=0;
static char gMyboxSn[32]={0};

static pthread_t thread[36];
pthread_t localServerThread;
pthread_t uartbtMontorThread;
pthread_t routeListCacheThread;


static pthread_mutex_t mut;
static pthread_cond_t cond;
static int pthread_flag = 1;

UARTBT_STATE uartbt_state = UARTBT_NOINSERT; 



static int uart_fd=-1;

#define BUFF_SIZE                       1024
#define MAX_CMD_SIZE                    400
#define MAX_ROUTE_LIST_NUM              100

static unsigned int p_in = 0, p_out = 0;//接收循环buffer下标
static unsigned char uartRecvBuf[BUFF_SIZE] = {0};
static unsigned int ps_in = 0, ps_out = 0;//发送循环buffer下标
static unsigned char uartSendBuf[BUFF_SIZE] = {0};

static SET_WIFI_LOGIC_TE swl_get_list_ctl = SWL_NOP_WAIT;
static SET_WIFI_LOGIC_TE swl_send_list_ctl = SWL_NOP_WAIT;
static ROUTE_T route_list[MAX_ROUTE_LIST_NUM] = {0};
static unsigned int route_cnt = 0, has_send_route_cnt = 0;
static int get_list_flag = FALSE;
static int g_isSpeedTesting = FALSE;
static int g_duringCfg = 0;// during config and extend wifi
static int g_duringSetWifiLogic = 0;//during config, extend, speed test, select net, save config.
static int g_is_upload_set_wifi_state = 0;// is or not upload set wifi state
static unsigned char conn_timeout = 0;
static unsigned int conn_to = 0;
static unsigned int timer_cnt = 0;
static T_MKOS_APCLI_PARA wifi[2] = {0};
static int g24ext_status = WIFI_EXT_NOT_START;
static int g58ext_status = WIFI_EXT_NOT_START;
static int g24extended = FALSE;
static int g58extended = FALSE;
static unsigned char set_wifi_type = 0;
static MK_CTRL_STEP_TE mk_ctrl = MK_HAND_SHAKE;
static ROUTE_T cache_route_list2[2][MAX_ROUTE_LIST_NUM] = {0};// route list
static int cache_route_list_tmp_cnt2[2] = {0};
static int cache_route_list_cnt2[2] = {0};// route count
static int cache_route_list_status2[2] = {0};// 0: NULL, 1: HALF, 2: FULL
static int cache_copy_from = 0;// 0:default from cache_route_list2[0], 1: from cache_route_list2[1]
static RESEND_CMD_T g_resend_cmd = {0};


static time_t t_start_conn = 0, t_stop_conn = 0, t_stop_24g_ext = 0, t_stop_58g_ext = 0, t_start_speed_test = 0, t_stop_speed_test = 0,t_stop_select = 0, t_stop_save = 0;

//for wifi global
#define RT2860_NVRAM 0
#define RTDEV_NVRAM 1
T_MKOS_APLIST_FCTL_P pMkosAplist_data;
T_MKOS_APLIST_FCTL mkosAplist_data;
int currentG24ApcliStatus=1;
int currentG58ApcliStatus=1;


unsigned int aplist_tbl_index;
static unsigned char seq_id = 0;


static unsigned int ptr_plus(unsigned int ptr, unsigned int off);
static unsigned int ptr_minus(unsigned int ptr, unsigned int off);
static void send2UartBuf(STRU_BLE_DATA *data);



int hexCharToValue(const char ch)
{
	int result = TRUE;
	
	if(ch >= '0' && ch <= '9')
	{
		result = (int)(ch - '0');
	}
	else if(ch >= 'a' && ch <= 'z')
	{
		result = (int)(ch - 'a') + 10;
	}
	else if(ch >= 'A' && ch <= 'Z')
	{
		result = (int)(ch - 'A') + 10;
	}
	else
	{
		DBG("\nthe hex char error");
		result = FALSE;
	}
	return result;
}


static int get_mib(char *val, char *mib)
{
	FILE *mibFp;
 	char mibBuf[2048];

	sprintf(mibBuf, "nvram_get 2860 %s", mib);
    mibFp = popen(mibBuf, "r");
	if (mibFp==NULL)
	{
		return FALSE;
	}
	if ((NULL == fgets(mibBuf, sizeof(mibBuf),mibFp)) ||  (mibBuf[0] == '\0') ||  (mibBuf[0] == 10)) 
	{
		pclose(mibFp);
		return FALSE;
	}

	//strcpy(val, strstr(buf, "\"")+1);
	strcpy(val, mibBuf);
	val[strlen(val)-1] = '\0';
	pclose(mibFp);
	return TRUE;
}


static void commuPraseApList(char *buf, T_MKOS_APLIST_ENTRY *ap, unsigned int *cnt, unsigned int mode)
{
	unsigned int k = 0, m = 0, route_cnt = *cnt;

	int i,j;
	char localCh[4]={0};
	char localSsid[64]={0};
	char localBssid[32]={0};
	char localSecurity[32]={0};
	char localRssi[8]={0};
	char localMode[16]={0};
	char localExtch[8]={0};
	int firstApNum=0;
	int needAdd = 1;

	firstApNum = route_cnt;
	while(buf[k] != '\0')
	{
		memset(localCh,0,sizeof(localCh));
		memset(localSsid,0,sizeof(localSsid));
		memset(localBssid,0,sizeof(localBssid));
		memset(localSecurity,0,sizeof(localSecurity));
		memset(localRssi,0,sizeof(localRssi));
		memset(localMode,0,sizeof(localMode));
		memset(localExtch,0,sizeof(localExtch));
		needAdd = 1;
		while(buf[k] != '\n')
		{
			localCh[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localSsid[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localBssid[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localSecurity[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localRssi[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localMode[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localExtch[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
			
		if(mode)
		{
			needAdd = 0;
			for(i=0;i<firstApNum;i++)
			{
				if(!strcmp(pMkosAplist_data->all_aplist_entry_tbl[i].ssid,localSsid))
				{
					break;
				}
			}
			if(i >= firstApNum)
			{
				needAdd = 1;
			}
		}
		//DBG("\r\nssid=%s,ch=%s,bssid=%s,sec=%s,rssi=%s,mode=%s,ext=%s\r\n",localSsid,localCh,localBssid,localSecurity,localRssi,localMode,localExtch);
		if(needAdd)
		{

			if(localSsid[0] != '\0')
			{
				strcpy(ap[route_cnt].ch,localCh);
				strcpy(ap[route_cnt].ssid,localSsid);
				strcpy(ap[route_cnt].bssid,localBssid);
				strcpy(ap[route_cnt].security,localSecurity);
				strcpy(ap[route_cnt].rssi,localRssi);
				strcpy(ap[route_cnt].mode,localMode);
				strcpy(ap[route_cnt].extch,localExtch);
				if(mode)
				{
					ap[route_cnt].is_new = 1;
				}
				else
				{
					ap[route_cnt].is_new = 0;
				}
				if(atoi(ap[route_cnt].ch) > 14)
				{
					ap[route_cnt].is_5g = 1;
				}
				route_cnt++;
				if(route_cnt >= 128)
				{
					route_cnt = 0;
				}
			}
		}
	}
	*cnt = route_cnt;
}


static void reinit_mkos_aplist_info(void)
{
	unsigned int  i;
	T_MKOS_APLIST_ENTRY *pData;
	aplist_tbl_index=0;
	for(i=0;i<MAX_APLIST_NUM;i++)
	{
		pData=&(pMkosAplist_data->all_aplist_entry_tbl[i]);
		memset(pData->ssid,0,64);
		memset(pData->ch,0,4);
		memset(pData->security,0,32);
		memset(pData->rssi,0,8);
		memset(pData->bssid,0,32);
		memset(pData->mode,0,16);
		memset(pData->extch,0,8);
		pData->is_online = 0;
		pData->is_new = 0;
		pData->is_5g = 0;
	}
	return;
	
}


int mkos_aplist_init(void)
{
	pMkosAplist_data = &mkosAplist_data;
	memset(pMkosAplist_data,0,sizeof(T_MKOS_APLIST_FCTL));
	pMkosAplist_data->is_enable = 1;
	pMkosAplist_data->rules = 0;
	reinit_mkos_aplist_info();
	
}


// set the time data and TZ to master, 0 is ok, 
static int commuSetTimeAndPhoneTZ(char *timeStr, char *tzStr)
{
	return 0;
}

//get the wifi band ,0 2g, 1 5g, 2 dual
static int commuGetWifiBand(void)
{
	return getWifiBandPlatformlib();
}

// start scan ap , 0 is start ok
static int commuStartScanAp(void)
{
	int result = -1;
	
	result = startScanApPlatformlib();
	mkos_aplist_init();
	return result;
}

// get the wifi ap site list, after 3s
static int commuGetApSiteList(char *contentBuff)
{
	int result = -1;
	int i=0;
	
	sleep(3);
	getApSiteListPlatformlib(contentBuff);
	commuPraseApList(contentBuff,pMkosAplist_data->all_aplist_entry_tbl,&aplist_tbl_index,0);
	if(aplist_tbl_index == 0)
	{
		result = -2;
	}
	else
	{
		memset(contentBuff,0,sizeof(contentBuff));
		for(i=0;i<aplist_tbl_index;i++)
		{
			sprintf(contentBuff,"%s%s\n%s\n%s\n%s\n%s\n",contentBuff,pMkosAplist_data->all_aplist_entry_tbl[i].ssid,pMkosAplist_data->all_aplist_entry_tbl[i].ch,pMkosAplist_data->all_aplist_entry_tbl[i].security,pMkosAplist_data->all_aplist_entry_tbl[i].rssi,pMkosAplist_data->all_aplist_entry_tbl[i].bssid);
			result = 0;
		}
		startScanApPlatformlib();
	}
	//strcpy(contentBuff,"ssid1\n1\nWPA2/AES\n-56\nAABBCCDDEEFF\nssid2\n157\nWPAWPA2/AES\n-60\n1122334455\n\0");
	//strcpy(contentBuff,"111111111111111111111111111111111111111111111111111111111111111\n1\nWPA2/AES\n-56\nAABBCCDDEEFF\n\
//222222222222222222222222222222222222222222222222222222222222222\n11\nWPA2/AES\n-60\n1122334455\n\
//333333333333333333333333333333333333333333333333333333333333333\n157\nWPA2/AES\n-61\n1122334456\n\
//444444444444444444444444444444444444444444444444444444444444444\n157\nWPA2/AES\n-62\n1122334457\n\
//555555555555555555555555555555555555555555555555555555555555555\n157\nWPA2/AES\n-63\n1122334458\n\
//666666666666666666666666666666666666666666666666666666666666666\n157\nWPA2/AES\n-64\n1122334459\n\
//777777777777777777777777777777777777777777777777777777777777777\n157\nWPA2/AES\n-65\n1122334450\n\
//888888888888888888888888888888888888888888888888888888888888888\n157\nWPA2/AES\n-66\n112233445A\n\
//999999999999999999999999999999999999999999999999999999999999999\n157\nWPA2/AES\n-67\n112233445B\n\
//000000000000000000000000000000000000000000000000000000000000000\n157\nWPA2/AES\n-68\n112233445C\n\0");
	//strcpy(contentBuff,"11111111-2.4G\n1\nWPA2/AES\n-56\nAABBCCDDEEFF\n\
//2222222-2.4G\n11\nWPA2/AES\n-60\n1122334455\n\
//33中文3-5G\n157\nWPA2/AES\n-61\n1122334456\n\
//44Français-5G\n157\nWPA2/AES\n-62\n1122334457\n\
//55⊙﹏⊙b�?5G\n157\nWPA2/AES\n-63\n1122334458\n\
//66★~�?5G\n157\nWPA2/AES\n-64\n1122334459\n\
//77🤗-5G\n157\nWPA2/AES\n-65\n1122334450\n\
//88888-5G\n157\nWPA2/AES\n-66\n112233445A\n\
//99999-5G\n157\nWPA2/AES\n-67\n112233445B\n\
//00000-5G\n157\nWPA2/AES\n-68\n112233445C\n\0");

	return result;
}

// second get the wifi ap site list, maybe you call the func affer commuGetApSiteList() 8s
static int commuGetApSiteListSecond(char *contentBuff)
{
	int result = -1;
	int i=0;

	getApSiteListPlatformlib(contentBuff);
	commuPraseApList(contentBuff,pMkosAplist_data->all_aplist_entry_tbl,&aplist_tbl_index,1);
	if(aplist_tbl_index == 0)
	{
		result = -2;
	}
	else
	{
		memset(contentBuff,0,sizeof(contentBuff));
		for(i=0;i<aplist_tbl_index;i++)
		{
			if(pMkosAplist_data->all_aplist_entry_tbl[i].is_new)
			{
				sprintf(contentBuff,"%s%s\n%s\n%s\n%s\n%s\n",contentBuff,pMkosAplist_data->all_aplist_entry_tbl[i].ssid,pMkosAplist_data->all_aplist_entry_tbl[i].ch,pMkosAplist_data->all_aplist_entry_tbl[i].security,pMkosAplist_data->all_aplist_entry_tbl[i].rssi,pMkosAplist_data->all_aplist_entry_tbl[i].bssid);
				result = 0;
			}
		}
	}
	return result;
}

static void praseApList(char *buf,T_MKOS_APLIST_ENTRY *ap, unsigned int *cnt)
{
	unsigned int k = 0, m = 0, route_cnt = *cnt;

	int i,j;
	char localCh[4]={0};
	char localSsid[64]={0};
	char localBssid[32]={0};
	char localSecurity[32]={0};
	char localRssi[8]={0};
	char localMode[16]={0};
	char localExtch[8]={0};
	int needAdd = 0;

	while(buf[k] != '\0')
	{
		memset(localCh,0,sizeof(localCh));
		memset(localSsid,0,sizeof(localSsid));
		memset(localBssid,0,sizeof(localBssid));
		memset(localSecurity,0,sizeof(localSecurity));
		memset(localRssi,0,sizeof(localRssi));
		memset(localMode,0,sizeof(localMode));
		memset(localExtch,0,sizeof(localExtch));
	
		while(buf[k] != '\n')
		{
			localCh[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localSsid[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localBssid[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localSecurity[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localRssi[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localMode[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		while(buf[k] != '\n')
		{
			localExtch[m] = buf[k];
			k++;
			m++;
		}
		k++;
		m = 0;
		
		needAdd = 0;
		for(i=0;i<route_cnt;i++)
		{
			/*if(!strcmp(pMkosAplist_data->all_aplist_entry_tbl[i].ssid,localSsid))
			{
				break;
			}*/
		
			if(!strcmp(pMkosAplist_data->all_aplist_entry_tbl[i].bssid,localBssid))
			{
				break;
			}
		}
		if(i >= route_cnt)
		{
			needAdd = 1;
		}
		
		//DBG("\r\nssid=%s,ch=%s,bssid=%s,sec=%s,rssi=%s,mode=%s,ext=%s\r\n",localSsid,localCh,localBssid,localSecurity,localRssi,localMode,localExtch);
		if(needAdd)
		{

			if(localSsid[0] != '\0')
			{
				strcpy(ap[route_cnt].ch,localCh);
				strcpy(ap[route_cnt].ssid,localSsid);
				strcpy(ap[route_cnt].bssid,localBssid);
				strcpy(ap[route_cnt].security,localSecurity);
				strcpy(ap[route_cnt].rssi,localRssi);
				strcpy(ap[route_cnt].mode,localMode);
				strcpy(ap[route_cnt].extch,localExtch);
				
				
				route_cnt++;
				if(route_cnt >= 128)
				{
					route_cnt = 0;
				}
			}
		}
	}
	*cnt = route_cnt;
}
// start scan ap , 0 is start ok
static int startScan(void)
{
	int result = -1;
	
	result = startScanApPlatformlib();
	return result;
}

static int getApList(char *buf)
{
	int result = -1;
	int i=0;
	unsigned int cnt_temp = 0;
	cnt_temp = aplist_tbl_index;
	
	getApSiteListPlatformlib(buf);
	praseApList(buf,pMkosAplist_data->all_aplist_entry_tbl,&aplist_tbl_index);
	memset(buf,0,sizeof(buf));
	DBG(YELLOW"getApList: cnt_temp = %d, aplist_tbl_index = %d\n"NONE,cnt_temp,aplist_tbl_index);
	for(i=cnt_temp;i<aplist_tbl_index;i++)
	{
		sprintf(buf,"%s%s\n%s\n%s\n%s\n%s\n",buf,pMkosAplist_data->all_aplist_entry_tbl[i].ssid,pMkosAplist_data->all_aplist_entry_tbl[i].ch,pMkosAplist_data->all_aplist_entry_tbl[i].security,pMkosAplist_data->all_aplist_entry_tbl[i].rssi,pMkosAplist_data->all_aplist_entry_tbl[i].bssid);
		result = 0;
	}
}

static void stopScan(void)
{
	char buf[8192] = {0};
	getApSiteListPlatformlib(buf);
}

//connect the ap, 0 means save ok,not connetct ok

static int commuConnectAp(T_MKOS_APCLI_PARA *g24,T_MKOS_APCLI_PARA *g58)
//static int commuConnectAp(char *ssid,char *channel,char *security, char *password, unsigned char to)
{
	int result = -1;
	char authmode[16]={0};
	char encrypt[16]={0};
		

	if(g24 != NULL)
	{
		currentG24ApcliStatus = 1;
		if(strstr(g24->security,"NONE"))
		{
			strcpy(authmode,"OPEN");
			strcpy(encrypt,"NONE");
		}
		else if(strstr(g24->security,"WEP"))
		{
			strcpy(authmode,"OPEN");
			strcpy(encrypt,"WEP");
		}
		else if(strstr(g24->security,"WPA2PSK"))
		{
			strcpy(authmode,"WPA2PSK");
			if(strstr(g24->security,"AES"))
			{
				strcpy(encrypt,"AES");
			}
			else if(strstr(g24->security,"TKIP"))
			{
				strcpy(encrypt,"TKIP");
			}
			else
			{
				strcpy(encrypt,"AES");
			}
			
		}
		else
		{
			strcpy(authmode,"WPAPSK");
			if(strstr(g24->security,"AES"))
			{
				strcpy(encrypt,"AES");
			}
			else if(strstr(g24->security,"TKIP"))
			{
				strcpy(encrypt,"TKIP");
			}
			else
			{
				strcpy(encrypt,"AES");
			}
			
		}

		connectApPlatformlib(0,g24->ssid,g24->ch,authmode,encrypt,g24->key, g24->timeout);
		startApcliDhcpcPlatformlib(0);
	}
	
	if(g58 != NULL)
	{
		currentG58ApcliStatus = 1;
		memset(authmode,0,sizeof(authmode));
		memset(encrypt,0,sizeof(encrypt));
		if(strstr(g58->security,"NONE"))
		{
			strcpy(authmode,"OPEN");
			strcpy(encrypt,"NONE");
		}
		else if(strstr(g58->security,"WEP"))
		{
			strcpy(authmode,"OPEN");
			strcpy(encrypt,"WEP");
		}
		else if(strstr(g58->security,"WPA2PSK"))
		{
			strcpy(authmode,"WPA2PSK");
			if(strstr(g58->security,"AES"))
			{
				strcpy(encrypt,"AES");
			}
			else if(strstr(g58->security,"TKIP"))
			{
				strcpy(encrypt,"TKIP");
			}
			else
			{
				strcpy(encrypt,"AES");
			}
			
		}
		else
		{
			strcpy(authmode,"WPAPSK");
			if(strstr(g58->security,"AES"))
			{
				strcpy(encrypt,"AES");
			}
			else if(strstr(g58->security,"TKIP"))
			{
				strcpy(encrypt,"TKIP");
			}
			else
			{
				strcpy(encrypt,"AES");
			}
			
		}

		connectApPlatformlib(1,g58->ssid,g58->ch,authmode, encrypt,g58->key, g58->timeout);
		startApcliDhcpcPlatformlib(1);
	}

	return 0;
}


//save connect the ap, 0 means save ok,not connetct ok
static int commuSaveConnectApInfo(T_MKOS_APCLI_PARA *g24,T_MKOS_APCLI_PARA *g58)
{
	int result = -1;
	char authmode[16]={0};
	char encrypt[16]={0};
		

	if(g24 != NULL)
	{
		if(strstr(g24->security,"NONE"))
		{
			strcpy(authmode,"OPEN");
			strcpy(encrypt,"NONE");
		}
		else if(strstr(g24->security,"WEP"))
		{
			strcpy(authmode,"OPEN");
			strcpy(encrypt,"WEP");
		}
		else if(strstr(g24->security,"WPA2PSK"))
		{
			strcpy(authmode,"WPA2PSK");
			if(strstr(g24->security,"AES"))
			{
				strcpy(encrypt,"AES");
			}
			else if(strstr(g24->security,"TKIP"))
			{
				strcpy(encrypt,"TKIP");
			}
			else
			{
				strcpy(encrypt,"AES");
			}
			
		}
		else
		{
			strcpy(authmode,"WPAPSK");
			if(strstr(g24->security,"AES"))
			{
				strcpy(encrypt,"AES");
			}
			else if(strstr(g24->security,"TKIP"))
			{
				strcpy(encrypt,"TKIP");
			}
			else
			{
				strcpy(encrypt,"AES");
			}
			
		}

		saveConnectApInfoPlatformlib(0,g24->ssid,g24->ch,authmode,encrypt,g24->key, g24->timeout);
	}
	
	if(g58 != NULL)
	{
		memset(authmode,0,sizeof(authmode));
		memset(encrypt,0,sizeof(encrypt));
		if(strstr(g58->security,"NONE"))
		{
			strcpy(authmode,"OPEN");
			strcpy(encrypt,"NONE");
		}
		else if(strstr(g58->security,"WEP"))
		{
			strcpy(authmode,"OPEN");
			strcpy(encrypt,"WEP");
		}
		else if(strstr(g58->security,"WPA2PSK"))
		{
			strcpy(authmode,"WPA2PSK");
			if(strstr(g58->security,"AES"))
			{
				strcpy(encrypt,"AES");
			}
			else if(strstr(g58->security,"TKIP"))
			{
				strcpy(encrypt,"TKIP");
			}
			else
			{
				strcpy(encrypt,"AES");
			}
			
		}
		else
		{
			strcpy(authmode,"WPAPSK");
			if(strstr(g58->security,"AES"))
			{
				strcpy(encrypt,"AES");
			}
			else if(strstr(g58->security,"TKIP"))
			{
				strcpy(encrypt,"TKIP");
			}
			else
			{
				strcpy(encrypt,"AES");
			}
			
		}

		saveConnectApInfoPlatformlib(1,g58->ssid,g58->ch,authmode, encrypt,g58->key, g58->timeout);
	}

	return 0;
}


//get the connect the ap info, 0 means get ok
static int commuGetConnectApInfo(T_MKOS_APCLI_PARA *g24,T_MKOS_APCLI_PARA *g58)
{
	int result = -1;
	char authmode[16]={0};
	char encrypt[16]={0};
	int apclig24en = -1;
	int apclig58en = -1;	

	if(g24 != NULL)
	{
		apclig24en = getConnectApInfoPlatformlib(0,g24->enable,g24->ssid,g24->ch,authmode,encrypt,g24->key);
		if(!strcmp(encrypt,"NONE"))
		{
			strcpy(g24->security,"NONE");
		}
		else if(!strcmp(encrypt,"WEP"))
		{
			strcpy(g24->security,"WEP");
		}
		else
		{
			sprintf(g24->security,"%s/%s",authmode,encrypt);
		}
	}
	
	if(g58 != NULL)
	{
		memset(authmode,0,sizeof(authmode));
		memset(encrypt,0,sizeof(encrypt));
		apclig58en = getConnectApInfoPlatformlib(1,g58->enable,g58->ssid,g58->ch,authmode,encrypt,g58->key);
		if(!strcmp(encrypt,"NONE"))
		{
			strcpy(g58->security,"NONE");
		}
		else if(!strcmp(encrypt,"WEP"))
		{
			strcpy(g58->security,"WEP");
		}
		else
		{
			sprintf(g58->security,"%s/%s",authmode,encrypt);
		}
	}

	if((g24 != NULL) && (g58 == NULL))
	{
		if(apclig24en == 0)
		{
			result = 1;
		}
		else if(apclig24en == 1)
		{
			result = 0;
		}
		else
		{
			result = -1;
		}
	}
	else if((g24 == NULL) && (g58 != NULL))
	{
		if(apclig58en == 0)
		{
			result = 2;
		}
		else if(apclig58en == 1)
		{
			result = 0;
		}
		else
		{
			result = -1;
		}
	}
	else if((g24 != NULL) && (g58 != NULL))
	{
		if((apclig24en == 0) && (apclig58en == 0))
		{
			result = 3;
		}
		else if((apclig24en == 0) && (apclig58en == 1))
		{
			result = 1;
		}
		else if((apclig24en == 1) && (apclig58en == 0))
		{
			result = 2;
		}
		else if((apclig24en == 1) && (apclig58en == 1))
		{
			result = 0;
		}
		else
		{
			result = -1;
		}
	}
	else
	{
		result = -1;
	}
	return result;
}



//connect the ap status, 0 is mean command excute ok,return status=0 is connecting, status=1 is connect ok, status=-1 ,wifi connet ok bu not get the ipaddr, status=-2 ,password error, complete=0,all connect procedure end(2.4G ?��? 5G ???��????����???��?��???��쨨??????��??o?), complete=1, some connect procedure not end(??�먨????����???��??2??��쨨??????��????��?���|???��??-??��������??��???)
static int commuGetConnectApStatus(T_MKOS_APCLI_STATUS_INFO *apclig24status, T_MKOS_APCLI_STATUS_INFO *apclig58status)
{
	char apclig24enable[8]={0},apclig24ssid[64]={0},apclig24ch[8]={0},apclig24ip[32]={0},apclig24nm[32]={0};
	char apclig58enable[8]={0},apclig58ssid[64]={0},apclig58ch[8]={0},apclig58ip[32]={0},apclig58nm[32]={0};
	int getg24ip=0,getg58ip=0;
	int isConn=-1,connErr=9,rssi=0;
	unsigned long rate=0;
	char useacliname[32]={0};
		
	nvram_get_Platformlib(RT2860_NVRAM,"ApCliEnable", apclig24enable, sizeof(apclig24enable));
	nvram_get_Platformlib(RT2860_NVRAM,"useApcliName", useacliname, sizeof(useacliname));
	
	if(atoi(apclig24enable) == 1)
	{
		nvram_get_Platformlib(RT2860_NVRAM,"ApCliSsid", apclig24ssid, sizeof(apclig24ssid));
		nvram_get_Platformlib(RT2860_NVRAM,"Channel", apclig24ch, sizeof(apclig24ch));
		
		apclig24status->is_enable = 1;
		strcpy(apclig24status->ssid,apclig24ssid);
		strcpy(apclig24status->ch,apclig24ch);
		if(currentG24ApcliStatus)
		{
			apcliConnectStatusPlatformlib(0,&isConn,&connErr, &rssi ,&rate);
			if(strcmp(useacliname,"none"))
			{
				getg24ip = getIfIpComlib(LAN_IF_NALE_ALIS,apclig24ip);
				getIfMaskComlib(LAN_IF_NALE_ALIS,apclig24nm);
			}
			else
			{
				getg24ip = getIfIpComlib(WAN_WISP_NAME,apclig24ip);
				getIfMaskComlib(WAN_WISP_NAME,apclig24nm);
			}
			apclig24status->rssi = rssi;
			apclig24status->rate = rate;
			if(getg24ip == 1)
			{
				apclig24status->status = 1;
				strcpy(apclig24status->ip,apclig24ip);
				strcpy(apclig24status->nm,apclig24nm);
			}
			else
			{
				if(isConn == 1)
				{
					apclig24status->status = -1;
				}
				else
				{
					if(connErr == 3 || connErr == 1)
					{
						apclig24status->status = -2;
					}
					else if(connErr == 0)
					{
						apclig24status->status = -3;
					}
					else if(connErr == 2)
					{
						apclig24status->status = -4;
					}
					else
					{
						apclig24status->status = -5;
					}
				}
					
				strcpy(apclig24status->ip,"0.0.0.0");
				strcpy(apclig24status->nm,"0.0.0.0");
			}
		}
		else
		{
			apclig24status->status=-9;
			strcpy(apclig24status->ip,"0.0.0.0");
			strcpy(apclig24status->nm,"0.0.0.0");
		}
	}
	else
	{
		apclig24status->is_enable = 0;
		apclig24status->status=-9;
	}


	nvram_get_Platformlib(RTDEV_NVRAM,"ApCliEnable", apclig58enable, sizeof(apclig58enable));
	if(atoi(apclig58enable) == 1)
	{
		nvram_get_Platformlib(RTDEV_NVRAM,"ApCliSsid", apclig58ssid, sizeof(apclig58ssid));
		nvram_get_Platformlib(RTDEV_NVRAM,"Channel", apclig58ch, sizeof(apclig58ch));
		
		apclig58status->is_enable = 1;
		strcpy(apclig58status->ssid,apclig58ssid);
		strcpy(apclig58status->ch,apclig58ch);
		if(currentG58ApcliStatus)
		{
			isConn = rssi =0;
			connErr = 9;
			apcliConnectStatusPlatformlib(1,&isConn,&connErr, &rssi ,&rate);
			if(strcmp(useacliname,"none"))
			{
				getg58ip = getIfIpComlib(LAN_IF_NALE_ALIS,apclig58ip);
				getIfMaskComlib(LAN_IF_NALE_ALIS,apclig58nm);
			}
			else
			{
				getg58ip = getIfIpComlib(WAN_WISP_NAME_5G,apclig58ip);
				getIfMaskComlib(WAN_WISP_NAME_5G,apclig58nm);
			}
			apclig58status->rssi = rssi;
			apclig58status->rate = rate;
			if(getg58ip == 1)
			{
				apclig58status->status = 1;
				strcpy(apclig58status->ip,apclig58ip);
				strcpy(apclig58status->nm,apclig58nm);
			}
			else
			{
				if(isConn == 1)
				{
					apclig58status->status = -1;
				}
				else
				{
					if(connErr == 3 || connErr == 1)
					{
						apclig58status->status = -2;
					}
					else if(connErr == 0)
					{
						apclig58status->status = -3;
					}
					else if(connErr == 2)
					{
						apclig58status->status = -4;
					}
					else
					{
						apclig58status->status = -5;
					}
				}
				strcpy(apclig58status->ip,"0.0.0.0");
				strcpy(apclig58status->nm,"0.0.0.0");
			}
		}
		else
		{
			apclig58status->status=-9;
			strcpy(apclig58status->ip,"0.0.0.0");
			strcpy(apclig58status->nm,"0.0.0.0");
		}
	}
	else
	{
		apclig58status->is_enable = 0;
		apclig58status->status=-9;
	}
	
	return 0;
}


// stop the connect ap, mode=0 is stop 2.4g, mode=1 is stop 5.8g ,mode=2 are both
static int commuStopConnectAp(int mode)
{
	int result = 0;

	if(mode == 0)
	{
		stopConnectApPlatformlib(0);
		currentG24ApcliStatus =0;
	}
	else if(mode == 1)
	{
		stopConnectApPlatformlib(1);
		currentG58ApcliStatus=0;
	}
	else if(mode == 2)
	{
		stopConnectApPlatformlib(0);
		stopConnectApPlatformlib(1);
		currentG24ApcliStatus=0;
		currentG58ApcliStatus=0;
	}
	else
	{
		result = -1;
	}
	return result;
}


// set the extend ap ,mode =0 is set 2.4g, mode=1 is set 5.8 g, return 0 is set ok
static int commuSetExtAp(int mode,char *enable,char *hidden,char *ssid, char *security, char *pass, char *channel)
{
	int result = 0;

	result= setExtApPlatformlib(mode,enable,hidden,ssid,security,pass,channel);
	
	return result;
}

// get the extend ap info ,mode =0 is get 2.4g, mode=1 is get 5.8 g, return 0 is get ok
static int commuGetExtAp(int mode,char *enable,char *hidden,char *ssid, char *security, char *pass, char *channel)
{
	int result = 0;

	result= getExtApPlatformlib(mode,enable,hidden,ssid,security,pass,channel);
	
	return result;
}


// save the extend ap info ,mode =0 is get 2.4g, mode=1 is get 5.8 g, return 0 is get ok
static int commuSaveExtAp(int mode,char *enable,char *hidden,char *ssid, char *security, char *pass, char *channel)
{
	int result = 0;

	result= saveExtApPlatformlib(mode,enable,hidden,ssid,security,pass,channel);
	
	return result;
}


//select the apclient interface to use ,0 is 2.4g, 1 is 5.8g
static int commuSelectAplientInterfaceToUse(int mode)
{
	int result = 0;

	result= selectAplientInterfaceToUsePlatformlib(mode);
	startBridgeDhcpcPlatformlib();
	
	return result;
}


//get the current apclient interface to use ,reurn 0 is 2.4g, 1 is 5.8g,other is error
static int commuGetAplientInterfaceToUse()
{
	int result = 0;

	result= getAplientInterfaceToUsePlatformlib();
	
	return result;
}


// test 2.4g and 5.8g apcli speed, return 0 means 2.4g, 1 means 5.8g ,others is error
static int commuGetSpeedTestStatusWAN(T_MKOS_APCLI_WAN_SPEEDTEST_INFO *g24,T_MKOS_APCLI_WAN_SPEEDTEST_INFO *g58)
{
	int result = -1;
	
	result = apcliSpeedTestPlatformlib(&(g24->sends),&(g24->losts),&(g24->maxRtt),&(g24->minRtt),&(g24->averRtt),&(g24->rssi),&(g58->sends),&(g58->losts),&(g58->maxRtt),&(g58->minRtt),&(g58->averRtt),&(g58->rssi));
	
	return result;
}

// new single test 2.4g and 5.8g apcli speed, return 0 is ok ,others is error
static int commuGetSingleSpeedTestStatusSetupLAN(int mode,int totalPackets,int totalTimeout,int oneTimeout,T_MKOS_APCLI_LAN_SPEEDTEST_INFO *testResult)
{

	int result = -1;
	
	result = apcliSingleSpeedTestSetupLanPlatformlib(mode,totalPackets,totalTimeout,oneTimeout,&(testResult->sends),&(testResult->losts),&(testResult->maxRtt),&(testResult->minRtt),&(testResult->averRtt),&(testResult->allRtt),&(testResult->rssi));
	
	return result;
}

// bridge apcli speed, return 0 is ok ,others is error
static int commuGetBrSpeedTestStatusRunLAN(int totalPackets,int totalTimeout,int oneTimeout,T_MKOS_APCLI_LAN_SPEEDTEST_INFO *testResult)
{

	int result = -1;
	
	result = apcliSingleSpeedTestRunBrPlatformlib(totalPackets,totalTimeout,oneTimeout,&(testResult->sends),&(testResult->losts),&(testResult->maxRtt),&(testResult->minRtt),&(testResult->averRtt),&(testResult->allRtt),&(testResult->rssi));
	
	return result;
}


//control the led return 0 is set ok
static int commuSetLedControl(int led,int on,int off)
{
	int result = 0;

	if(led !=1 && led !=2 && led != 3)
	{
		return -1;
	}
	if(on == 0 && off == 0)
	{
		return -2;
	}

	result= setLedControPlatformlib(led,on,off);
	
	return result;
}

//get the current sta list buf, return 0 is get ok
static int commuGetCurrentWifiStaListBuf(char *listBuff)
{
	int result = -1;

	result= getCurrentWifiStaListBufPlatformlib(listBuff);
	
	return result;
}


//get the WIFI ACL access police
static int commuGetWifiAclPolicy()
{
	int result = -1;

	result= getWifiAclPolicyPlatformlib();
	
	return result;
}

//set the WIFI ACL access police 0 meas set ok
static int commuSetWifiAclPolicy(int mode)
{
	int result = -1;

	result= setWifiAclPolicyPlatformlib(mode);
	
	return result;
}

//get the WIFI ACL white list
static int commuGetWifiAclWhiteList(char *aclListBuff)
{
	int result = -1;

	result= getWifiAclWhiteListPlatformlib(aclListBuff);
	
	return result;
}

//get the WIFI ACL blank list
static int commuGetWifiAclBlankList(char *aclListBuff)
{
	int result = -1;

	result= getWifiAclBlankListPlatformlib(aclListBuff);
	
	return result;
}

//add  WIFI ACL white entry
static int commuAddWifiAclWhiteEntry(char *macaddr)
{
	int result = -1;

	result= addWifiAclWhiteEntryPlatformlib(macaddr);
	
	return result;
}

//add the WIFI ACL blank list
static int commuAddWifiAclBlankEntry(char *macaddr)
{
	int result = -1;

	result= addWifiAclBlankEntryPlatformlib(macaddr);
	
	return result;
}

//delete  WIFI ACL white entry
static int commuDelWifiAclWhiteEntry(char *macaddr)
{
	int result = -1;

	result= delWifiAclWhiteEntryPlatformlib(macaddr);
	
	return result;
}

//delete the WIFI ACL blank list
static int commuDelWifiAclBlankEntry(char *macaddr)
{
	int result = -1;

	result= delWifiAclBlankEntryPlatformlib(macaddr);
	
	return result;
}

//delete  All WIFI ACL white entry
static int commuDelAllWifiAclWhiteEntry()
{
	int result = -1;

	result= delAllWifiAclWhiteEntryPlatformlib();
	
	return result;
}

//delete All the WIFI ACL blank list
static int commuDelAllWifiAclBlankEntry()
{
	int result = -1;

	result= delAllWifiAclBlankEntryPlatformlib();
	
	return result;
}

//get the all led contrl rule
static int commuGetALLledControl()
{
	int result = -1;

	result= getALLledControlPlatformlib();
	
	return result;
}

//set the all led contrl rule
static int commuSetALLledControl(int type)
{
	int result = -1;

	result= setALLledControlPlatformlib(type);
	
	return result;
}

//disable the apcli mode=0 is 2.4g ,mode =1 is 5.8g ,mode =2 are both
static int commuDisableApcliConnect(int mode)
{
	int result = -1;

	result= disableApcliConnectPlatformlib(mode);
	
	return result;
}


//get the mac name  info list
static int commuMacNameInfoList(char *macNameBuff)
{
	int result = -1;

	result= macNameInfoListPlatformlib(macNameBuff);
	
	return result;
}


//add  mac name   entry
static int commuAddMacNameEntry(char *macaddr, char *alisName)
{
	int result = -1;

	result= addMacNameEntryPlatformlib(macaddr,alisName);
	
	return result;
}

//get the system config status 0 is not config, 1 is config, -1 is error
static int commuGetSystemConfig()
{
	int result = -1;

	result= getSystemConfigPlatformlib();
	
	return result;
}

//set the system config status ,return 0 is set ok ,type =1 means config , 0 is not config
static int commuSetSystemConfig(int type)
{
	int result = -1;

	result= setSystemConfigPlatformlib(type);
	
	return result;
}



// start speed test, 0 means command execute ok, 
//param[in] 
//type_in:       unsigned char, =0 is LAN speed test, =1 is WAN speed test, 
//set_test_band: unsigned char, =0 means use the band which transmit data now, =1 means use 2.4G to test speed, =2 means use 5G to test speed,
//param[out]
//status:        unsigned char*, =0 means success, =1 means faild(eg. not in connection), =2 means busy(eg. during speed test)
//type_out:      unsigned char*, =0 is LAN speed test, =1 is WAN speed test, 
//real_test_band:unsigned char*, =0 means unknown, =1 means 2.4G, =2 means 5G,
//rssi_2_4G:     singed char*,(eg. -65), if not in 2.4G connection, set to 100,
//rssi_5G:       singed char*,(eg. -65), if not in 5G connection, set to 100,
static int commuStartSpeedTest(unsigned char type_in, unsigned char set_test_band,
unsigned char* status, unsigned char* type_out, unsigned char* real_test_band, signed char* rssi_2_4G, signed char* rssi_5G )
{
	*status = 0;
	*type_out = type_in;
	if(set_test_band != 0)
		*real_test_band = set_test_band;
	else
		*real_test_band = 1;//means 2.4G
	*rssi_2_4G = -73;
	*rssi_5G = 100;
	return 0;
}


// get LAN speed test status, 0 means command execute ok,
//param[in]
//null
//param[out]
//type_out:      unsigned char*, =0 is LAN speed test, =1 is WAN speed test, 
//test_progress: unsigned char*, range: 1<= test_progress <= 100, 
//test_result:   unsigned char*, =0 means testing, =1 means test complete, =2 means test faild,
//speed:         int*, unit is bit/s, the real time speed, =0 means no this value(eg. test faild)
//ping_min:      short int*, unit is ms, just set this value in the test result, otherwise, set to 0xFFFF, if no this value in the test result, also set to 0xFFFF,
//ping_max:      short int*, unit is ms, just set this value in the test result, otherwise, set to 0xFFFF, if no this value in the test result, also set to 0xFFFF,
//ping_avg:      short int*, unit is ms, just set this value in the test result, otherwise, set to 0xFFFF, if no this value in the test result, also set to 0xFFFF,
//static int commuGetSpeedTestStatusLAN()
//{
//
//	return 0;
//}

int uart_open(char* port)  
{  
    int fd;
	fd = open( port, O_RDWR|O_NDELAY);//O_NOCTTY  
	if (fd < 0)  
	{  
	   DBG("Can't Open Serial Port");  
	   return FALSE;  
	}  
	if(fcntl(fd, F_SETFL, 0) < 0)  
	{  
		DBG("fcntl failed!\n");  
	 	return FALSE;  
	}       
	else  
	{  
		DBG("fcntl=%d\n",fcntl(fd, F_SETFL,0));  
	}  
	
	DBG("fd->open=%d\n",fd);  
	return fd;  
}  


void uart_close(int fd)  
{  
    close(fd);  
}  


int uart_set_speed(int fd, int speed)
{  
	int   i;   
	int   status;   
	struct termios   Opt;
	int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300, };  
	int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,  115200, 38400, 19200, 9600, 4800, 2400, 1200,  300, };  
	if( tcgetattr( fd,&Opt)  !=  0)  
	{  
		perror("setspeed");      
		return FALSE;   
	}  
	for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) 
	{   
		if  (speed == name_arr[i]) 
		{       
			tcflush(fd, TCIOFLUSH);       
			cfsetispeed(&Opt, speed_arr[i]);    
			cfsetospeed(&Opt, speed_arr[i]);     
			status = tcsetattr(fd, TCSANOW, &Opt);    
			if (status != 0) 
			{          
				perror("tcsetattr fd1");    
				return FALSE;       
			}      
			tcflush(fd,TCIOFLUSH);     
		}    
	}
	return TRUE;
}  


int uart_set_parity(int fd,int databits,int stopbits,int parity)  
{   
	struct termios options;   
	if  ( tcgetattr( fd,&options)  !=  0) 
	{   
		perror("SetParity");       
		return FALSE;    
	}  
	options.c_cflag &= ~CSIZE;   
	switch (databits)
	{     
		case 7:       
			options.c_cflag |= CS7;   
			break;  
		case 8:       
			options.c_cflag |= CS8;  
			break;     
		default:      
			fprintf(stderr,"Unsupported data size\n"); 
			return FALSE;    
	}  
	switch (parity)   
	{     
		case 'n':  
		case 'N':      
			options.c_cflag &= ~PARENB;   /* Clear parity enable */  
			options.c_iflag &= ~INPCK;     /* Enable parity checking */  
		break;    
		case 'o':     
		case 'O':       
			options.c_cflag |= (PARODD | PARENB);  
			options.c_iflag |= INPCK;             /* Disnable parity checking */   
		break;    
		case 'e':    
		case 'E':     
			options.c_cflag |= PARENB;     /* Enable parity */      
			options.c_cflag &= ~PARODD;        
			options.c_iflag |= INPCK;       /* Disnable parity checking */  
			break;  
		case 'S':   
		case 's':  /*as no parity*/     
			options.c_cflag &= ~PARENB;  
			options.c_cflag &= ~CSTOPB;
			break;    
		default:     
			fprintf(stderr,"Unsupported parity\n");      
			return FALSE;    
	}     
	switch (stopbits)  
	{     
		case 1:      
			options.c_cflag &= ~CSTOPB;    
			break;    
		case 2:      
			options.c_cflag |= CSTOPB;    
			break;  
		default:      
			fprintf(stderr,"Unsupported stop bits\n");    
			return FALSE;   
	}   
	/* Set input parity option */   
	if (parity != 'n' && parity != 'N')     
		options.c_iflag |= INPCK;
	options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
	options.c_cflag |= CLOCAL;	
	options.c_cflag |= CREAD;
  	options.c_oflag &= ~OPOST;

	options.c_iflag &= ~(ICRNL | IXON);
	options.c_oflag &= ~(ONLCR | OCRNL);  
	options.c_iflag &= ~(ICRNL | INLCR);
	options.c_iflag &= ~(IXON | IXOFF | IXANY); 
	tcflush(fd,TCIFLUSH);  
	options.c_cc[VTIME] = 0;   
	options.c_cc[VMIN] = 0; 
	
	if (tcsetattr(fd,TCSANOW,&options) != 0)     
	{   
		perror("SetupSerial 3");     
		return FALSE;    
	}
	return TRUE;

}  
////////////////////////////////////////////////////////////////////////////////


int validFileMd5(char *file, char *md5)
{
	return 0;
}


int get_usb_status(void)
{
	return 0;
}

int init_usb_uart(void )
{
	/* 打开串口并且配置串口 */
	if(0 == get_usb_status())
	{
		DBG("%s is insert\n",UART_DEVICE);
		uart_fd = uart_open(UART_DEVICE);
	}
	else
	{
		DBG("%s is not insert\n",UART_DEVICE);
		return 1;
	}
	if(uart_fd <= 0)
	{
		DBG("open uart error\n");
		return 2;
	}

	DBG("%s is opened\n",UART_DEVICE);
	sleep(1);
	uart_set_speed(uart_fd,BAUDRATE);
	sleep(1);
	if (uart_set_parity(uart_fd,8,1,'N') == -1)
	{
		DBG("Set Parity Error\n");
		close(uart_fd);
		uart_fd=-1;
		return 3;
	}
	DBG("UART_SetParity...ok\n");
	sleep(1);
	tcflush(uart_fd,TCIOFLUSH);
	sleep(1);

	return 0;
}


static int get_nvram_para_all(void)
{	
	char localtmpBuf[64]={0};
	int result = 1;
	
	/* ��ȡSN�� */
	if(get_mib(gMyboxSn,NVRAM_AP_SN))
	{
		result = 0;
	    strcpy(gMyboxSn,DEVICE_DEFAULT_SN);
		DBG("default deviceApSn = %s\n",gMyboxSn);
	}
	else
	{
	    DBG("nvram_get %s = %s\n",NVRAM_AP_SN,gMyboxSn);
	}

	/* ��ȡ��������ַ */
	if(get_mib(gCloudServerAddr,NVRAM_CLOUD_ADDR))
	{
		result = 0;
	    DBG("nvram_get %s so use default\n",NVRAM_CLOUD_ADDR);
	    strcpy(gCloudServerAddr,CLOUD_DEFAULT_SERVER);
	    DBG("default cloudServerAddr = %s\n",gCloudServerAddr);
	}
	else
	{
	    DBG("nvram_get %s = %s\n",NVRAM_CLOUD_ADDR,gCloudServerAddr);
	}

	/* ��ȡ�������˿� */
	if(get_mib(localtmpBuf,NVRAM_CLOUD_PORT))
	{
		result = 0;
	    DBG("nvram_get %s so use default\n",NVRAM_CLOUD_PORT);
	    gCloudServerPort=CLOUD_DEFAULT_PORT;
	    DBG("default cloudServerPort = %d\n",gCloudServerPort);
	}
	else
	{
		
		gCloudServerPort = atoi(localtmpBuf);
    	DBG("nvram_get %s = %s--%d\n",NVRAM_CLOUD_PORT,localtmpBuf,gCloudServerPort);
	}
	
    return result;
}

static int str2int(char *s)
{
	int r = 0, i = 0;
	switch(s[0])
	{
		case '-':
		{
			i = 1;
			while(s[i] != '\0')
			{
				r = r*10 + (s[i++] - '0');
			}
			r = 0 -r;
		}break;
		case '+':
		{
			i = 1;
			while(s[i] != '\0')
			{
				r = r*10 + (s[i++] - '0');
			}
		}break;
		default:
		{
			while(s[i] != '\0')
			{
				r = r*10 + (s[i++] - '0');
			}
		}
	}
	
	return r;
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

static unsigned int hex2dstr(unsigned int hex,char *pstr)
{
	unsigned int i,len = 0;
	char *p_start,*p_end,tmp;
	char dex[] = "0123456789";
	p_start = pstr;
	for(i = 0; i < 10; i++)
	{
		*pstr++ = dex[hex%10];
		hex = hex/10;
		if(hex <= 0)break;
	}
	p_end = pstr -1;
	len = i;
	for(i = 0; i <(p_end - p_start)/2; i++)
	{
		tmp = *p_start;
		*p_start = *p_end;
		*p_end = tmp;
		p_start++;
		p_end--;
	}
	return len;
}

static unsigned int tz2str(signed char tz, char *pstr)
{
	char *p_start,*p_end;
	char dex[] = "0123456789";

	p_start = pstr;
	if(tz < 0)
	{
		*pstr++ = '-';
		tz = 0 - tz;
	}
	if((tz&0x0F)>= 10)
	{
		*pstr++ = dex[(tz&0x0F)/10];
	}
	*pstr++ = dex[(tz&0x0F)%10];
	if((tz&0x10))
	{
		*pstr++ = '.';
		*pstr++ = dex[5];
	}
	p_end = pstr -1;
	return pstr - p_start;
}

static void praseRouteList(char *buf, ROUTE_T *route, unsigned int *cnt)
{
	unsigned int k = 0, m = 0, route_cnt = *cnt;
	
	while(buf[k] != '\0')
	{
		//while(buf[k] != '#')
		//{
			while(buf[k] != '\n')
			{
				route[route_cnt].ssid[m] = buf[k];
				k++;
				m++;
			}
			route[route_cnt].ssid[m] = '\0';
			route[route_cnt].ssid_len = m;
			k++;
			m = 0;
			DBG(L_BLUE"route[%d].ssid: %s\n"NONE,route_cnt,route[route_cnt].ssid);
			while(buf[k] != '\n')
			{
				route[route_cnt].ch[m] = buf[k];
				k++;
				m++;
			}
			route[route_cnt].ch[m] = '\0';
			route[route_cnt].ch_len = m;
			k++;
			m = 0;
			DBG("%s ",route[route_cnt].ch);
			while(buf[k] != '\n')
			{
				route[route_cnt].security[m] = buf[k];
				k++;
				m++;
			}
			route[route_cnt].security[m] = '\0';
			route[route_cnt].security_len = m;
			k++;
			m = 0;
			DBG("%s ",route[route_cnt].security);
			while(buf[k] != '\n')
			{
				route[route_cnt].rssi[m] = buf[k];
				k++;
				m++;
			}
			route[route_cnt].rssi[m] = '\0';
			route[route_cnt].rssi_len = m;
			k++;
			m = 0;
			DBG("%s ",route[route_cnt].rssi);
			while((buf[k] != '\n')&&(buf[k] != '\0'))
			{
				route[route_cnt].bssid[m] = buf[k];
				k++;
				m++;
			}
			route[route_cnt].bssid[m] = '\0';
			route[route_cnt].bssid_len = m;
			k++;
			m = 0;
			DBG("%s\n",route[route_cnt].bssid);
		//}
		route[route_cnt].route_len = route[route_cnt].bssid_len + route[route_cnt].ch_len + route[route_cnt].rssi_len + route[route_cnt].security_len + route[route_cnt].ssid_len;
		//DBG("route[%d].route_len: %d\n",route_cnt,route[route_cnt].route_len);
		route_cnt++;
		if(route_cnt >= MAX_ROUTE_LIST_NUM)
		{
			route_cnt = 0;
		}
		//k++;
	}
	*cnt = route_cnt;
}

static void praseDevListNow(char *buf, T_MKOS_DEV_LIST_NOW *dev, unsigned int *cnt)
{
	unsigned int k = 0, m = 0, dev_cnt = *cnt;

	while(buf[k] != '\0')
	{
		//while(buf[k] != '\n')
		{
			/*if(buf[k] == ' ')
			{
				k++;
				continue;
			}*/
			//while(buf[k] != ' ')
			{
				//get mac
				int i = 0;
				char mac_str[16] = {0};
				for(i = 0; i < 12; )//delete ':'
				{
					if(buf[k] == ':')
					{
						k++;
						continue;
					}
					mac_str[i] = buf[k];
					k++;
					i++;
				}
				DBG(YELLOW"mac_str: %s\n"NONE,mac_str);
				for(i = 0; i < 12; i+=2)//str to hex
				{
					dev[dev_cnt].mac[i/2] = (((mac_str[i] >= 'A')?(mac_str[i] -'A' +0x0A):(mac_str[i] - '0'))<<4) + ((mac_str[i+1] >= 'A')?(mac_str[i+1] -'A' +0x0A):(mac_str[i+1] - '0'));
				}
				DBG(YELLOW"dev[dev_cnt].mac: OK\n"NONE);
			}
			
			k++;//jump ' '
			{
				//get band
				dev[dev_cnt].band = buf[k] - '0';
				k++;
				k++;//jump 'G'
				DBG(YELLOW"band: %d\n"NONE,dev[dev_cnt].band);
			}
			k++;//jump ' '
			{
				//get speed_tx
				char speed_tx_str[8] = {0};
				int i = 0;
				while(buf[k] != ' ')
				{
					speed_tx_str[i] = buf[k];
					k++;
					i++;
				}
				dev[dev_cnt].speed_tx = str2int(speed_tx_str);
				DBG(YELLOW"speed_tx_str: %s\n"NONE,speed_tx_str);
			}
			k++;//jump ' '
			{
				//get bandwidth
				char bandwidth_str[8] = {0};
				int i = 0;
				while(buf[k] != ' ')
				{
					if(buf[k] == 'M')
					{
						k++;
						continue;
					}
					bandwidth_str[i] = buf[k];
					k++;
					i++;
				}
				strcpy(dev[dev_cnt].bandwidth, bandwidth_str);
				DBG(YELLOW"bandwidth_str: %s\n"NONE,bandwidth_str);
			}
			k++;//jump ' '
			{
				//get rssi
				char rssi_str[8] = {0};
				int i = 0;
				while(buf[k] != ' ')
				{
					rssi_str[i] = buf[k];
					k++;
					i++;
				}
				dev[dev_cnt].rssi = str2int(rssi_str);
				DBG(YELLOW"rssi_str: %s\n"NONE,rssi_str);
			}
			k++;//jump ' '
			{
				//get time_online
				char time_online_str[8] = {0};
				int i = 0;
				while((buf[k] != '\n') && (buf[k] != '\0'))
				{
					time_online_str[i] = buf[k];
					k++;
					i++;
				}
				dev[dev_cnt].time_online = str2int(time_online_str);
				DBG(YELLOW"time_online_str: %s\n"NONE,time_online_str);
			}
			dev[dev_cnt].name_len = 1;//just '\0'
			dev[dev_cnt].remark_name_len = 1;//just '\0'
			dev[dev_cnt].dev_len = 1+1+4+6+1+4+dev[dev_cnt].name_len+dev[dev_cnt].remark_name_len;
			/*k++;//jump ' '
			{
				//get name
				int i = 0;
				while(buf[k] != ' ')
				{
					dev[dev_cnt].name[i] = buf[k];
					k++;
					i++;
				}
				dev[dev_cnt].name[i++] = '\0';
				dev[dev_cnt].name_len = i;//include '\0'
				DBG(YELLOW"dev[dev_cnt].name: %s\n"NONE,dev[dev_cnt].name);
			}
			k++;//jump ' '
			{
				//get remark name
				int i = 0;
				while((buf[k] != '\n') && (buf[k] != '\0'))
				{
					dev[dev_cnt].remark_name[i] = buf[k];
					k++;
					i++;
				}
				dev[dev_cnt].remark_name[i++] = '\0';
				dev[dev_cnt].remark_name_len = i;//include '\0'
				DBG(YELLOW"dev[dev_cnt].remark_name: %s\n"NONE,dev[dev_cnt].remark_name);
			}
			//dev[dev_cnt].name_len = 1;//just '\0'
			//dev[dev_cnt].remark_name_len = 1;//just '\0'
			dev[dev_cnt].dev_len = 1+1+4+6+1+4+dev[dev_cnt].name_len+dev[dev_cnt].remark_name_len;
			*/
		}
		if(buf[k] == '\n')
		{
			k++;//jump '\n'
		}
		
		DBG(YELLOW"after while(buf[k] != n)\n"NONE);
		dev_cnt++;
		if(dev_cnt >= 32)
		{
			dev_cnt = 0;
		}
	}
	DBG(YELLOW"dev_cnt: %d\n"NONE,dev_cnt);
	*cnt = dev_cnt;
}

static void praseDevListNowRemarkName(char *buf, T_MKOS_DEV_LIST_NOW *dev)
{
	unsigned int k = 0, m = 0;
	struct
	{
		unsigned char mac[6];
		char rn[64];
	}mrn = {0};

	while(buf[k] != '\0')
	{
		{
			//get mac
			int i = 0;
			char mac_str[16] = {0};
			for(i = 0; i < 12; )//delete ':'
			{
				if(buf[k] == ':')
				{
					k++;
					continue;
				}
				mac_str[i] = buf[k];
				k++;
				i++;
			}
			DBG(YELLOW"mac_str: %s\n"NONE,mac_str);
			for(i = 0; i < 12; i+=2)//str to hex
			{
				mrn.mac[i/2] = (((mac_str[i] >= 'A')?(mac_str[i] -'A' +0x0A):(mac_str[i] - '0'))<<4) + ((mac_str[i+1] >= 'A')?(mac_str[i+1] -'A' +0x0A):(mac_str[i+1] - '0'));
			}
			DBG(YELLOW"dev[dev_cnt].mac: OK\n"NONE);
		}
	
		{
			//get remark name
			int i = 0;
			while((buf[k] != ';') && (buf[k] != '\0'))
			{
				mrn.rn[i] = buf[k];
				k++;
				i++;
			}
			mrn.rn[i++] = '\0';
			DBG(YELLOW"mrn.rn: %s\n"NONE,mrn.rn);
		}
		if(buf[k] == ';')
		{
			k++;//jump ';'
		}
		{
			//set remark name
			int i = 0;
			for(m = 0; m < 32; m++)
			{
				for(i = 0; i < 6; i++)//search mac
				{
					if(dev[m].mac[i] != mrn.mac[i])
					{
						break;
					}
				}
				if(i == 6)//find the dev, set remark name
				{
					strcpy(dev[m].remark_name,mrn.rn);
					dev[m].remark_name_len = strlen(mrn.rn) + 1;// include the '\0'
					dev[m].dev_len += (dev[m].remark_name_len -1);
				}
				
			}
		}
	}
}

static void praseWriteList(char *buf, T_MKOS_WRITE_LIST *dev, unsigned int *cnt)
{
	unsigned int k = 0, m = 0, dev_cnt = *cnt;

	while(buf[k] != '\0')
	{
		//while(buf[k] != '\n')
		{
			
			//get mac
			int i = 0;
			char mac_str[16] = {0};
			for(i = 0; i < 12; )//delete ':'
			{
				if(buf[k] == ':')
				{
					k++;
					continue;
				}
				mac_str[i] = buf[k];
				k++;
				i++;
			}
			for(i = 0; i < 12; i+=2)//str to hex
			{
				dev[dev_cnt].mac[i/2] = (((mac_str[i] >= 'A')?(mac_str[i] -'A' +0x0A):(mac_str[i] - '0'))<<4) + ((mac_str[i+1] >= 'A')?(mac_str[i+1] -'A' +0x0A):(mac_str[i+1] - '0'));
			}
		}
		if(buf[k] == '\n')
		{
			k++;//jump '\n'
		}
		
		dev_cnt++;
		if(dev_cnt >= 200)
		{
			dev_cnt = 0;
		}
	}
	*cnt = dev_cnt;
}

static void praseBlackList(char *buf, T_MKOS_BLACK_LIST *dev, unsigned int *cnt)
{
	unsigned int k = 0, m = 0, dev_cnt = *cnt;

	while(buf[k] != '\0')
	{
		//while(buf[k] != '\n')
		{
			
			//get mac
			int i = 0;
			char mac_str[16] = {0};
			for(i = 0; i < 12; )//delete ':'
			{
				if(buf[k] == ':')
				{
					k++;
					continue;
				}
				mac_str[i] = buf[k];
				k++;
				i++;
			}
			for(i = 0; i < 12; i+=2)//str to hex
			{
				dev[dev_cnt].mac[i/2] = (((mac_str[i] >= 'A')?(mac_str[i] -'A' +0x0A):(mac_str[i] - '0'))<<4) + ((mac_str[i+1] >= 'A')?(mac_str[i+1] -'A' +0x0A):(mac_str[i+1] - '0'));
			}
		}
		if(buf[k] == '\n')
		{
			k++;//jump '\n'
		}
		
		dev_cnt++;
		if(dev_cnt >= 200)
		{
			dev_cnt = 0;
		}
	}
	*cnt = dev_cnt;
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

static void send2UartBuf(STRU_BLE_DATA *data)
{
	
	unsigned int i = 0;
	unsigned char checksum = 0x00;
	unsigned int ps_in_tmp = 0;

	ps_in_tmp = ps_in;
	//打包�?
	uartSendBuf[ps_in] = WIFIAMP_DEV_HEADER;
	ps_in = ptr_plus(ps_in,1);
	uartSendBuf[ps_in] = HI_UINT16(data->len + 4);
	ps_in = ptr_plus(ps_in,1);
	uartSendBuf[ps_in] = LO_UINT16(data->len + 4);
	ps_in = ptr_plus(ps_in,1);
	uartSendBuf[ps_in] = data->state;
	ps_in = ptr_plus(ps_in,1);
	uartSendBuf[ps_in] = data->seq_id;
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
	//计算校验�?
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

static void packDeviceCmd(STRU_BLE_DATA *data)
{
	unsigned int i = 0;

	if(data->is_resp == 0)// not response cmd, neeed seq++, and override data->seq_id
	{
		data->seq_id = seq_id;
		seq_id++;
		if(seq_id >=255){seq_id = 0;}
	}
	
	send2UartBuf(data);
	
	if(data->need_resend == 1)
	{
		data->need_resend = 0;
		//copy to resend buffer
		g_resend_cmd.has_cmd = 1;
		g_resend_cmd.is_cmd_changed = 1;
		g_resend_cmd.resend_cnt = 0;//clear counter value
		g_resend_cmd.resend_to = 0;//clear timerout counter value
		g_resend_cmd.cmd.need_resend = data->need_resend;
		g_resend_cmd.cmd.state = data->state;
		g_resend_cmd.cmd.seq_id = data->seq_id;
		g_resend_cmd.cmd.product_type = data->product_type;
		g_resend_cmd.cmd.cmd_id = data->cmd_id;
		g_resend_cmd.cmd.len = data->len;
		g_resend_cmd.cmd.buff = g_resend_cmd.cmd_data;
		for(i = 0; i < data->len; i++)
		{
			g_resend_cmd.cmd_data[i] = data->buff[i];
		}
	}
	
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
		DBG(GREEN"START PRASE CMD\n"NONE);
	while(p_out != p_in)
	{
		//DBG("p_out: %d, p_in: %d\n",p_out,p_in);
		if(cmd_header_flag == 0x00)//寻找帧头
		{
			if(uartRecvBuf[ptr_minus(p_out,2)] == WIFIAMP_APP_HEADER)
			{
				DBG(GREEN"FIND CMD HEAD\n"NONE);
				byte_cnt = uartRecvBuf[ptr_minus(p_out,1)] << 8;
				byte_cnt += uartRecvBuf[p_out];
				if((byte_cnt >= 4) && (byte_cnt <= BLE_CMD_MAX_LEN - 4))
				{
					cmd_len = byte_cnt;
          			cmd_header_flag = 0x01;
          			cmd_header_loc = ptr_minus(p_out,2);
          			//cmd_parsed_result = CMD_PARSED_UNFINISHED;
				}
			}
		}
		else
		{
			if(ptr_minus(p_out,cmd_header_loc) == (cmd_len + 3))//帧长度达�?
			{
				DBG(GREEN"CMD LEN OK\n"NONE);
				cmd_checksum = 0x00;
				for(i = 0; i < cmd_len; i++)//计算校验�?
				{
					cmd_checksum +=  uartRecvBuf[ptr_minus(p_out,(i + 1))];
				}
				if(cmd_checksum == uartRecvBuf[p_out])
				{
					DBG(GREEN"CMD CRC OK\n"NONE);
					//执行指令
					switch(uartRecvBuf[ptr_plus(cmd_header_loc,6)])
					{
						case CMD_MODULE_HAND_SHAKE:
						{
							int ret = 0;
							mk_ctrl = MK_SET_ADV_DATA;
							DBG(YELLOW"command: CMD_MODULE_HAND_SHAKE\n"NONE);
							ret = commuGetSystemConfig();
							DBG(YELLOW"commuGetSystemConfig(): %d\n"NONE,ret);
							if(ret == -1)//error
							{
								DBG(RED"command: CMD_MODULE_HAND_SHAKE commuGetSystemConfig() error\n"NONE);
							}
							if(ret == 0)// not config
							{
								// not configure, set led blink
								int ret = 0;
								// off all led
								ret = commuSetLedControl(LED2,0,1);
								ret = commuSetLedControl(LED3,0,1);
								ret = commuSetLedControl(LED4,0,1);
								// blink
								ret = commuSetLedControl(LED2,1,4);
								ret = commuSetLedControl(LED3,1,4);
								ret = commuSetLedControl(LED4,1,4);
								if(ret != 0)
								{
									DBG(RED"commuSetLedControl() error\n"NONE);
								}
							}
							else if(ret == 1)// configured
							{
								// configured, set led blink
								int ret = 0;
								// off all led
								ret = commuSetLedControl(LED2,0,1);
								ret = commuSetLedControl(LED3,0,1);
								ret = commuSetLedControl(LED4,0,1);
								// blink
								ret = commuSetLedControl(LED2,1,0);
								ret = commuSetLedControl(LED3,1,0);
								ret = commuSetLedControl(LED4,1,9);
								if(ret != 0)
								{
									DBG(RED"commuSetLedControl() error\n"NONE);
								}
							}
						}break;
						case CMD_SET_ADV_DATA:
						{
							mk_ctrl = MK_START_ADV;
							DBG(YELLOW"command: CMD_SET_ADV_DATA\n"NONE);
						}break;
						case CMD_ADV_CTRL:
						{
							mk_ctrl = MK_NOP;
							DBG(YELLOW"command: CMD_ADV_CTRL\n"NONE);
						}break;
						case CMD_CONN_STATE_NOTIFY:
						{
							unsigned char state = 0;
							state = uartRecvBuf[ptr_plus(cmd_header_loc,7)];
							DBG(YELLOW"BLE conn state: %d\n"NONE,state);
							/*if(state == 0)// 未连�?
							{
								get_list_flag = FALSE;
								g_isSpeedTesting = FALSE;
								swl_send_list_ctl = SWL_NOP_WAIT;
								swl_get_list_ctl = SWL_NOP_WAIT;
							}*/
						}break;
						case CMD_HAND_SHAKE:
						{
							DBG(YELLOW"command: CMD_HAND_SHAKE\n"NONE);
							unsigned int utime = 0;
							signed char tz = 0;
							char utime_str[11] = {0}, tz_str[5] = {0};
							
							for(i = 0; i < 4; i++)
							{
								utime += uartRecvBuf[ptr_plus(cmd_header_loc,i+7)];
								if(i == 3)break;
								utime <<=8;
							}
							tz = uartRecvBuf[ptr_plus(cmd_header_loc,4+7)];
							//hex2dstr(utime,utime_str);
							//itoa(utime,utime_str,10);
							sprintf(utime_str,"%d",utime);
							tz2str(tz,tz_str);
							commuSetTimeAndPhoneTZ(utime_str,tz_str);
							DBG(BLUE"unix time: %s, tz: %s\n"NONE,utime_str,tz_str);
							
							unsigned int len = 0;
							int ret = 0;
							len = sizeof(idps)/sizeof(unsigned char) + 1;
							unsigned char buf[100] = {0};
							for(i = 0; i < len -1; i++)
							{
								buf[i] = idps[i];
							}
							buf[len -1] = (unsigned char)commuGetWifiBand();
							ret = commuGetSystemConfig();
							DBG(YELLOW"commuGetSystemConfig(): %d\n"NONE,ret);
							if(ret == -1)//error
							{
								DBG(RED"command: CMD_MODULE_HAND_SHAKE commuGetSystemConfig() error\n"NONE);
							}
							if((ret == 0)&&(g_duringCfg == 1))
							{
								buf[len++] = 2;//during config
							}
							else
							{
								buf[len++] = (unsigned char)ret;
							}
							
							//set is or not upload set_wifi_state
							buf[len++] = (unsigned char)g_is_upload_set_wifi_state;

							//set net config mode: APP(1) or WPS(2)
							buf[len++] = 1;

							//set config wifi type
							buf[len++] = set_wifi_type;

							d.is_resp = 1;
							d.state = 0;
							d.seq_id = uartRecvBuf[ptr_plus(cmd_header_loc,4)];
							d.product_type = CMDTYPE_SET_WIFI;
							d.cmd_id = CMD_HAND_SHAKE;
							d.len = len;
							d.buff = buf;
							packDeviceCmd(&d);
						}break;
						case CMD_REQ_ROUTE_LIST:
						{
							DBG(YELLOW"command: CMD_REQ_ROUTE_LIST\n"NONE);
							d.is_resp = 1;
							d.state = 0;
							d.seq_id = uartRecvBuf[ptr_plus(cmd_header_loc,4)];
							d.product_type = CMDTYPE_SET_WIFI;
							d.cmd_id = CMD_REQ_ROUTE_LIST;
							d.len = 0;
							d.buff = (void *)0;
							packDeviceCmd(&d);
							
							/*T_MKOS_APCLI_STATUS_INFO g24status;
							T_MKOS_APCLI_STATUS_INFO g58status;
							memset(&g24status,0,sizeof(g24status));
							memset(&g58status,0,sizeof(g58status));

							commuGetConnectApStatus(&g24status,&g58status);

							if(g24status.is_enable || g58status.is_enable)// configured
							{
								commuStartScanAp();
								swl_get_list_ctl = SWL_FIRST_GET_ROUTE_LIST;
								//swl_send_list_ctl = SWL_SEND_ROUTE_LIST;
							}
							else*/
							{
								swl_get_list_ctl = SWL_GET_ROUTE_LIST_FROM_CATCH;
								pthread_mutex_lock(&mut);
								// select which cache list buffer to copy.
								if(cache_route_list_status2[0] == 1)
								{
									cache_copy_from = 0;
								}
								else if(cache_route_list_status2[1] == 1)
								{
									cache_copy_from = 1;
								}
								else if(cache_route_list_status2[0] == 2)
								{
									cache_copy_from = 0;
								}
								else if(cache_route_list_status2[1] == 2)
								{
									cache_copy_from = 1;
								}
								else
								{
									cache_copy_from = 0;
								}
								pthread_mutex_unlock(&mut);
							}
							
							get_list_flag = TRUE;
							has_send_route_cnt = route_cnt;
						}break;
						case CMD_SEND_ROUTE_LIST:
						{
							DBG(YELLOW"command: CMD_SEND_ROUTE_LIST\n"NONE);
							swl_send_list_ctl = SWL_SEND_ROUTE_LIST;
							if((g_resend_cmd.has_cmd == 1)&&(g_resend_cmd.cmd.cmd_id == CMD_SEND_ROUTE_LIST))
							{
								g_resend_cmd.has_cmd = 0;
								g_resend_cmd.is_cmd_changed = 0;
								g_resend_cmd.resend_cnt = 0;
								g_resend_cmd.resend_to = 0;
							}
						}break;
						case CMD_SET_WIFI:
						{
							DBG(YELLOW"command: CMD_SET_WIFI\n"NONE);
							unsigned char type = 0;
							//T_MKOS_APCLI_PARA wifi[2] = {0};
							T_MKOS_APCLI_PARA *p_2_4G = NULL, *p_5G = NULL;
							//T_MKOS_APCLI_PARA 5G = {0};
							
							unsigned char timeout = 0;
							char* p[4] = {0};
							unsigned int i = 0, j = 0, k = 0,m = 0,n = 0;

							g_duringCfg = 1;
							g_duringSetWifiLogic = 1;
							stopScan();
							// stop scan wifi
							//system("iwpriv ra0 get_site_survey");
							//system("iwpriv rai0 get_site_survey");
							//unsigned char buf[8192] = {0};
							//commuGetApSiteListSecond(buf);
							// stop all upper connection.
							//commuStopConnectAp(2);
							
							memset(&wifi[0],0,sizeof(wifi[0]));
							memset(&wifi[1],0,sizeof(wifi[1]));
							
							type = uartRecvBuf[ptr_plus(cmd_header_loc,7)];
							set_wifi_type = type;
							
							switch(type)
							{
								case 1:{m = 1; p_2_4G = &wifi[0]; p_5G = NULL;}break;
								case 2:{m = 1; p_2_4G = NULL;     p_5G = &wifi[0];}break;
								case 3:{m = 2; p_2_4G = &wifi[0]; p_5G = &wifi[1];}break;
								default: DBG(RED"CMD_SET_WIFI error!\n"NONE);
							}
							
							for(n = 0; n < m; n++)
							{
								p[0] = wifi[n].security;
								p[1] = wifi[n].ch;
								p[2] = wifi[n].ssid;
								p[3] = wifi[n].key;
								j = 0;
								for(k = 0; k < 4; k++)
								{
									while(uartRecvBuf[ptr_plus(cmd_header_loc,i+8)] != '\0')
									{
										p[k][j] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
										j++; i++;
									}
									i++; //j++;
									p[k][j] = '\0';
									j = 0;
									DBG("p[%d]: %s\n",k,p[k]);
								}
							}
							
							timeout = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
							wifi[0].timeout = timeout;
							wifi[1].timeout = timeout;
							conn_timeout = timeout/5;
							
							m = 0;
							DBG(BLUE"wifi[0]:\nsecurity: %s\nch: %s\nssid: %s\npassword: %s\ntimerout: %d\n"NONE,wifi[0].security,wifi[0].ch,wifi[0].ssid,wifi[0].key,wifi[0].timeout);
							DBG(BLUE"wifi[1]:\nsecurity: %s\nch: %s\nssid: %s\npassword: %s\ntimerout: %d\n"NONE,wifi[1].security,wifi[1].ch,wifi[1].ssid,wifi[1].key,wifi[1].timeout);

							//during config, set led blink
							int ret = 0;
							// off all led
							ret = commuSetLedControl(LED2,0,1);
							ret = commuSetLedControl(LED3,0,1);
							ret = commuSetLedControl(LED4,0,1);
							// blink
							ret = commuSetLedControl(LED2,1,1);
							ret = commuSetLedControl(LED3,1,1);
							ret = commuSetLedControl(LED4,1,1);
							if(ret != 0)
							{
								DBG(RED"commuSetLedControl() error\n"NONE);
							}

							d.is_resp = 1;
							d.state = 0;
							d.seq_id = uartRecvBuf[ptr_plus(cmd_header_loc,4)];
							d.product_type = CMDTYPE_SET_WIFI;
							d.cmd_id = CMD_SET_WIFI;
							d.len = 0;
							d.buff = (void*)0;
							packDeviceCmd(&d);
							time(&t_start_conn);
							// stop all upper connection.
							DBG(RED"Stop connect AP\n"NONE);
							commuStopConnectAp(2);
							DBG(RED"Start commuConnectAp\n"NONE);
							commuConnectAp(p_2_4G,p_5G);
							DBG(RED"Stop commuConnectAp\n"NONE);
							commuSaveConnectApInfo(p_2_4G,p_5G);
							DBG(RED"Stop commuSaveConnectApInfo\n"NONE);
							timer_cnt = 0;
							
							get_list_flag = TRUE;
							//swl_send_list_ctl = SWL_SEND_WIFI_CONN_STATE;
							swl_get_list_ctl = SWL_GET_WIFI_CONN_STATE; //SWL_NOP_WAIT; // 调试不调用查询API。

							g24ext_status = WIFI_EXT_NOT_START;
							g24extended = FALSE;
							g58ext_status = WIFI_EXT_NOT_START;
							g58extended = FALSE;
							
						}break;
						case CMD_SEND_WIFI_CONN_STATE:
						{
							DBG(YELLOW"command: CMD_SEND_WIFI_CONN_STATE\n"NONE);
							//get_list_flag = FALSE;
							//swl_send_list_ctl = SWL_NOP_WAIT;
							//swl_get_list_ctl = SWL_NOP_WAIT;
							g_is_upload_set_wifi_state = 1;
							if((g_resend_cmd.has_cmd == 1)&&(g_resend_cmd.cmd.cmd_id == CMD_SEND_WIFI_CONN_STATE))
							{
								g_resend_cmd.has_cmd = 0;
								g_resend_cmd.is_cmd_changed = 0;
								g_resend_cmd.resend_cnt = 0;
								g_resend_cmd.resend_to = 0;
							}
						}break;
						case CMD_CANCEL_SET_WIFI:
						{
							unsigned char buf[2] = {0};
							int len = 0;
							if(g_duringCfg == 1)
							{
								// stop all upper connection.
								commuStopConnectAp(2);
								buf[len++] = 0;//cancel success

								conn_to = 0;
								g_duringCfg = 0;
								g_duringSetWifiLogic = 0;
								get_list_flag = FALSE;
								//swl_send_list_ctl = SWL_SEND_WIFI_CONN_STATE;
								swl_get_list_ctl = SWL_NOP_WAIT;
							}
							else
							{
								buf[len++] = 1;//cancel faild
							}

							d.is_resp = 1;
							d.state = 0;
							d.seq_id = uartRecvBuf[ptr_plus(cmd_header_loc,4)];
							d.product_type = CMDTYPE_SET_WIFI;
							d.cmd_id = CMD_CANCEL_SET_WIFI;
							d.len = len;
							d.buff = buf;
							packDeviceCmd(&d);

						}break;
						case CMD_READ_CONFIGER_PARAM:
						{
							unsigned char param_id = 0;
							unsigned char param_group_num = 0;
							unsigned char read_group_cnt = 0;
							int param_len = 0;
							unsigned char buf[392] = {0};
							unsigned int len = 0, ret = 0;
							unsigned char *p = &buf[2];

							DBG(YELLOW"command: CMD_READ_CONFIGER_PARAM\n"NONE);
							d.is_resp = 1;
							d.state = 0;
							d.seq_id = uartRecvBuf[ptr_plus(cmd_header_loc,4)];
							d.product_type = CMDTYPE_WIFI_CONFIG_PARAM;
							d.cmd_id = CMD_READ_CONFIGER_PARAM;
							d.len = 0;
							d.buff = buf;
							buf[0] = 0;//success
							param_id       = uartRecvBuf[ptr_plus(cmd_header_loc,0+7)];// test type: LAN(0) or WAN(1)
							param_group_num = uartRecvBuf[ptr_plus(cmd_header_loc,1+7)];// test band: 2.4G(1) or 5G(2) or dual band(0)
							read_group_cnt = uartRecvBuf[ptr_plus(cmd_header_loc,2+7)];// read group count
							buf[1] = param_id;
							
							switch(param_id)
							{
								case ID_DEV_INFO:
								{
									DBG(YELLOW"param_id: ID_DEV_INFO\n"NONE);
									len = sizeof(idps)/sizeof(unsigned char);
									for(i = 0; i < len; i++)
									{
										p[i] = idps[i];
									}
									d.len = len + 2;
								}break;
								case ID_UPPER_NETWORK_INFO:
								{
									DBG(YELLOW"param_id: ID_UPPER_NETWORK_INFO\n"NONE);
									T_MKOS_APCLI_STATUS_INFO g24status;
									T_MKOS_APCLI_STATUS_INFO g58status;
									T_MKOS_APCLI_PARA g24info,g58info;
									int recommand_band = 0;
									memset(&g24status,0,sizeof(g24status));
									memset(&g58status,0,sizeof(g58status));
									memset(&g24info,0,sizeof(g24info));
									memset(&g58info,0,sizeof(g58info));

									recommand_band = commuGetAplientInterfaceToUse();
									commuGetConnectApStatus(&g24status,&g58status);
									ret = commuGetConnectApInfo(&g24info,&g58info);
									if(ret == -1)// get AP info failed
									{
										buf[0] = 1;//failed
										d.len = 2;
										break;
									}
									switch(param_group_num)
									{
										case 1:
										{
											DBG(YELLOW"param_group_num: 1\n"NONE);
											p[len++] = 1;// group id
											if(g24status.is_enable)
											{
												p[len++] = 1;//已配置
											}
											else
											{
												p[len++] = 0;//未配置
											}
											if(recommand_band == 0)// means used 2.4G 
											{
												p[len++] = 1;// have already connected to net
											}
											else
											{
												p[len++] = 0;// not connect to net
											}
											p[len++] = (unsigned char)(g24status.rate >>24);// set real time connection speed
											p[len++] = (unsigned char)(g24status.rate >>16);
											p[len++] = (unsigned char)(g24status.rate >>8);
											p[len++] = (unsigned char)(g24status.rate);
											p[len++] = (g24status.rssi > -127) ?(char)g24status.rssi : -127;// set rssi
											p[len++] = (unsigned char)str2int(g24info.ch);// set channel
											// strcpy() include '\0', strlen() exclude '\0'
											strcpy(&p[len],g24info.security);// set security
											len += strlen(g24info.security);
											len++;
											strcpy(&p[len],g24info.ssid);// set ssid
											len += strlen(g24info.ssid);
											len++;
											strcpy(&p[len],g24info.key);// set password
											len += strlen(g24info.key);
											len++;// add the '\0'
											DBG(YELLOW"g24status.is_enable: %d\n"NONE,g24status.is_enable);
											DBG(YELLOW"g24status.status: %d\n"NONE,g24status.status);
											DBG(YELLOW"g24status.rate: %d\n"NONE,g24status.rate);
											DBG(YELLOW"g24status.rssi: %d\n"NONE,g24status.rssi);
											DBG(YELLOW"g24info.ssid: %s\n"NONE,g24info.ssid);
											DBG(YELLOW"g24info.security: %s\n"NONE,g24info.security);
											DBG(YELLOW"g24info.key: %s\n"NONE,g24info.key);
											DBG(YELLOW"g24info.ch: %s\n"NONE,g24info.ch);
											
											DBG(YELLOW"param_group_num: 1 PACK OK \n"NONE);

											if(read_group_cnt <= 1)
											{
												break;
											}
										}
										case 2:
										{
											DBG(YELLOW"param_group_num: 2\n"NONE);
											p[len++] = 2;// group id
											if(g58status.is_enable)
											{
												p[len++] = 1;//已配置
											}
											else
											{
												p[len++] = 0;//未配置
											}
											if(recommand_band == 1)// means used 5G 
											{
												p[len++] = 1;// have already connected to net
											}
											else
											{
												p[len++] = 0;// not connect to net
											}
											p[len++] = (unsigned char)(g58status.rate >>24);// set real time connection speed
											p[len++] = (unsigned char)(g58status.rate >>16);
											p[len++] = (unsigned char)(g58status.rate >>8);
											p[len++] = (unsigned char)(g58status.rate);
											p[len++] = (g58status.rssi > -127) ?(char)g58status.rssi : -127;// set rssi
											p[len++] = (unsigned char)str2int(g58info.ch);// set channel
											
											strcpy(&p[len],g58info.security);// set security
											len += strlen(g58info.security);
											len++;
											DBG(YELLOW"g58status.is_enable: %d\n"NONE,g58status.is_enable);
											DBG(YELLOW"g58status.status: %d\n"NONE,g58status.status);
											DBG(YELLOW"g58status.rate: %d\n"NONE,g58status.rate);
											DBG(YELLOW"g58status.rssi: %d\n"NONE,g58status.rssi);
											DBG(YELLOW"g58info.ssid: %s\n"NONE,g58info.ssid);
											DBG(YELLOW"g58info.security: %s\n"NONE,g58info.security);
											DBG(YELLOW"g58info.key: %s\n"NONE,g58info.key);
											DBG(YELLOW"g58info.ch: %s\n"NONE,g58info.ch);
											strcpy(&p[len],g58info.ssid);// set ssid
											len += strlen(g58info.ssid);
											len++;
											strcpy(&p[len],g58info.key);// set password
											len += strlen(g58info.key);
											len++;// add the '\0'
											DBG(YELLOW"param_group_num: 2 PACK OK \n"NONE);
										}break;
										default:
										{
											DBG(RED"ID_UPPER_NETWORK_INFO error!!\n"NONE);
										}
									}
									d.len = len + 2;
									DBG(YELLOW"d.len = %d\n"NONE,d.len);
								}break;
								case ID_EXTEND_NETWORK_INFO:
								{
									T_MKOS_EXT_AP_INFO ext_ap_info;
									DBG(YELLOW"param_id: ID_EXTEND_NETWORK_INFO\n"NONE);
									switch(param_group_num)
									{
										case 1:// get 2.4G ext ap info
										{
											p[len++] = 1;// group id
											memset(&ext_ap_info,0,sizeof(ext_ap_info));
											ret = commuGetExtAp(0,ext_ap_info.enable,ext_ap_info.hidden,ext_ap_info.ssid,ext_ap_info.security,ext_ap_info.pass,ext_ap_info.ch);
											if(ret != 0)// API return failed
											{
												buf[0] = 1;//failed
												d.len = 2;
												break;
											}
											p[len++] = ext_ap_info.enable[0] == '0'?0:1;// 0: not extend, 1: extend, default: 1
											p[len++] = ext_ap_info.hidden[0] == '1'?0:1;// 0: not adv ssid, 1: adv ssid, default: 1
											p[len++] = str2int(ext_ap_info.ch);// set channel
											strcpy(&p[len],ext_ap_info.security);// set security
											len += strlen(ext_ap_info.security);
											len++;
											strcpy(&p[len],ext_ap_info.ssid);// set ssid
											len += strlen(ext_ap_info.ssid);
											len++;
											strcpy(&p[len],ext_ap_info.pass);// set pass
											len += strlen(ext_ap_info.pass);
											len++;// add the '\0'
											
											if(read_group_cnt <= 1)
											{
												break;
											}
										}
										case 2:// get 5G ext ap info
										{
											p[len++] = 2;// group id
											memset(&ext_ap_info,0,sizeof(ext_ap_info));
											ret = commuGetExtAp(1,ext_ap_info.enable,ext_ap_info.hidden,ext_ap_info.ssid,ext_ap_info.security,ext_ap_info.pass,ext_ap_info.ch);
											if(ret != 0)// API return failed
											{
												buf[0] = 1;//failed
												d.len = 2;
												break;
											}
											p[len++] = ext_ap_info.enable[0] == '0'?0:1;// 0: not extend, 1: extend, default: 1
											p[len++] = ext_ap_info.hidden[0] == '1'?0:1;// 0: not adv ssid, 1: adv ssid, default: 1
											p[len++] = str2int(ext_ap_info.ch);// set channel
											strcpy(&p[len],ext_ap_info.security);// set security
											len += strlen(ext_ap_info.security);
											len++;
											strcpy(&p[len],ext_ap_info.ssid);// set ssid
											len += strlen(ext_ap_info.ssid);
											len++;
											strcpy(&p[len],ext_ap_info.pass);// set pass
											len += strlen(ext_ap_info.pass);
											len++;// add the '\0'
											
										}break;
										default:
										{
											DBG(RED"ID_UPPER_NETWORK_INFO error!!\n"NONE);
										}
									}
									d.len = len + 2;
								}break;
								case ID_DEV_LIST_CONN_NOW:
								{
									DBG(YELLOW"param_id: ID_DEV_LIST_CONN_NOW\n"NONE);
									static T_MKOS_DEV_LIST_NOW dev_list_now[32];
									static int dev_cnt = 0;
									switch(param_group_num)
									{
										case 1:
										{
											DBG(YELLOW"param_group_num: 1\n"NONE);
											char list_buf[2048] = {0};
											char list_mrn_buf[2048] = {0};
											DBG(YELLOW"commuGetCurrentWifiStaListBuf before \n"NONE);
											ret = commuGetCurrentWifiStaListBuf(list_buf);
											DBG(YELLOW"commuGetCurrentWifiStaListBuf after \n"NONE);
											DBG(YELLOW"%s\n"NONE,list_buf);
											if(ret != 0)//API return failed
											{
												buf[0] = 1;//failed
												d.len = 2;
												break;
											}
											ret = commuMacNameInfoList(list_mrn_buf);
											DBG(YELLOW"%s\n"NONE,list_mrn_buf);
											if(ret != 0)//API return failed
											{
												buf[0] = 1;//failed
												d.len = 2;
												break;
											}
											dev_cnt = 0;
											memset(dev_list_now,0,sizeof(dev_list_now));
											
											//dbg dummy data
											//memset(list_buf,0,sizeof(list_buf));
											//char dummy_data[2048] = {"00:E0:4C:11:22:31 2G 300 40M -37 260 1-this-is-the-name-session-can-not-have-space 1-this-is-the-remark-name-session-can-not-have-space\n01:44:4C:33:22:55 5G 350 22M -38 260 2-this-is-the-name-session-can-not-have-space 2-this-is-the-remark-name-session-can-not-have-space\n77:88:4C:11:44:33 2G 300 40M -78 400 3-this-is-the-name-session-can-not-have-space 3-this-is-the-remark-name-session-can-not-have-space\nAF:E0:4C:A1:A2:A3 5G 100 20M -45 500 4-this-is-the-name-session-can-not-have-space 4-this-is-the-remark-name-session-can-not-have-space\nB0:E0:C0:D0:E0:F0 2G 156 40M -120 777 5-this-is-the-name-session-can-not-have-space 5-this-is-the-remark-name-session-can-not-have-space\n\0"};
											//strcpy(list_buf,dummy_data);
											
											DBG(YELLOW"praseDevListNow before \n"NONE);
											praseDevListNow(list_buf,dev_list_now,&dev_cnt);
											praseDevListNowRemarkName(list_mrn_buf,dev_list_now);
											DBG(YELLOW"praseDevListNow after \n"NONE);
											DBG(YELLOW"param_group_num: 1 PACK OK \n"NONE);
										}
										default:
										{
											if(param_group_num > dev_cnt)// no data
											{
												buf[0] = 2;//no data
												d.len = 2;
												break;
											}
											else
											{
												int i = 0, m = 0;
												int group_id = 0;
												while(i < read_group_cnt)
												{
													group_id = (param_group_num - 1) + i;// group_id use as array index, so need -1
													if((len + dev_list_now[group_id].dev_len) > 390)
													{
														break;
													}
													p[len++] = group_id + 1;// start at 1, so need add 1
													if((group_id +1) < dev_cnt)// still have data
													{
														p[len++] = 1;//still have data
													}
													else
													{
														p[len++] = 0;// have no data
													}
													//set IP
													{
														for(m = 0; m < 4; m++)
														{
															p[len++] = dev_list_now[group_id].ip[m];
														}
													}
													//set mac
													{
														for(m = 0; m < 6; m++)
														{
															p[len++] = dev_list_now[group_id].mac[m];
														}
													}
													//set rssi
													{
														p[len++] = ((dev_list_now[group_id].rssi > -127) ? ((char)(dev_list_now[group_id].rssi)) : -127);
													}
													//set real time speed
													{
														p[len++] = (char)(dev_list_now[group_id].speed_tx >> 24);
														p[len++] = (char)(dev_list_now[group_id].speed_tx >> 16);
														p[len++] = (char)(dev_list_now[group_id].speed_tx >> 8);
														p[len++] = (char)(dev_list_now[group_id].speed_tx);
													}
													//set name
													{
														//p[len++] = '\0';
														int n = 0;
														n = dev_list_now[group_id].name_len;
														for(m = 0; m < n; m++)
														{
															p[len++] = dev_list_now[group_id].name[m];
														}
													}
													//set remark_name
													{
														int n = 0;
														n = dev_list_now[group_id].remark_name_len;
														for(m = 0; m < n; m++)
														{
															p[len++] = dev_list_now[group_id].remark_name[m];
														}
													}
													if((group_id +1) >= dev_cnt)// has no data, so break while()
													{
														break;
													}
													i++;
												}
											}
										}
									}
									d.len = len + 2;
									
								}break;
								case ID_SET_MAC_FILTER:
								{
									DBG(YELLOW"GET ID_SET_MAC_FILTER \n"NONE);
									ret = commuGetWifiAclPolicy();
									if(ret == -1)// get AP info failed
									{
										buf[0] = 1;//failed
										d.len = 2;
										break;
									}
									p[len++] = (unsigned char)ret;
									d.len = len + 2;
								}break;
								case ID_GET_MAC_FILTER_WRITE_LIST:
								{
									DBG(YELLOW"GET ID_GET_MAC_FILTER_WRITE_LIST \n"NONE);
									static T_MKOS_WRITE_LIST write_list[200];
									static int write_list_cnt = 0;
									switch(param_group_num)
									{
										case 1:
										{
											char write_list_buf[2048] = {0};
											ret = commuGetWifiAclWhiteList(write_list_buf);
											if(ret != 0)//API return failed
											{
												buf[0] = 1;//failed
												d.len = 2;
												break;
											}
											DBG(YELLOW"write_list_buf: %s\n"NONE,write_list_buf);
											write_list_cnt = 0;
											memset(write_list,0,sizeof(write_list));
											praseWriteList(write_list_buf,write_list,&write_list_cnt);
										}
										default:
										{
											if(param_group_num > write_list_cnt)// no data
											{
												buf[0] = 2;//no data
												d.len = 2;
												break;
											}
											else
											{
												int i = 0, m = 0;
												int group_id = 0;
												while(i < read_group_cnt)
												{
													group_id = (param_group_num - 1) + i;
													if((len + 6) > 390)
													{
														break;
													}
													p[len++] = group_id + 1;
													if((group_id + 1) < write_list_cnt)// still have data
													{
														p[len++] = 1;//still have data
													}
													else
													{
														p[len++] = 0;// have no data
													}
													//set mac
													{
														for(m = 0; m < 6; m++)
														{
															p[len++] = write_list[group_id].mac[m];
														}
													}
													if((group_id +1) >= write_list_cnt)// has no data, so break while()
													{
														break;
													}
													i++;
												}
											}
										}
									}
									d.len = len + 2;
									
								}break;
								case ID_GET_MAC_FILTER_BLACK_LIST:
								{
									DBG(YELLOW"GET ID_GET_MAC_FILTER_BLACK_LIST \n"NONE);
									static T_MKOS_BLACK_LIST black_list[200];
									static int black_list_cnt = 0;
									switch(param_group_num)
									{
										case 1:
										{
											char black_list_buf[2048] = {0};
											ret = commuGetWifiAclBlankList(black_list_buf);
											if(ret != 0)//API return failed
											{
												buf[0] = 1;//failed
												d.len = 2;
												break;
											}
											black_list_cnt = 0;
											memset(black_list,0,sizeof(black_list));
											praseBlackList(black_list_buf,black_list,&black_list_cnt);
										}
										default:
										{
											if(param_group_num > black_list_cnt)// no data
											{
												buf[0] = 2;//no data
												d.len = 2;
												break;
											}
											else
											{
												int i = 0, m = 0;
												int group_id = 0;
												while(i < read_group_cnt)
												{
													group_id = (param_group_num - 1) + i;
													if((len + 6) > 390)
													{
														break;
													}
													p[len++] = group_id + 1;
													if((group_id + 1) < black_list_cnt)// still have data
													{
														p[len++] = 1;//still have data
													}
													else
													{
														p[len++] = 0;// have no data
													}
													//set mac
													{
														for(m = 0; m < 6; m++)
														{
															p[len++] = black_list[group_id].mac[m];
														}
													}
													if((group_id +1) >= black_list_cnt)// has no data, so break while()
													{
														break;
													}
													i++;
												}
											}
										}
									}
									d.len = len + 2;
								}break;
								case ID_CTRL_LED:
								{
									DBG(YELLOW"GET ID_CTRL_LED \n"NONE);
									ret = commuGetALLledControl();
									if(ret == -1)
									{
										buf[0] = 1;//failed
										d.len = 2;
										break;
									}
									else
									{
										p[len++] = (unsigned char)ret;
										d.len = len + 2;
									}
								}break;
								default:
								{
									DBG(RED"ID error !"NONE);
								}
							}

							packDeviceCmd(&d);
						}break;
						case CMD_WRITE_CONFIGER_PARAM:
						{
							unsigned char param_id = 0;
							unsigned char param_group_num = 0;
							int param_len = 0;
							unsigned char buf[5] = {0};
							unsigned int len = 0, ret = 0;
							d.is_resp = 1;
							d.state = 0;
							d.seq_id = uartRecvBuf[ptr_plus(cmd_header_loc,4)];
							d.product_type = CMDTYPE_WIFI_CONFIG_PARAM;
							d.cmd_id = CMD_WRITE_CONFIGER_PARAM;
							d.len = 0;
							d.buff = buf;
							buf[0] = 0;//success
							param_id = uartRecvBuf[ptr_plus(cmd_header_loc,0+7)];// test type: LAN(0) or WAN(1)
							buf[1] = param_id;
							

							switch(param_id)
							{
								case ID_EXTEND_NETWORK_INFO:
								{
									T_MKOS_EXT_AP_INFO ext_ap_info;
									T_MKOS_EXT_AP_INFO_SET_FROM_APP ext_ap;
									unsigned int i = 0, k = 0, data_len = 0;
									data_len = cmd_len -5;
									while(i < data_len)
									{
										memset(&ext_ap,0,sizeof(ext_ap));
										ext_ap.id = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
										i++;
										ext_ap.net_status = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
										i++;
										ext_ap.ssid_broadcast_status = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
										i++;
										ext_ap.ch = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
										i++;
										// set security
										k = 0;
										while(uartRecvBuf[ptr_plus(cmd_header_loc,i+8)] != '\0')
										{
											ext_ap.security[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
											k++;
											i++;
										}
										ext_ap.security[k] = '\0';
										i++;
										// set ssid
										k = 0;
										while(uartRecvBuf[ptr_plus(cmd_header_loc,i+8)] != '\0')
										{
											ext_ap.ssid[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
											k++;
											i++;
										}
										ext_ap.ssid[k] = '\0';
										i++;
										// set pass
										k = 0;
										while(uartRecvBuf[ptr_plus(cmd_header_loc,i+8)] != '\0')
										{
											ext_ap.pass[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
											k++;
											i++;
										}
										ext_ap.pass[k] = '\0';
										i++;
										DBG(YELLOW"ext_ap.id: %d\n"NONE,ext_ap.id);
										DBG(YELLOW"ext_ap.net_status: %d\n"NONE,ext_ap.net_status);
										DBG(YELLOW"ext_ap.ssid_broadcast_status: %d\n"NONE,ext_ap.ssid_broadcast_status);
										DBG(YELLOW"ext_ap.ch: %d\n"NONE,ext_ap.ch);
										DBG(YELLOW"ext_ap.security: %s\n"NONE,ext_ap.security);
										DBG(YELLOW"ext_ap.ssid: %s\n"NONE,ext_ap.ssid);
										DBG(YELLOW"ext_ap.pass: %s\n"NONE,ext_ap.pass);

										memset(&ext_ap_info,0,sizeof(ext_ap_info));
										ext_ap_info.enable[0] = ext_ap.net_status == 1?'1':'0';
										ext_ap_info.hidden[0] = ext_ap.ssid_broadcast_status == 1?'0':'1';
										strcpy(ext_ap_info.security,ext_ap.security);
										strcpy(ext_ap_info.ssid,ext_ap.ssid);
										strcpy(ext_ap_info.pass,ext_ap.pass);
										sprintf(ext_ap_info.ch,"%d",ext_ap.ch);

										switch(ext_ap.id)
										{
											case 1:
											{
												// call the set 2.4G ext AP API
												DBG(YELLOW"SET ID_EXTEND_NETWORK_INFO 1! "NONE);
												ret = commuSetExtAp(0,ext_ap_info.enable,ext_ap_info.hidden,ext_ap_info.ssid,ext_ap_info.security,ext_ap_info.pass,ext_ap_info.ch);
												/*if(ret == 0)
												{
													commuSaveExtAp(0,ext_ap_info.enable,ext_ap_info.hidden,ext_ap_info.ssid,ext_ap_info.security,ext_ap_info.pass,ext_ap_info.ch);
												}*/
											}break;
											case 2:
											{
												// call the set 5G ext AP API
												DBG(YELLOW"SET ID_EXTEND_NETWORK_INFO 2! "NONE);
												ret = commuSetExtAp(1,ext_ap_info.enable,ext_ap_info.hidden,ext_ap_info.ssid,ext_ap_info.security,ext_ap_info.pass,ext_ap_info.ch);
												/*if(ret == 0)
												{
													commuSaveExtAp(1,ext_ap_info.enable,ext_ap_info.hidden,ext_ap_info.ssid,ext_ap_info.security,ext_ap_info.pass,ext_ap_info.ch);
												}*/
											}break;
											default:
											{
												DBG(RED"SET ID_EXTEND_NETWORK_INFO error! "NONE);
											}
										}
										if(ret != 0)//API return failed
										{
											buf[0] = 1;//failed
											d.len = 2;
											break;
										}
									}
									d.len = len +2;
								}break;
								case ID_DEV_LIST_CONN_NOW:
								{
									DBG(YELLOW"SET ID_DEV_LIST_CONN_NOW \n"NONE);
									T_MKOS_DEV_LIST_NOW_FROM_APP dev_conn;
									unsigned int i = 0, k = 0, data_len = 0;
									data_len = cmd_len -5;
									while(i < data_len)
									{
										memset(&dev_conn,0,sizeof(dev_conn));
										dev_conn.id = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
										i++;
										dev_conn.more = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
										i++;
										for(k = 0; k < 4; k++)
										{
											dev_conn.ip[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
											i++;
										}
										for(k = 0; k < 6; k++)
										{
											dev_conn.mac[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
											i++;
										}
										dev_conn.rssi = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
										i++;
										for(k = 0; k < 4; k++)
										{
											dev_conn.speed_tx = (dev_conn.speed_tx <<8) + uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
											i++;
										}
										// set name
										k = 0;
										while(uartRecvBuf[ptr_plus(cmd_header_loc,i+8)] != '\0')
										{
											dev_conn.name[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
											k++;
											i++;
										}
										dev_conn.name[k] = '\0';
										dev_conn.name_len = k + 1;
										i++;
										// set remark name
										k = 0;
										while(uartRecvBuf[ptr_plus(cmd_header_loc,i+8)] != '\0')
										{
											dev_conn.remark_name[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
											k++;
											i++;
										}
										dev_conn.remark_name[k] = '\0';
										dev_conn.remark_name_len = k + 1;
										i++;
										
										// call the set connected dev remark name API
										char mac_str_colon[20] = {0};
										char hex[] = "0123456789ABCDEF";
										int j = 0;
										for(j = 0,k = 0; k < 6; k++,j+=3)
										{
											mac_str_colon[j] = hex[dev_conn.mac[k]>>4];
											mac_str_colon[j+1] = hex[dev_conn.mac[k]&0x0F];
											mac_str_colon[j+2] = ':';
										}
										mac_str_colon[17] = '\0';

										ret = commuAddMacNameEntry(mac_str_colon,dev_conn.remark_name);
										if(ret != 0)//API return failed
										{
											buf[0] = 1;//failed
											d.len = 2;
											break;
										}
										
										DBG(YELLOW"set %02X%02X%02X%02X%02X%02X remark name: %s\n"NONE,dev_conn.mac[0],dev_conn.mac[1],dev_conn.mac[2],dev_conn.mac[3],dev_conn.mac[4],dev_conn.mac[5],dev_conn.remark_name);
									}
									d.len = len +2;
								}break;
								case ID_SET_MAC_FILTER:
								{
									DBG(YELLOW"SET ID_SET_MAC_FILTER \n"NONE);
									unsigned int i = 0, k = 0, data_len = 0;
									int mode = 0, ctrl = 0;
									
									data_len = cmd_len -5;
									mode = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
									i++;
									ctrl = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
									i++;
									switch(mode)
									{
										case 0:// close filter
										{
											ret = commuSetWifiAclPolicy(mode);
											if(ret != 0)//API return failed
											{
												buf[0] = 1;//failed
												d.len = 2;
												break;
											}
										}break;
										case 1:// write list mode
										{
											switch(ctrl)
											{
												case 0: // open
												{
													DBG(YELLOW"write list mode: open\n"NONE);
													ret = commuSetWifiAclPolicy(mode);
													if(ret != 0)//API return failed
													{
														buf[0] = 1;//failed
														d.len = 2;
														break;
													}
												}break;
												case 1: // add
												{
													DBG(YELLOW"write list mode: add\n"NONE);
													unsigned char mac[6] = {0};
													char mac_str_colon[20] = {0};
													char hex[] = "0123456789ABCDEF";
													int j = 0;
													while(i < data_len)
													{
														for(k = 0; k < 6; k++)
														{
															mac[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
															i++;
														}
														DBG(YELLOW"mac hex: %02X%02X%02X%02X%02X%02X\n"NONE,mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
														for(j = 0,k = 0; k < 6; k++,j+=3)
														{
															mac_str_colon[j] = hex[mac[k]>>4];
															mac_str_colon[j+1] = hex[mac[k]&0x0F];
															mac_str_colon[j+2] = ':';
														}
														mac_str_colon[17] = '\0';

														DBG(YELLOW"mac: %s\n"NONE,mac_str_colon);

														ret = commuAddWifiAclWhiteEntry(mac_str_colon);
														if(ret != 0)//API return failed
														{
															buf[0] = 1;//failed
															d.len = 2;
															break;
														}
													}
												}break;
												case 2: // delete
												{
													DBG(YELLOW"write list mode: delete\n"NONE);
													unsigned char mac[6] = {0};
													char mac_str_colon[20] = {0};
													char hex[] = "0123456789ABCDEF";
													int j = 0;
													while(i < data_len)
													{
														for(k = 0; k < 6; k++)
														{
															mac[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
															i++;
														}
														
														for(j = 0,k = 0; k < 6; k++,j+=3)
														{
															mac_str_colon[j] = hex[mac[k]>>4];
															mac_str_colon[j+1] = hex[mac[k]&0x0F];
															mac_str_colon[j+2] = ':';
														}
														mac_str_colon[17] = '\0';

														ret = commuDelWifiAclWhiteEntry(mac_str_colon);
														if(ret != 0)//API return failed
														{
															buf[0] = 1;//failed
															d.len = 2;
															break;
														}
													}
												}break;
												case 3: // delete all
												{
													DBG(YELLOW"write list mode: delete all\n"NONE);
													ret = commuDelAllWifiAclWhiteEntry();
													if(ret != 0)//API return failed
													{
														buf[0] = 1;//failed
														d.len = 2;
														break;
													}
												}break;
												default:
												{
													DBG(RED" set write list operation error\n"NONE);
												}
											}
											d.len = len +2;
										}break;
										case 2:// black list mode
										{
											switch(ctrl)
											{
												case 0: // open
												{
													DBG(YELLOW"black list mode: open\n"NONE);
													ret = commuSetWifiAclPolicy(mode);
													if(ret != 0)//API return failed
													{
														buf[0] = 1;//failed
														d.len = 2;
														break;
													}
												}break;
												case 1: // add
												{
													DBG(YELLOW"black list mode: add\n"NONE);
													unsigned char mac[6] = {0};
													char mac_str_colon[20] = {0};
													char hex[] = "0123456789ABCDEF";
													int j = 0;
													while(i < data_len)
													{
														for(k = 0; k < 6; k++)
														{
															mac[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
															i++;
														}
														
														for(j = 0,k = 0; k < 6; k++,j+=3)
														{
															mac_str_colon[j] = hex[mac[k]>>4];
															mac_str_colon[j+1] = hex[mac[k]&0x0F];
															mac_str_colon[j+2] = ':';
														}
														mac_str_colon[17] = '\0';

														ret = commuAddWifiAclBlankEntry(mac_str_colon);
														if(ret != 0)//API return failed
														{
															buf[0] = 1;//failed
															d.len = 2;
															break;
														}
													}
												}break;
												case 2: // delete
												{
													DBG(YELLOW"black list mode: delete\n"NONE);
													unsigned char mac[6] = {0};
													char mac_str_colon[20] = {0};
													char hex[] = "0123456789ABCDEF";
													int j = 0;
													while(i < data_len)
													{
														for(k = 0; k < 6; k++)
														{
															mac[k] = uartRecvBuf[ptr_plus(cmd_header_loc,i+8)];
															i++;
														}
														
														for(j = 0,k = 0; k < 6; k++,j+=3)
														{
															mac_str_colon[j] = hex[mac[k]>>4];
															mac_str_colon[j+1] = hex[mac[k]&0x0F];
															mac_str_colon[j+2] = ':';
														}
														mac_str_colon[17] = '\0';

														ret = commuDelWifiAclBlankEntry(mac_str_colon);
														if(ret != 0)//API return failed
														{
															buf[0] = 1;//failed
															d.len = 2;
															break;
														}
													}
												}break;
												case 3: // delete all
												{
													DBG(YELLOW"black list mode: delete all\n"NONE);
													ret = commuDelAllWifiAclBlankEntry();
													if(ret != 0)//API return failed
													{
														buf[0] = 1;//failed
														d.len = 2;
														break;
													}
												}break;
												default:
												{
													DBG(RED" set black list operation error\n"NONE);
												}
											}
										}break;
										default:
										{
											DBG(RED"set ID_SET_MAC_FILTER mode error\n"NONE);
										}
									}
									d.len = len + 2;
									
								}break;
								case ID_CTRL_LED:
								{
									DBG(YELLOW"SET ID_CTRL_LED \n"NONE);
									int type = 0;
									type = uartRecvBuf[ptr_plus(cmd_header_loc,0+8)];
									ret = commuSetALLledControl(type);
									if(ret != 0)//API return failed
									{
										buf[0] = 1;//failed
										d.len = 2;
										break;
									}
									d.len = len + 2;
								}break;
								default:
								{
									DBG(RED"ID error !"NONE);
								}
							}
							packDeviceCmd(&d);
						}break;
						case CMD_START_SPEED_TEST:
						{
							unsigned char type_in = 0,set_test_band = 0;
							unsigned char status, type_out, real_test_band;
							signed char rssi_2_4G, rssi_5G;
							
							DBG(YELLOW"command: CMD_START_SPEED_TEST\n"NONE);

							unsigned char buf[5] = {0};
							d.is_resp = 1;
							d.state = 0;
							d.seq_id = uartRecvBuf[ptr_plus(cmd_header_loc,4)];
							d.product_type = CMDTYPE_WIFI_CONFIG_PARAM;
							d.cmd_id = CMD_START_SPEED_TEST;
							d.len = sizeof(buf)/sizeof(unsigned char);
							d.buff = buf;
							buf[0] = 0;//success
							type_in = uartRecvBuf[ptr_plus(cmd_header_loc,0+7)];// test type: LAN(0) or WAN(1)
							set_test_band = uartRecvBuf[ptr_plus(cmd_header_loc,1+7)];// test band: 2.4G(1) or 5G(2) or dual band(0)
							buf[1] = type_in;
							buf[2] = set_test_band;
							buf[3] = 100;// 2.4G rssi
							buf[4] = 100;// 5G rssi
							
							T_MKOS_APCLI_STATUS_INFO g24status;
							T_MKOS_APCLI_STATUS_INFO g58status;
							memset(&g24status,0,sizeof(g24status));
							memset(&g58status,0,sizeof(g58status));

							commuGetConnectApStatus(&g24status,&g58status);
							
							
							DBG(YELLOW"g24status.is_enable: %d\n"NONE,g24status.is_enable);
							DBG(YELLOW"g24status.status: %d\n"NONE,g24status.status);
							DBG(YELLOW"g24status.ip: %s\n"NONE,g24status.ip);
							DBG(YELLOW"g24status.rate: %d\n"NONE,g24status.rate);
							DBG(YELLOW"g24status.rssi: %d\n"NONE,g24status.rssi);
							DBG(YELLOW"g58status.is_enable: %d\n"NONE,g58status.is_enable);
							DBG(YELLOW"g58status.status: %d\n"NONE,g58status.status);
							DBG(YELLOW"g58status.ip: %s\n"NONE,g58status.ip);
							DBG(YELLOW"g58status.rate: %d\n"NONE,g58status.rate);
							DBG(YELLOW"g58status.rssi: %d\n"NONE,g58status.rssi);

							//if(g24status.is_enable && g24status.status == WIFI_CONN_SUCCESS)
							if(g24status.is_enable)
							{
								buf[3] = g24status.rssi;
							}
							//if(g58status.is_enable && g58status.status == WIFI_CONN_SUCCESS)
							if(g58status.is_enable)
							{
								buf[4] = g58status.rssi;
							}

							if(buf[3] == 100 && buf[4] == 100)
							{
								buf[0] = 1;//faild: not connect to net.
							}

							if(g_isSpeedTesting == TRUE)
							{
								buf[0] = 2;//busy: wifi extender is testing.
							}

							/*if((set_test_band == 0)&&(buf[3] != 100 && buf[4] != 100))
							{
								real_test_band = 0;// dual band speed test
							}
							else if((set_test_band == 0)&&(buf[3] != 100))
							{
								real_test_band = 1;// 2.4G speed test
							}
							else if((set_test_band == 0)&&(buf[4] != 100))
							{
								real_test_band = 2;// 5G speed test
							}
							else if((set_test_band == 1)&&(buf[3] != 100))
							{
								real_test_band = 1;// 2.4G speed test
							}
							else if((set_test_band == 1)&&(buf[4] != 100))
							{
								real_test_band = 2;// 5G speed test
							}
							else if((set_test_band == 2)&&(buf[4] != 100))
							{
								real_test_band = 2;// 5G speed test
							}
							else if((set_test_band == 2)&&(buf[3] != 100))
							{
								real_test_band = 1;// 2.4G speed test
							}*/
							real_test_band = 1 + commuGetAplientInterfaceToUse();
							buf[2] = real_test_band;
							DBG(YELLOW"real_test_band: %d\n"NONE,real_test_band);
							packDeviceCmd(&d);
							
							if(g_isSpeedTesting == FALSE)
							{
								if(type_in == 0)// means LAN speed test
								{
									if(real_test_band == 0)
									{
										swl_get_list_ctl = SWL_SPEED_TEST_LAN_SINGLE;//SWL_SPEED_TEST_LAN_DUAL;
									}
									else
									{
										swl_get_list_ctl = SWL_SPEED_TEST_LAN_SINGLE;
									}
								}
								else// means WAN speed test
								{
									swl_get_list_ctl = SWL_SPEED_TEST_WAN;
								}
								swl_send_list_ctl = SWL_NOP_WAIT;
								get_list_flag = TRUE;
							}

							g_isSpeedTesting = TRUE;
							if(buf[0] == 1)// not connect to net.
							{
								swl_get_list_ctl = SWL_NOP_WAIT;
								swl_send_list_ctl = SWL_NOP_WAIT;
								get_list_flag = FALSE;
								g_isSpeedTesting = FALSE;
							}
							
						}break;
						
						case CMD_SEND_SPEED_TEST_STATE:
						{
							if((g_resend_cmd.has_cmd == 1)&&(g_resend_cmd.cmd.cmd_id == CMD_SEND_SPEED_TEST_STATE))
							{
								g_resend_cmd.has_cmd = 0;
								g_resend_cmd.is_cmd_changed = 0;
								g_resend_cmd.resend_cnt = 0;
								g_resend_cmd.resend_to = 0;
							}
						}break;
						case CMD_TEST:
						{
							DBG(YELLOW"command: CMD_TEST\n"NONE);
							//DBG("for test to conect ");
							
							g_duringCfg = 1;

							int ret =0;
							
							T_MKOS_APCLI_PARA testg24 = 
								{
									.ssid = "TP_LINK_3019",
									.ch = "11",
									.security = "WPA1PSKWPA2PSK/TKIPAES",
									.key = "nicaiba!",
									.timeout = 5
								};
							T_MKOS_APCLI_PARA testg58 = 
								{
									.ssid = "TP_LINK_3019_5G",
									.ch = "157",
									.security = "WPA1PSKWPA2PSK/TKIPAES",
									.key = "nicaiba!",
									.timeout = 5
								};
							
							//unsigned int cnt = 100;
							//while(cnt >0)
							{
								DBG(RED"Stop connect AP\n"NONE);
								commuStopConnectAp(2);
								DBG(RED"Start connect AP\n"NONE);
								commuConnectAp(&testg24,&testg58);
								commuSaveConnectApInfo(&testg24,&testg58);
								//sleep(30);
								T_MKOS_APCLI_STATUS_INFO g24status;
								T_MKOS_APCLI_STATUS_INFO g58status;
								memset(&g24status,0,sizeof(g24status));
								memset(&g58status,0,sizeof(g58status));

								//commuGetConnectApStatus(&g24status,&g58status);

								//if(g24status.is_enable)
								{
									DBG("2.4g CLI is Enable:\r\n");
									if(g24status.status == -9)
									{
										DBG("2.4g connect is stop\r\n");
									}
									else if(g24status.status == 0)
									{
										DBG("2.4g is connecting\r\n");
									}
									else if(g24status.status == 1)
									{
										DBG("2.4g is connected ok\r\n");
									}
									else
									{
										DBG("2.4g not defined code\r\n");
									}
									DBG("2.4g connect ssid =%s\r\n",g24status.ssid);
									DBG("2.4g connect chan =%s\r\n",g24status.ch);
									DBG("2.4g connect ip =%s\r\n",g24status.ip);
									DBG("2.4g connect mask =%s\r\n",g24status.nm);
								}
								/*else
								{
									DBG("2.4g CLI is Disabled:\r\n");
								}*/


								//if(g58status.is_enable)
								{
									DBG("5.8g CLI is Enable:\r\n");
									if(g58status.status == -9)
									{
										DBG("5.8g connect is stop\r\n");
									}
									else if(g58status.status == 0)
									{
										DBG("5.8g is connecting\r\n");
									}
									else if(g58status.status == 1)
									{
										DBG("5.8g is connected ok\r\n");
									}
									else
									{
										DBG("5.8g not defined code\r\n");
									}
									DBG("5.8g connect ssid =%s\n",g58status.ssid);
									DBG("5.8g connect chan =%s\n",g58status.ch);
									DBG("5.8g connect ip =%s\n",g58status.ip);
									DBG("5.8g connect mask =%s\n",g58status.nm);
								}
								/*else
								{
									DBG("5.8g CLI is Disabled:\r\n");
								}*/
								//cnt--;
							}
						}break;
						default:
							DBG(RED"No THIS CMD !!!!\n"NONE);
					}
					cmd_header_flag = 0x00;
					
				}
				else
				{
					p_out = ptr_plus(cmd_header_loc,2);
					cmd_header_flag = 0x00;
					DBG(RED"CMD CRC ERROR!\n"NONE);
					//cmd_parsed_result = CMD_PARSED_ERR_CHECKSUM;
				}
			}
		}
		p_out = ptr_plus(p_out,1);
	}
}


void *setWifiLogicThread()
{
	unsigned int mk_ctrl_to = 0, resend_to = 0;
	
	char conn_ssid[64] = {0};
	char conn_ch[8] = {0};
	int conn_status = 0,conn_complete = 0;
	char conn_ip[16] = {0};
	char conn_netmask[16] = {0};
	char conn_gateway[16] = {0};
	char conn_dns1[16] = {0};
	char conn_dns2[16] = {0};
	DBG("Creat thread set wifi logic ok\n");
	sleep(1);
	while(1)
	{
		//解析并执行指�?
		praseAppCmd();
		switch(mk_ctrl)
		{
			case MK_HAND_SHAKE:
			{
				STRU_BLE_DATA d = {0};
				d.state = 0;
				d.seq_id = 0;
				d.product_type = CMDTYPE_BLE_MODULE;
				d.cmd_id = CMD_MODULE_HAND_SHAKE;
				d.len = 0;
				d.buff = NULL;
				packDeviceCmd(&d);
				mk_ctrl = MK_HAND_SHAKE_TO;
				mk_ctrl_to = 0;
			}break;
			case MK_HAND_SHAKE_TO:
			{
				mk_ctrl_to++;
				if(mk_ctrl_to >= 50)// 1s
				{
					mk_ctrl = MK_HAND_SHAKE;
				}
			}break;
			case MK_SET_ADV_DATA:
			{
				int ret = 0;
				unsigned char adv_data = 0x00;// not configure
				STRU_BLE_DATA d = {0};
				ret = commuGetSystemConfig();
				DBG(YELLOW"commuGetSystemConfig(): %d\n"NONE,ret);
				/*if(ret == 1)
				{
					adv_data = 0x01;// configured
				}*/
				
				d.state = 0;
				d.seq_id = 0;
				d.product_type = CMDTYPE_BLE_MODULE;
				d.cmd_id = CMD_SET_ADV_DATA;
				d.len = 1;
				d.buff = &adv_data;
				packDeviceCmd(&d);
				mk_ctrl = MK_SET_ADV_DATA_TO;
				mk_ctrl_to = 0;
			}break;
			case MK_SET_ADV_DATA_TO:
			{
				mk_ctrl_to++;
				if(mk_ctrl_to >= 50)// 1s
				{
					mk_ctrl = MK_SET_ADV_DATA;
				}
			}break;
			case MK_START_ADV:
			{
				unsigned char adv_ctrl = 0x01;// start adv
				STRU_BLE_DATA d = {0};
				d.state = 0;
				d.seq_id = 0;
				d.product_type = CMDTYPE_BLE_MODULE;
				d.cmd_id = CMD_ADV_CTRL;
				d.len = 1;
				d.buff = &adv_ctrl;
				packDeviceCmd(&d);
				mk_ctrl = MK_START_ADV_TO;
				mk_ctrl_to = 0;
			}break;
			case MK_START_ADV_TO:
			{
				mk_ctrl_to++;
				if(mk_ctrl_to >= 50)// 1s
				{
					mk_ctrl = MK_START_ADV;
				}
			}break;
			case MK_NOP:
			{
				
			}break;
			default:break;
		}
		//获取列表
		if(get_list_flag == TRUE)
		{
			switch(swl_get_list_ctl)
			{
				case SWL_FIRST_GET_ROUTE_LIST:
				{
					char buf[8192] = {0};//{"0xabcE9998CE4B88A-5G\n157\nWPA2/AES\n-68\n112233445C\n\0"};//{0};
					memset(buf,0,8192);
					DBG("swl_get_list_ctl: SWL_FIRST_GET_ROUTE_LIST\n");
					commuGetApSiteList(buf);
					DBG("buf: %s\n",buf);
					praseRouteList(buf,route_list,&route_cnt);
					DBG("route_cnt: %d\n",route_cnt);
					swl_get_list_ctl = SWL_SECOND_GET_ROUTE_LIST;
					swl_send_list_ctl = SWL_SEND_ROUTE_LIST;
					timer_cnt = 0;
					
				}break;
				case SWL_SECOND_GET_ROUTE_LIST:
				{
					
					if(timer_cnt >= 400)// 8s
					{
						char buf[8192] = {0};//{"0xabcE9998CE4B88A-5G\n157\nWPA2/AES\n-68\n112233445C\n\0"};//{0};
						memset(buf,0,8192);
						DBG("swl_get_list_ctl: SWL_SECOND_GET_ROUTE_LIST\n");
						commuGetApSiteListSecond(buf);
						praseRouteList(buf,route_list,&route_cnt);
						swl_get_list_ctl = SWL_NOP_WAIT;
						//swl_send_list_ctl = SWL_SEND_ROUTE_LIST;
						timer_cnt = 0;
					}
					
				}break;
				case SWL_GET_ROUTE_LIST_FROM_CATCH:
				{
					DBG("swl_get_list_ctl: SWL_GET_ROUTE_LIST_FROM_CATCH\n");
					pthread_mutex_lock(&mut);
					if(cache_route_list_status2[cache_copy_from] == 1)
					{
						int i = 0;
						for(i = 0; i < cache_route_list_tmp_cnt2[cache_copy_from]; i++)  
						{
							memcpy(&route_list[route_cnt],&cache_route_list2[cache_copy_from][i],sizeof(ROUTE_T));
							route_cnt++;
							if(route_cnt >= MAX_ROUTE_LIST_NUM)
							{
								route_cnt = 0;
							}
						}
						DBG(YELLOW"SWL_GET_ROUTE_LIST_FROM_CATCH status = 1, route_cnt = %d, has_send_route_cnt = %d\n"NONE,route_cnt,has_send_route_cnt);
						swl_get_list_ctl = SWL_GET_ROUTE_LIST_FROM_CATCH_2;
					}
					else if(cache_route_list_status2[cache_copy_from] == 2)
					{
						int i = 0;
						for(i = 0; i < cache_route_list_cnt2[cache_copy_from]; i++)
						{
							memcpy(&route_list[route_cnt],&cache_route_list2[cache_copy_from][i],sizeof(ROUTE_T));
							route_cnt++;
							if(route_cnt >= MAX_ROUTE_LIST_NUM)
							{
								route_cnt = 0;
							}
						}
						DBG(YELLOW"SWL_GET_ROUTE_LIST_FROM_CATCH status = 2, route_cnt = %d, has_send_route_cnt = %d\n"NONE,route_cnt,has_send_route_cnt);
						swl_get_list_ctl = SWL_NOP_WAIT;
					}
					else
					{
						//swl_get_list_ctl = SWL_NOP_WAIT;
					}
					pthread_mutex_unlock(&mut);
					swl_send_list_ctl = SWL_SEND_ROUTE_LIST;
				}break;
				case SWL_GET_ROUTE_LIST_FROM_CATCH_2:
				{
					DBG("swl_get_list_ctl: SWL_GET_ROUTE_LIST_FROM_CATCH_2\n");
					pthread_mutex_lock(&mut);
					if(cache_route_list_status2[cache_copy_from] == 2)
					{
						int i = 0;
						for(i = cache_route_list_tmp_cnt2[cache_copy_from]; i < cache_route_list_cnt2[cache_copy_from]; i++)
						{
							memcpy(&route_list[route_cnt],&cache_route_list2[cache_copy_from][i],sizeof(ROUTE_T));
							route_cnt++;
							if(route_cnt >= MAX_ROUTE_LIST_NUM)
							{
								route_cnt = 0;
							}
						}
						swl_get_list_ctl = SWL_NOP_WAIT;
						DBG(YELLOW"SWL_GET_ROUTE_LIST_FROM_CATCH_2: route_cnt = %d, has_send_route_cnt = %d\n"NONE,route_cnt,has_send_route_cnt);
					}
					pthread_mutex_unlock(&mut);
				}break;
				case SWL_GET_WIFI_CONN_STATE:
				{
				//调用中继接口后，5S后查询状�?
				//-1的话需要等待到超时
				//-2�?3的话需要等�?S
				//当然这个等待应该要每一秒查询一下状�?
					if(timer_cnt >= 250)// 5s
					{
						STRU_BLE_DATA d = {0};
						char buf[400] = {0};
						T_MKOS_APCLI_STATUS_INFO g24status;
						T_MKOS_APCLI_STATUS_INFO g58status;
						
						memset(&g24status,0,sizeof(g24status));
						memset(&g58status,0,sizeof(g58status));

						commuGetConnectApStatus(&g24status,&g58status);
						DBG(RED"g24status.status: %d g58status.status: %d\n"NONE,g24status.status,g58status.status);
						
						swl_send_list_ctl = SWL_NOP_WAIT;
						timer_cnt = 0;
						conn_to++;
						DBG(RED"conn_to: %d\n"NONE,conn_to);

						if(conn_to > conn_timeout)
						{
							int mode = -1;
							if(g24status.is_enable)
							{
								switch(g24status.status)
								{
									case -9:
									case WIFI_CONN_CONNECTING:{buf[0] = TO_APP_CONN_CONNECTING;}break;
									case WIFI_CONN_SUCCESS:{buf[0] = TO_APP_CONN_SUCCESS;}break;
									case WIFI_CONN_DHCP_ERROR:{buf[0] = TO_APP_CONN_DHCP_ERROR;}break;
									case WIFI_CONN_NOT_FIND_ROUTE:{buf[0] = TO_APP_CONN_NOT_FIND_ROUTE;}break;
									case WIFI_CONN_PASSWORD_ERROR:{buf[0] = TO_APP_CONN_PASSWORD_ERROR;}break;
									case WIFI_CONN_FAILED:
									default:{buf[0] = TO_APP_CONN_OTHER_ERROR;}
								}
							}
							
							if(g58status.is_enable)
							{
								switch(g58status.status)
								{
									case -9:
									case WIFI_CONN_CONNECTING:{buf[1] = TO_APP_CONN_CONNECTING;}break;
									case WIFI_CONN_SUCCESS:{buf[1] = TO_APP_CONN_SUCCESS;}break;
									case WIFI_CONN_DHCP_ERROR:{buf[1] = TO_APP_CONN_DHCP_ERROR;}break;
									case WIFI_CONN_NOT_FIND_ROUTE:{buf[1] = TO_APP_CONN_NOT_FIND_ROUTE;}break;
									case WIFI_CONN_PASSWORD_ERROR:{buf[1] = TO_APP_CONN_PASSWORD_ERROR;}break;
									case WIFI_CONN_FAILED:
									default:{buf[1] = TO_APP_CONN_OTHER_ERROR;}
								}
							}
							
							if((g24status.is_enable)&&((g24status.status == -9)||(g24status.status == WIFI_CONN_CONNECTING)))
							{
								buf[0] = TO_APP_CONN_CONN_TIMEOUT;
							}
							if((g58status.is_enable)&&((g58status.status == -9)||(g58status.status == WIFI_CONN_CONNECTING)))
							{
								buf[1] = TO_APP_CONN_CONN_TIMEOUT;
							}
							if(set_wifi_type == 1)
							{
								buf[1] = TO_APP_CONN_NOT_SET;
							}
							if(set_wifi_type == 2)
							{
								buf[0] = TO_APP_CONN_NOT_SET;
							}
							buf[2] = g24ext_status;
							buf[3] = g58ext_status;

							//configure failed, set led blink
							int ret = 0;
							// off all led
							ret = commuSetLedControl(LED2,0,1);
							ret = commuSetLedControl(LED3,0,1);
							ret = commuSetLedControl(LED4,0,1);
							// blink
							ret = commuSetLedControl(LED2,5,5);
							ret = commuSetLedControl(LED3,5,5);
							ret = commuSetLedControl(LED4,5,5);
							if(ret != 0)
							{
								DBG(RED"commuSetLedControl() error\n"NONE);
							}
							
							g_duringCfg = 0;
							g_duringSetWifiLogic = 0;
							conn_to = 0;
							swl_send_list_ctl = SWL_NOP_WAIT;
							swl_get_list_ctl = SWL_NOP_WAIT;//发过结果了，不用查询了�?
							
							if( ((g24status.is_enable)&&(g24status.status != WIFI_CONN_SUCCESS))&&
								((g58status.is_enable)&&(g58status.status != WIFI_CONN_SUCCESS)) ){mode = 2;}
							else if((g24status.is_enable)&&(g24status.status != WIFI_CONN_SUCCESS)){mode = 0;}
							else if((g58status.is_enable)&&(g58status.status != WIFI_CONN_SUCCESS)){mode = 1;}
								
							commuStopConnectAp(mode);
						}
						else
						{
							
							if(set_wifi_type == 1)
							{
								buf[1] = TO_APP_CONN_NOT_SET;
								switch(g24status.status)
								{
									case -9:
									case WIFI_CONN_DHCP_ERROR:
									case WIFI_CONN_NOT_FIND_ROUTE:
									case WIFI_CONN_PASSWORD_ERROR:
									case WIFI_CONN_CONNECTING:{buf[0] = TO_APP_CONN_CONNECTING;}break;
									case WIFI_CONN_SUCCESS:
									{
										buf[0] = TO_APP_CONN_SUCCESS;
										//if(g24extended == FALSE)
										{
											T_MKOS_APCLI_PARA wifi_ext = {0};
											char newssid[64] = {0};
											strcpy(wifi_ext.ssid, wifi[0].ssid);
											strcpy(wifi_ext.security, wifi[0].security);
											strcpy(wifi_ext.ch, wifi[0].ch);
											strcpy(wifi_ext.key, wifi[0].key);
											DBG("ssid: %s\nsecurity: %s\nch: %s\nkey: %s\n",newssid,wifi_ext.security,wifi_ext.ch,wifi_ext.key);
											time(&t_stop_conn);
											DBG(YELLOW"time conn: %f\n"NONE,difftime(t_stop_conn,t_start_conn));

											if(g24extended == FALSE)
											{
												sprintf(newssid,"%s-24gEXT",wifi_ext.ssid);
												if(0 != commuSetExtAp(0,"1","0",newssid,wifi_ext.security,wifi_ext.key,wifi_ext.ch))
												{
													g24ext_status = WIFI_EXT_ERROR;
												}
												else
												{
													g24ext_status = WIFI_EXT_SUCCESS;
													g24extended = TRUE;
												}
											}
											time(&t_stop_24g_ext);
											DBG(YELLOW"time 24g ext: %f\n"NONE,difftime(t_stop_24g_ext,t_stop_conn));

											if(g58extended == FALSE)
											{
												memset(newssid, 0, sizeof(newssid));
												sprintf(newssid,"%s-5gEXT",wifi_ext.ssid);
												if(0 != commuSetExtAp(1,"1","0",newssid,wifi_ext.security,wifi_ext.key,NULL))
												{
													g58ext_status = WIFI_EXT_ERROR;
												}
												else
												{
													g58ext_status = WIFI_EXT_SUCCESS;
													g58extended = TRUE;
												}
											}
											time(&t_stop_58g_ext);
											DBG(YELLOW"time 58g ext: %f\n"NONE,difftime(t_stop_58g_ext,t_stop_24g_ext));
											
											buf[2] = g24ext_status;
											buf[3] = g58ext_status;
										}
										
									}break;
									//case WIFI_CONN_DHCP_ERROR:{buf[0] = TO_APP_CONN_DHCP_ERROR;}break;
									//case WIFI_CONN_NOT_FIND_ROUTE:{buf[0] = TO_APP_CONN_NOT_FIND_ROUTE;}break;
									//case WIFI_CONN_PASSWORD_ERROR:{buf[0] = TO_APP_CONN_PASSWORD_ERROR;}break;
									case WIFI_CONN_FAILED:
									default:{buf[0] = TO_APP_CONN_OTHER_ERROR;}
								}
								switch (g24status.status)
								{
									case -9:
									case WIFI_CONN_CONNECTING:
									case WIFI_CONN_SUCCESS:
									case WIFI_CONN_DHCP_ERROR:
									case WIFI_CONN_NOT_FIND_ROUTE:
									case WIFI_CONN_PASSWORD_ERROR: break;
									default:
									{
										buf[2] = g24ext_status;
										buf[3] = g58ext_status;
										commuStopConnectAp(0);
										
										swl_send_list_ctl = SWL_NOP_WAIT;
										swl_get_list_ctl = SWL_NOP_WAIT;//发过结果了，不用查询了�?
										conn_to = 0;
										g_duringCfg = 0;
										g_duringSetWifiLogic = 0;
									}
								}
								
							}
							else if(set_wifi_type == 2)
							{
								buf[0] = TO_APP_CONN_NOT_SET;
								switch(g58status.status)
								{
									case -9:
									case WIFI_CONN_DHCP_ERROR:
									case WIFI_CONN_NOT_FIND_ROUTE:
									case WIFI_CONN_PASSWORD_ERROR:
									case WIFI_CONN_CONNECTING:{buf[1] = TO_APP_CONN_CONNECTING;}break;
									case WIFI_CONN_SUCCESS:
									{
										buf[1] = TO_APP_CONN_SUCCESS;
										//if(g58extended == FALSE)
										{
											T_MKOS_APCLI_PARA wifi_ext = {0};
											char newssid[64] = {0};
											strcpy(wifi_ext.ssid, wifi[0].ssid);
											strcpy(wifi_ext.security, wifi[0].security);
											strcpy(wifi_ext.ch, wifi[0].ch);
											strcpy(wifi_ext.key, wifi[0].key);
											DBG("ssid: %s\nsecurity: %s\nch: %s\nkey: %s\n",newssid,wifi_ext.security,wifi_ext.ch,wifi_ext.key);
											time(&t_stop_conn);
											DBG(YELLOW"time conn: %f\n"NONE,difftime(t_stop_conn,t_start_conn));

											if(g24extended == FALSE)
											{
												sprintf(newssid,"%s-24gEXT",wifi_ext.ssid);
												if(0 != commuSetExtAp(0,"1","0",newssid,wifi_ext.security,wifi_ext.key,NULL))
												{
													g24ext_status = WIFI_EXT_ERROR;
												}
												else
												{
													g24ext_status = WIFI_EXT_SUCCESS;
													g24extended = TRUE;
												}
											}
											time(&t_stop_24g_ext);
											DBG(YELLOW"time 24g ext: %f\n"NONE,difftime(t_stop_24g_ext,t_stop_conn));

											if(g58extended == FALSE)
											{
												memset(newssid, 0, sizeof(newssid));
												sprintf(newssid,"%s-5gEXT",wifi_ext.ssid);
												if(0 != commuSetExtAp(1,"1","0",newssid,wifi_ext.security,wifi_ext.key,wifi_ext.ch))
												{
													g58ext_status = WIFI_EXT_ERROR;
												}
												else
												{
													g58ext_status = WIFI_EXT_SUCCESS;
													g58extended = TRUE;
												}
											}
											time(&t_stop_58g_ext);
											DBG(YELLOW"time 58g ext: %f\n"NONE,difftime(t_stop_58g_ext,t_stop_24g_ext));
											
											buf[2] = g24ext_status;
											buf[3] = g58ext_status;
										}
										
									}break;
									//case WIFI_CONN_DHCP_ERROR:{buf[1] = TO_APP_CONN_DHCP_ERROR;}break;
									//case WIFI_CONN_NOT_FIND_ROUTE:{buf[1] = TO_APP_CONN_NOT_FIND_ROUTE;}break;
									//case WIFI_CONN_PASSWORD_ERROR:{buf[1] = TO_APP_CONN_PASSWORD_ERROR;}break;
									case WIFI_CONN_FAILED:
									default:{buf[1] = TO_APP_CONN_OTHER_ERROR;}
								}
								switch (g58status.status)
								{
									case -9:
									case WIFI_CONN_CONNECTING:
									case WIFI_CONN_SUCCESS:
									case WIFI_CONN_DHCP_ERROR:
									case WIFI_CONN_NOT_FIND_ROUTE:
									case WIFI_CONN_PASSWORD_ERROR: break;
									default:
									{
										buf[2] = g24ext_status;
										buf[3] = g58ext_status;
										commuStopConnectAp(1);
										
										swl_send_list_ctl = SWL_NOP_WAIT;
										swl_get_list_ctl = SWL_NOP_WAIT;//发过结果了，不用查询了�?
										conn_to = 0;
										g_duringCfg = 0;
										g_duringSetWifiLogic = 0;
									}
								}
							}
							else if(set_wifi_type == 3)
							{
								DBG(RED"set_wifi_type == 3\n"NONE);
								switch(g24status.status)
								{
									case -9:
									case WIFI_CONN_DHCP_ERROR:
									case WIFI_CONN_NOT_FIND_ROUTE:
									case WIFI_CONN_PASSWORD_ERROR:
									case WIFI_CONN_CONNECTING:{buf[0] = TO_APP_CONN_CONNECTING;}break;
									case WIFI_CONN_SUCCESS:
									{
										buf[0] = TO_APP_CONN_SUCCESS;
										
									}break;
									//case WIFI_CONN_DHCP_ERROR:{buf[0] = TO_APP_CONN_DHCP_ERROR;}break;
									//case WIFI_CONN_NOT_FIND_ROUTE:{buf[0] = TO_APP_CONN_NOT_FIND_ROUTE;}break;
									//case WIFI_CONN_PASSWORD_ERROR:{buf[0] = TO_APP_CONN_PASSWORD_ERROR;}break;
									case WIFI_CONN_FAILED:
									default:{buf[0] = TO_APP_CONN_OTHER_ERROR;}
								}
								switch(g58status.status)
								{
									case -9:
									case WIFI_CONN_DHCP_ERROR:
									case WIFI_CONN_NOT_FIND_ROUTE:
									case WIFI_CONN_PASSWORD_ERROR:
									case WIFI_CONN_CONNECTING:{buf[1] = TO_APP_CONN_CONNECTING;}break;
									case WIFI_CONN_SUCCESS:
									{
										buf[1] = TO_APP_CONN_SUCCESS;
										
									}break;
									//case WIFI_CONN_DHCP_ERROR:{buf[1] = TO_APP_CONN_DHCP_ERROR;}break;
									//case WIFI_CONN_NOT_FIND_ROUTE:{buf[1] = TO_APP_CONN_NOT_FIND_ROUTE;}break;
									//case WIFI_CONN_PASSWORD_ERROR:{buf[1] = TO_APP_CONN_PASSWORD_ERROR;}break;
									case WIFI_CONN_FAILED:
									default:{buf[1] = TO_APP_CONN_OTHER_ERROR;}
								}

								if((g24status.status == WIFI_CONN_SUCCESS)&&(g58status.status == WIFI_CONN_SUCCESS))
								{
									//extend all
									T_MKOS_APCLI_PARA wifi_ext = {0};
									char newssid[64] = {0};
									strcpy(wifi_ext.ssid, wifi[0].ssid);
									strcpy(wifi_ext.security, wifi[0].security);
									strcpy(wifi_ext.ch, wifi[0].ch);
									strcpy(wifi_ext.key, wifi[0].key);
									DBG("ssid: %s\nsecurity: %s\nch: %s\nkey: %s\n",newssid,wifi_ext.security,wifi_ext.ch,wifi_ext.key);
									time(&t_stop_conn);
									DBG(YELLOW"time conn: %f\n"NONE,difftime(t_stop_conn,t_start_conn));

									if(g24extended == FALSE)
									{
										sprintf(newssid,"%s-24gEXT",wifi_ext.ssid);
										//strcat(wifi_ext.ssid,"-24gEXT");
										//strcpy(newssid,wifi_ext.ssid);
										DBG(RED"newssid: %s\n"NONE,newssid);
										if(0 != commuSetExtAp(0,"1","0",newssid,wifi_ext.security,wifi_ext.key,wifi_ext.ch))
										{
											g24ext_status = WIFI_EXT_ERROR;
											DBG(RED"commuSetExtAp(0): WIFI_EXT_ERROR\n"NONE);
										}
										else
										{
											g24ext_status = WIFI_EXT_SUCCESS;
											g24extended = TRUE;
											DBG(RED"commuSetExtAp(0): WIFI_EXT_SUCCESS\n"NONE);
										}
									}
									time(&t_stop_24g_ext);
									DBG(YELLOW"time 24g ext: %f\n"NONE,difftime(t_stop_24g_ext,t_stop_conn));
									buf[2] = g24ext_status;

									memset(&wifi_ext, 0, sizeof(wifi_ext));
									memset(newssid, 0, sizeof(newssid));
									strcpy(wifi_ext.ssid, wifi[1].ssid);
									strcpy(wifi_ext.security, wifi[1].security);
									strcpy(wifi_ext.ch, wifi[1].ch);
									strcpy(wifi_ext.key, wifi[1].key);
									DBG("ssid: %s\nsecurity: %s\nch: %s\nkey: %s\n",newssid,wifi_ext.security,wifi_ext.ch,wifi_ext.key);

									if(g58extended == FALSE)
									{
										sprintf(newssid,"%s-5gEXT",wifi_ext.ssid);
										//strcat(wifi_ext.ssid,"-5gEXT");
										//strcpy(newssid,wifi_ext.ssid);
										DBG(RED"newssid: %s\n"NONE,newssid);
										if(0 != commuSetExtAp(1,"1","0",newssid,wifi_ext.security,wifi_ext.key,wifi_ext.ch))
										{
											g58ext_status = WIFI_EXT_ERROR;
											DBG(RED"commuSetExtAp(1): WIFI_EXT_ERROR\n"NONE);
										}
										else
										{
											g58ext_status = WIFI_EXT_SUCCESS;
											g58extended = TRUE;
											DBG(RED"commuSetExtAp(1): WIFI_EXT_SUCCESS\n"NONE);
										}
									}
									time(&t_stop_58g_ext);
									DBG(YELLOW"time 58g ext: %f\n"NONE,difftime(t_stop_58g_ext,t_stop_24g_ext));
									buf[3] = g58ext_status;

									
								}
								else
								{
									if(conn_to == conn_timeout)// extend, when timeout
									{
										if(g24status.status == WIFI_CONN_SUCCESS)
										{
											// stop conn 5G 
											commuStopConnectAp(1);
											
											T_MKOS_APCLI_PARA wifi_ext = {0};
											char newssid[64] = {0};
											strcpy(wifi_ext.ssid, wifi[0].ssid);
											strcpy(wifi_ext.security, wifi[0].security);
											strcpy(wifi_ext.ch, wifi[0].ch);
											strcpy(wifi_ext.key, wifi[0].key);
											DBG("ssid: %s\nsecurity: %s\nch: %s\nkey: %s\n",newssid,wifi_ext.security,wifi_ext.ch,wifi_ext.key);

											if(g24extended == FALSE)
											{
												sprintf(newssid,"%s-24gEXT",wifi_ext.ssid);
												//strcat(wifi_ext.ssid,"-24gEXT");
												//strcpy(newssid,wifi_ext.ssid);
												DBG(RED"newssid: %s\n"NONE,newssid);
												if(0 != commuSetExtAp(0,"1","0",newssid,wifi_ext.security,wifi_ext.key,wifi_ext.ch))
												{
													g24ext_status = WIFI_EXT_ERROR;
													DBG(RED"commuSetExtAp(0): WIFI_EXT_ERROR\n"NONE);
												}
												else
												{
													g24ext_status = WIFI_EXT_SUCCESS;
													g24extended = TRUE;
													DBG(RED"commuSetExtAp(0): WIFI_EXT_SUCCESS\n"NONE);
												}
											}
											buf[2] = g24ext_status;

											if((conn_to == conn_timeout)&&(g58status.status != WIFI_CONN_SUCCESS)&&(g58extended == FALSE))
											{
												memset(newssid, 0, sizeof(newssid));
												sprintf(newssid,"%s-5gEXT",wifi_ext.ssid);
												//strcat(wifi_ext.ssid,"-24gEXT");
												//strcpy(newssid,wifi_ext.ssid);
												DBG(RED"newssid: %s\n"NONE,newssid);
												if(0 != commuSetExtAp(1,"1","0",newssid,wifi_ext.security,wifi_ext.key,NULL))
												{
													g58ext_status = WIFI_EXT_ERROR;
													DBG(RED"commuSetExtAp(1): WIFI_EXT_ERROR\n"NONE);
												}
												else
												{
													g58ext_status = WIFI_EXT_SUCCESS;
													g58extended = TRUE;
													DBG(RED"commuSetExtAp(1): WIFI_EXT_SUCCESS\n"NONE);
												}

												buf[3] = g58ext_status;
											}
										}
										else if(g58status.status == WIFI_CONN_SUCCESS)
										{
											// stop conn 2.4G
											commuStopConnectAp(0);
											
											T_MKOS_APCLI_PARA wifi_ext = {0};
											char newssid[64] = {0};
											strcpy(wifi_ext.ssid, wifi[1].ssid);
											strcpy(wifi_ext.security, wifi[1].security);
											strcpy(wifi_ext.ch, wifi[1].ch);
											strcpy(wifi_ext.key, wifi[1].key);
											DBG("ssid: %s\nsecurity: %s\nch: %s\nkey: %s\n",newssid,wifi_ext.security,wifi_ext.ch,wifi_ext.key);

											if(g58extended == FALSE)
											{
												sprintf(newssid,"%s-5gEXT",wifi_ext.ssid);
												//strcat(wifi_ext.ssid,"-5gEXT");
												//strcpy(newssid,wifi_ext.ssid);
												DBG(RED"newssid: %s\n"NONE,newssid);
												if(0 != commuSetExtAp(1,"1","0",newssid,wifi_ext.security,wifi_ext.key,wifi_ext.ch))
												{
													g58ext_status = WIFI_EXT_ERROR;
													DBG(RED"commuSetExtAp(1): WIFI_EXT_ERROR\n"NONE);
												}
												else
												{
													g58ext_status = WIFI_EXT_SUCCESS;
													g58extended = TRUE;
													DBG(RED"commuSetExtAp(1): WIFI_EXT_SUCCESS\n"NONE);
												}
											}
											
											buf[3] = g58ext_status;

											if((conn_to == conn_timeout)&&(g24status.status != WIFI_CONN_SUCCESS)&&(g24extended == FALSE))
											{
												memset(newssid, 0, sizeof(newssid));
												sprintf(newssid,"%s-24gEXT",wifi_ext.ssid);
												//strcat(wifi_ext.ssid,"-24gEXT");
												//strcpy(newssid,wifi_ext.ssid);
												DBG(RED"newssid: %s\n"NONE,newssid);
												if(0 != commuSetExtAp(0,"1","0",newssid,wifi_ext.security,wifi_ext.key,NULL))
												{
													g24ext_status = WIFI_EXT_ERROR;
													DBG(RED"commuSetExtAp(0): WIFI_EXT_ERROR\n"NONE);
												}
												else
												{
													g24ext_status = WIFI_EXT_SUCCESS;
													g24extended = TRUE;
													DBG(RED"commuSetExtAp(0): WIFI_EXT_SUCCESS\n"NONE);
												}

												
												buf[2] = g24ext_status;
											}
										}
										else
										{
											// no connect, not extend
										}
									}
								}
								int flag_g24 = FALSE, flag_g58 = FALSE;
								switch (g24status.status)
								{
									case -9:
									case WIFI_CONN_CONNECTING:
									case WIFI_CONN_SUCCESS:
									case WIFI_CONN_DHCP_ERROR:
									case WIFI_CONN_NOT_FIND_ROUTE:
									case WIFI_CONN_PASSWORD_ERROR: break;
									default:
									{
										flag_g24 = TRUE;
									}
								}
								switch (g58status.status)
								{
									case -9:
									case WIFI_CONN_CONNECTING:
									case WIFI_CONN_SUCCESS:
									case WIFI_CONN_DHCP_ERROR:
									case WIFI_CONN_NOT_FIND_ROUTE:
									case WIFI_CONN_PASSWORD_ERROR: break;
									default:
									{
										flag_g58 = TRUE;
									}
								}
								if(flag_g24 == TRUE && flag_g58 == TRUE)
								{
									buf[2] = g24ext_status;
									buf[3] = g58ext_status;
									commuStopConnectAp(2);
									
									swl_send_list_ctl = SWL_NOP_WAIT;
									swl_get_list_ctl = SWL_NOP_WAIT;//发过结果了，不用查询了�?
									conn_to = 0;
									g_duringCfg = 0;
									g_duringSetWifiLogic = 0;
								}
								
							}
							
							if((g24extended == TRUE)&&(g58extended == TRUE))
							{
								
								g_duringCfg = 0;

								//configured, set led blink
								int ret = 0;
								// off all led
								ret = commuSetLedControl(LED2,0,1);
								ret = commuSetLedControl(LED3,0,1);
								ret = commuSetLedControl(LED4,0,1);
								// blink
								ret = commuSetLedControl(LED2,1,0);
								ret = commuSetLedControl(LED3,1,0);
								ret = commuSetLedControl(LED4,1,9);
								if(ret != 0)
								{
									DBG(RED"commuSetLedControl() error\n"NONE);
								}
								
								if(set_wifi_type == 3)
								{
									switch (g24status.status)
									{
										case -9:
										case WIFI_CONN_CONNECTING:{buf[0] = TO_APP_CONN_CONN_TIMEOUT;}break;
										case WIFI_CONN_SUCCESS:break;
										case WIFI_CONN_DHCP_ERROR:{buf[0] = TO_APP_CONN_DHCP_ERROR;}break;
										case WIFI_CONN_NOT_FIND_ROUTE:{buf[0] = TO_APP_CONN_NOT_FIND_ROUTE;}break;
										case WIFI_CONN_PASSWORD_ERROR:{buf[0] = TO_APP_CONN_PASSWORD_ERROR;}break;
										default:{buf[0] = TO_APP_CONN_OTHER_ERROR;}
									}
									
									switch (g58status.status)
									{
										case -9:
										case WIFI_CONN_CONNECTING:{buf[1] = TO_APP_CONN_CONN_TIMEOUT;}break;
										case WIFI_CONN_SUCCESS:break;
										case WIFI_CONN_DHCP_ERROR:{buf[1] = TO_APP_CONN_DHCP_ERROR;}break;
										case WIFI_CONN_NOT_FIND_ROUTE:{buf[1] = TO_APP_CONN_NOT_FIND_ROUTE;}break;
										case WIFI_CONN_PASSWORD_ERROR:{buf[1] = TO_APP_CONN_PASSWORD_ERROR;}break;
										default:{buf[1] = TO_APP_CONN_OTHER_ERROR;}
									}
								}
								
								if((set_wifi_type == 3)&&(g24status.status == WIFI_CONN_SUCCESS)&&(g58status.status == WIFI_CONN_SUCCESS))
								{
									buf[4] = 1;//start speed test
									swl_send_list_ctl = SWL_SPEED_TEST_LAN_SINGLE;
								}
								else
								{
									buf[4] = 0;
									swl_send_list_ctl = SWL_NOP_WAIT;
									//select upper net
									
									switch(set_wifi_type)
									{
										case 1:
										{
											//select upper net
											commuSelectAplientInterfaceToUse(0);
										}break;
										case 2:
										{
											//select upper net
											commuSelectAplientInterfaceToUse(1);
										}break;
										case 3:
										{
											if(g58status.status == WIFI_CONN_SUCCESS)
											{
												commuSelectAplientInterfaceToUse(1);
											}
											else if(g24status.status == WIFI_CONN_SUCCESS)
											{
												commuSelectAplientInterfaceToUse(0);
											}
											else
											{
												//error, no upper net connected
												DBG(RED"error, no upper net connected"NONE);
											}
										}break;
										default:break;
									}
									time(&t_stop_select);
									DBG(YELLOW"time select: %f\n"NONE,difftime(t_stop_select,t_stop_58g_ext));

									//save set to configured
									commuSetSystemConfig(1);
									DBG(YELLOW"commuSetSystemConfig(1)\n"NONE);
									g_duringSetWifiLogic = 0;
									
									time(&t_stop_save);
									DBG(YELLOW"time save: %f\n"NONE,difftime(t_stop_save,t_stop_select));
									DBG(YELLOW"time all: %f\n"NONE,difftime(t_stop_save,t_start_conn));
								}
								
								mk_ctrl = MK_SET_ADV_DATA;//configure connection success, so change the adv data.
								
								swl_get_list_ctl = SWL_NOP_WAIT;//发过结果了，不用查询了�?
								conn_to = 0;
							}
						}

						if(swl_get_list_ctl == SWL_NOP_WAIT)//means this is the end cmd,need resend
						{
							d.need_resend = 1;
						}
						d.state = 0;
						d.seq_id = 0;
						d.product_type = CMDTYPE_SET_WIFI;
						d.cmd_id = CMD_SEND_WIFI_CONN_STATE;
						d.len = 5;
						d.buff = buf;
						packDeviceCmd(&d);
						g_is_upload_set_wifi_state = 0;
						DBG("route list pack to send buffer\n");

						if(g24status.is_enable)
						{
							DBG("2.4g CLI is Enable:\r\n");
							if(g24status.status == -9)
							{
								DBG("2.4g connect is stop\r\n");
							}
							else if(g24status.status == 0)
							{
								DBG("2.4g is connecting\r\n");
							}
							else if(g24status.status == 1)
							{
								DBG("2.4g is connected ok\r\n");
							}
							else
							{
								DBG("2.4g not defined code\r\n");
							}
							DBG("2.4g connect ssid =%s\r\n",g24status.ssid);
							DBG("2.4g connect chan =%s\r\n",g24status.ch);
							DBG("2.4g connect ip =%s\r\n",g24status.ip);
							DBG("2.4g connect mask =%s\r\n",g24status.nm);
						}
						else
						{
							DBG("2.4g CLI is Disabled:\r\n");
						}


						if(g58status.is_enable)
						{
							DBG("5.8g CLI is Enable:\r\n");
							if(g58status.status == -9)
							{
								DBG("5.8g connect is stop\r\n");
							}
							else if(g58status.status == 0)
							{
								DBG("5.8g is connecting\r\n");
							}
							else if(g58status.status == 1)
							{
								DBG("5.8g is connected ok\r\n");
							}
							else
							{
								DBG("5.8g not defined code\r\n");
							}
							DBG("5.8g connect ssid =%s\r\n",g58status.ssid);
							DBG("5.8g connect chan =%s\r\n",g58status.ch);
							DBG("5.8g connect ip =%s\r\n",g58status.ip);
							DBG("5.8g connect mask =%s\r\n",g58status.nm);
						}
						else
						{
							DBG("5.8g CLI is Disabled:\r\n");
						}
					}
				}break;
				case SWL_SPEED_TEST_LAN_DUAL:
				{
					int recommand_band = 0;
					T_MKOS_APCLI_WAN_SPEEDTEST_INFO g24,g58;
					memset(&g24,0,sizeof(g24));
					memset(&g58,0,sizeof(g58));
					recommand_band = commuGetSpeedTestStatusWAN(&g24,&g58);
					
					STRU_BLE_DATA d = {0};
					char buf[23] = {0};
					d.state = 0;
					d.seq_id = 0;
					d.product_type = CMDTYPE_WIFI_CONFIG_PARAM;
					d.cmd_id = CMD_SEND_SPEED_TEST_STATE;
					d.len = sizeof(buf)/sizeof(char);
					d.buff = buf;

					buf[0] = 0;// LAN speed test
					buf[1] = 0;// reserved
					buf[2] = (unsigned char)recommand_band;
					buf[3] = (unsigned char)(g24.sends >>24);
					buf[4] = (unsigned char)(g24.sends >>16);
					buf[5] = (unsigned char)(g24.sends >>8);
					buf[6] = (unsigned char)(g24.sends);
					buf[7] = (unsigned char)(((int)g24.minRtt) >>8);
					buf[8] = (unsigned char)((int)g24.minRtt);
					buf[9] = (unsigned char)(((int)g24.maxRtt) >>8);
					buf[10] = (unsigned char)((int)g24.maxRtt);
					buf[11] = (unsigned char)(((int)g24.averRtt) >>8);
					buf[12] = (unsigned char)((int)g24.averRtt);
					buf[13] = (unsigned char)(g58.sends >>24);
					buf[14] = (unsigned char)(g58.sends >>16);
					buf[15] = (unsigned char)(g58.sends >>8);
					buf[16] = (unsigned char)(g58.sends);
					buf[17] = (unsigned char)(((int)g58.minRtt) >>8);
					buf[18] = (unsigned char)((int)g58.minRtt);
					buf[19] = (unsigned char)(((int)g58.maxRtt) >>8);
					buf[20] = (unsigned char)((int)g58.maxRtt);
					buf[21] = (unsigned char)(((int)g58.averRtt) >>8);
					buf[22] = (unsigned char)((int)g58.averRtt);
					
					packDeviceCmd(&d);
					swl_get_list_ctl = SWL_NOP_WAIT;
					swl_send_list_ctl = SWL_NOP_WAIT;
					timer_cnt = 0;
					g_isSpeedTesting = FALSE;
				}break;
				case SWL_SPEED_TEST_LAN_SINGLE:
				{
					STRU_BLE_DATA d = {0};
					unsigned char buf[40] = {0};
					int recommand_band = 0, len = 0;
					double g24Speed = 0, g58Speed = 0;
					T_MKOS_APCLI_LAN_SPEEDTEST_INFO g24_test_result;
					T_MKOS_APCLI_LAN_SPEEDTEST_INFO g58_test_result;
					
					memset(&g24_test_result,0,sizeof(g24_test_result));
					memset(&g58_test_result,0,sizeof(g58_test_result));

					recommand_band = commuGetAplientInterfaceToUse();
					DBG(YELLOW"recommand_band: %d\n"NONE,recommand_band);
					//speed test
					if(recommand_band == 0)//2.4G
					{
						int err = 0;
						err = commuGetBrSpeedTestStatusRunLAN(3000,15,500,&g24_test_result);
						DBG(RED"err: %d\n"NONE,err);
					}
					else if(recommand_band == 1)//5G
					{
						commuGetBrSpeedTestStatusRunLAN(3000,15,500,&g58_test_result);
					}
					DBG(YELLOW"g24_test_result.allRtt: %f\n"NONE,g24_test_result.allRtt);
					DBG(YELLOW"g24_test_result.minRtt: %f\n"NONE,g24_test_result.minRtt);
					DBG(YELLOW"g24_test_result.maxRtt: %f\n"NONE,g24_test_result.maxRtt);
					DBG(YELLOW"g24_test_result.averRtt: %f\n"NONE,g24_test_result.averRtt);
					DBG(YELLOW"g24_test_result.sends: %d\n"NONE,g24_test_result.sends);
					DBG(YELLOW"g24_test_result.losts: %d\n"NONE,g24_test_result.losts);
					DBG(YELLOW"g58_test_result.allRtt: %f\n"NONE,g58_test_result.allRtt);
					DBG(YELLOW"g58_test_result.minRtt: %f\n"NONE,g58_test_result.minRtt);
					DBG(YELLOW"g58_test_result.maxRtt: %f\n"NONE,g58_test_result.maxRtt);
					DBG(YELLOW"g58_test_result.averRtt: %f\n"NONE,g58_test_result.averRtt);
					DBG(YELLOW"g58_test_result.sends: %d\n"NONE,g58_test_result.sends);
					DBG(YELLOW"g58_test_result.losts: %d\n"NONE,g58_test_result.losts);
					buf[len++] = 0;// LAN speed test
					buf[len++] = 0;// reserved
					buf[len++] = (unsigned char)recommand_band;
					buf[len++] = (unsigned char)((int)g24_test_result.allRtt >>24);
					buf[len++] = (unsigned char)((int)g24_test_result.allRtt >>16);
					buf[len++] = (unsigned char)((int)g24_test_result.allRtt >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.allRtt);
					buf[len++] = (unsigned char)(((int)g24_test_result.minRtt) >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.minRtt);
					buf[len++] = (unsigned char)(((int)g24_test_result.maxRtt) >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.maxRtt);
					buf[len++] = (unsigned char)(((int)g24_test_result.averRtt) >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.averRtt);
					buf[len++] = (unsigned char)(((int)g24_test_result.sends) >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.sends);
					buf[len++] = (unsigned char)(((int)g24_test_result.losts) >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.losts);
					buf[len++] = (unsigned char)((int)g58_test_result.allRtt >>24);
					buf[len++] = (unsigned char)((int)g58_test_result.allRtt >>16);
					buf[len++] = (unsigned char)((int)g58_test_result.allRtt >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.allRtt);
					buf[len++] = (unsigned char)(((int)g58_test_result.minRtt) >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.minRtt);
					buf[len++] = (unsigned char)(((int)g58_test_result.maxRtt) >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.maxRtt);
					buf[len++] = (unsigned char)(((int)g58_test_result.averRtt) >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.averRtt);
					buf[len++] = (unsigned char)(((int)g58_test_result.sends) >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.sends);
					buf[len++] = (unsigned char)(((int)g58_test_result.losts) >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.losts);

					d.need_resend = 1;
					d.state = 0;
					d.seq_id = 0;
					d.product_type = CMDTYPE_WIFI_CONFIG_PARAM;
					d.cmd_id = CMD_SEND_SPEED_TEST_STATE;
					d.len = len;
					d.buff = buf;
					packDeviceCmd(&d);
					swl_get_list_ctl = SWL_NOP_WAIT;
					swl_send_list_ctl = SWL_NOP_WAIT;
					timer_cnt = 0;
					g_isSpeedTesting = FALSE;
				}break;
				case SWL_SPEED_TEST_WAN:
				{
					
				}break;
				case SWL_NOP_WAIT:
				{
					//DBG("swl_get_list_ctl: SWL_NOP_WAIT\n");
					timer_cnt = 0;
				}break;
				default:
				{
					DBG(RED" SET WIFI LOGIC: error!\n"NONE);
				}
			}

			switch(swl_send_list_ctl)
			{
				case SWL_SEND_ROUTE_LIST:
				{
					//DBG("swl_send_list_ctl: SWL_SEND_ROUTE_LIST\n");
					//if(swl_get_list_ctl == SWL_NOP_WAIT)
					{
						char buf[400] = {0};
						unsigned int len = 0;
						STRU_BLE_DATA d = {0};
						if(route_cnt != has_send_route_cnt)
						{
							unsigned int i = 0, j = 0, k = 0, has_send_route_cnt_tmp,route_len_sum;
							has_send_route_cnt_tmp = has_send_route_cnt;
							route_len_sum = 0;
							while(has_send_route_cnt != route_cnt)
							{
								route_len_sum += route_list[has_send_route_cnt].route_len + 5;
								has_send_route_cnt++;
								if(has_send_route_cnt >= MAX_ROUTE_LIST_NUM)
								{
									has_send_route_cnt = 0;
								}
								k++;
								if(route_len_sum >= 390)
								{
									k--;
									break;
								}
							}
							has_send_route_cnt = has_send_route_cnt_tmp;
							buf[0] = k;//k是本次要发送的路由器数�?
							k = 0;
							for(i = 1; k < buf[0]; k++)
							{
								
								while(j < route_list[has_send_route_cnt].ssid_len)
								{
									buf[i++] = route_list[has_send_route_cnt].ssid[j++];
								}
								buf[i++] = 0;
								j = 0;
								while(j < route_list[has_send_route_cnt].ch_len)
								{
									buf[i++] = route_list[has_send_route_cnt].ch[j++];
								}
								buf[i++] = 0;
								j = 0;
								while(j < route_list[has_send_route_cnt].security_len)
								{
									buf[i++] = route_list[has_send_route_cnt].security[j++];
								}
								buf[i++] = 0;
								j = 0;
								while(j < route_list[has_send_route_cnt].rssi_len)
								{
									buf[i++] = route_list[has_send_route_cnt].rssi[j++];
								}
								buf[i++] = 0;
								j = 0;
								while(j < route_list[has_send_route_cnt].bssid_len)
								{
									buf[i++] = route_list[has_send_route_cnt].bssid[j++];
								}
								buf[i++] = 0;
								j = 0;
								has_send_route_cnt++;
								if(has_send_route_cnt >= MAX_ROUTE_LIST_NUM)
								{
									has_send_route_cnt = 0;
								}
							}
							if(has_send_route_cnt == route_cnt)//全部发送完毕�?
							{
								buf[i++] = 1;
							}
							else//未发完�?
							{
								buf[i++] = 0;
							}
							len = i;
							d.need_resend = 1;
							d.state = 0;
							d.seq_id = 0;
							d.product_type = CMDTYPE_SET_WIFI;
							d.cmd_id = CMD_SEND_ROUTE_LIST;
							d.len = len;
							d.buff = buf;
							packDeviceCmd(&d);//这个函数需要加锁�?
							swl_send_list_ctl = SWL_NOP_WAIT;
							DBG("route list pack to send buffer\n");
						}
					}
				}break;
				case SWL_SPEED_TEST_LAN_SINGLE:
				{
					STRU_BLE_DATA d = {0};
					char buf[40] = {0};
					int recommand_band = 0, len = 0;
					T_MKOS_APCLI_STATUS_INFO g24status;
					T_MKOS_APCLI_STATUS_INFO g58status;
					double g24Speed = 0, g58Speed = 0;
					T_MKOS_APCLI_LAN_SPEEDTEST_INFO g24_test_result;
					T_MKOS_APCLI_LAN_SPEEDTEST_INFO g58_test_result;
										
					memset(&g24status,0,sizeof(g24status));
					memset(&g58status,0,sizeof(g58status));
					memset(&g24_test_result,0,sizeof(g24_test_result));
					memset(&g58_test_result,0,sizeof(g58_test_result));

					commuGetConnectApStatus(&g24status,&g58status);
					DBG(RED"g24status.status: %d g58status.status: %d\n"NONE,g24status.status,g58status.status);
					
					swl_send_list_ctl = SWL_NOP_WAIT;
					g_isSpeedTesting = TRUE;
					time(&t_start_speed_test);
										
					switch(set_wifi_type)
					{
						case 1:
						{
							//speed test
							commuGetSingleSpeedTestStatusSetupLAN(0,3000,15,500,&g24_test_result);
							//select upper net
							commuSelectAplientInterfaceToUse(0);
							recommand_band = 0;
						}break;
						case 2:
						{
							//speed test
							commuGetSingleSpeedTestStatusSetupLAN(1,3000,15,500,&g58_test_result);
							//select upper net
							commuSelectAplientInterfaceToUse(1);
							recommand_band = 1;
						}break;
						case 3:
						{
							//speed test
							if(g24status.status == WIFI_CONN_SUCCESS)
							{
								commuGetSingleSpeedTestStatusSetupLAN(0,3000,15,500,&g24_test_result);
								g24Speed = g24_test_result.sends/g24_test_result.allRtt;
							}
							
							if(g58status.status == WIFI_CONN_SUCCESS)
							{
								commuGetSingleSpeedTestStatusSetupLAN(1,3000,15,500,&g58_test_result);
								g58Speed = g58_test_result.sends/g58_test_result.allRtt;
							}
							time(&t_stop_speed_test);
							DBG(YELLOW"time speed test: %f\n"NONE,difftime(t_stop_speed_test,t_start_speed_test));
							//select upper net
							if((g24status.status == WIFI_CONN_SUCCESS)&&(g58status.status == WIFI_CONN_SUCCESS))
							{
								if(g58Speed >= g24Speed)
								{
									commuSelectAplientInterfaceToUse(1);
									recommand_band = 1;
								}
								else
								{
									commuSelectAplientInterfaceToUse(0);
									recommand_band = 0;
								}
								
							}
							else if(g58status.status == WIFI_CONN_SUCCESS)
							{
								commuSelectAplientInterfaceToUse(1);
								recommand_band = 1;
							}
							else if(g24status.status == WIFI_CONN_SUCCESS)
							{
								commuSelectAplientInterfaceToUse(0);
								recommand_band = 0;
							}
							else
							{
								//error, no upper net connected
								DBG(RED"error, no upper net connected"NONE);
							}
							time(&t_stop_select);
							DBG(YELLOW"time select: %f\n"NONE,difftime(t_stop_select,t_stop_speed_test));
						}break;
						default:break;
					}

					//save set to configured
					commuSetSystemConfig(1);
					DBG(YELLOW"commuSetSystemConfig(1)\n"NONE);
					g_duringSetWifiLogic = 0;
					time(&t_stop_save);
					DBG(YELLOW"time save: %f\n"NONE,difftime(t_stop_save,t_stop_select));
					DBG(YELLOW"time all: %f\n"NONE,difftime(t_stop_save,t_start_conn));

					DBG(YELLOW"g24_test_result.allRtt: %f\n"NONE,g24_test_result.allRtt);
					DBG(YELLOW"g24_test_result.minRtt: %f\n"NONE,g24_test_result.minRtt);
					DBG(YELLOW"g24_test_result.maxRtt: %f\n"NONE,g24_test_result.maxRtt);
					DBG(YELLOW"g24_test_result.averRtt: %f\n"NONE,g24_test_result.averRtt);
					DBG(YELLOW"g24_test_result.sends: %d\n"NONE,g24_test_result.sends);
					DBG(YELLOW"g24_test_result.losts: %d\n"NONE,g24_test_result.losts);
					DBG(YELLOW"g58_test_result.allRtt: %f\n"NONE,g58_test_result.allRtt);
					DBG(YELLOW"g58_test_result.minRtt: %f\n"NONE,g58_test_result.minRtt);
					DBG(YELLOW"g58_test_result.maxRtt: %f\n"NONE,g58_test_result.maxRtt);
					DBG(YELLOW"g58_test_result.averRtt: %f\n"NONE,g58_test_result.averRtt);
					DBG(YELLOW"g58_test_result.sends: %d\n"NONE,g58_test_result.sends);
					DBG(YELLOW"g58_test_result.losts: %d\n"NONE,g58_test_result.losts);
					buf[len++] = 0;// LAN speed test
					buf[len++] = 0;// reserved
					buf[len++] = (unsigned char)recommand_band;
					buf[len++] = (unsigned char)((int)g24_test_result.allRtt >>24);
					buf[len++] = (unsigned char)((int)g24_test_result.allRtt >>16);
					buf[len++] = (unsigned char)((int)g24_test_result.allRtt >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.allRtt);
					buf[len++] = (unsigned char)(((int)g24_test_result.minRtt) >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.minRtt);
					buf[len++] = (unsigned char)(((int)g24_test_result.maxRtt) >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.maxRtt);
					buf[len++] = (unsigned char)(((int)g24_test_result.averRtt) >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.averRtt);
					buf[len++] = (unsigned char)(((int)g24_test_result.sends) >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.sends);
					buf[len++] = (unsigned char)(((int)g24_test_result.losts) >>8);
					buf[len++] = (unsigned char)((int)g24_test_result.losts);
					buf[len++] = (unsigned char)((int)g58_test_result.allRtt >>24);
					buf[len++] = (unsigned char)((int)g58_test_result.allRtt >>16);
					buf[len++] = (unsigned char)((int)g58_test_result.allRtt >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.allRtt);
					buf[len++] = (unsigned char)(((int)g58_test_result.minRtt) >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.minRtt);
					buf[len++] = (unsigned char)(((int)g58_test_result.maxRtt) >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.maxRtt);
					buf[len++] = (unsigned char)(((int)g58_test_result.averRtt) >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.averRtt);
					buf[len++] = (unsigned char)(((int)g58_test_result.sends) >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.sends);
					buf[len++] = (unsigned char)(((int)g58_test_result.losts) >>8);
					buf[len++] = (unsigned char)((int)g58_test_result.losts);

					d.need_resend = 1;
					d.state = 0;
					d.seq_id = 0;
					d.product_type = CMDTYPE_WIFI_CONFIG_PARAM;
					d.cmd_id = CMD_SEND_SPEED_TEST_STATE;
					d.len = len;
					d.buff = buf;
					packDeviceCmd(&d);
					g_isSpeedTesting = FALSE;
				}break;
				case SWL_NOP_WAIT:
				{
					//DBG("swl_send_list_ctl: SWL_NOP_WAIT\n");
				}break;
				default:
				{
					DBG(" SET WIFI LOGIC: error!\n");
				}
			}
		}
		if(g_resend_cmd.has_cmd == 1)
		{
			g_resend_cmd.resend_to++;
			if(g_resend_cmd.resend_to >= 100)//2s
			{
				g_resend_cmd.resend_to = 0;
				g_resend_cmd.resend_cnt++;
				if(g_resend_cmd.resend_cnt <= 2)
				{
					DBG(L_RED"resend cmd: %d\n"NONE,g_resend_cmd.resend_cnt);
					send2UartBuf(&g_resend_cmd.cmd);
				}
				else
				{
					g_resend_cmd.has_cmd = 0;
					g_resend_cmd.is_cmd_changed = 0;
					g_resend_cmd.resend_cnt = 0;
					g_resend_cmd.resend_to = 0;
					DBG(L_RED"resend cmd over\n"NONE);
				}
			}
		}
		else
		{
			resend_to = 0;
			
		}
		usleep(1000 * 20);
		timer_cnt++;
	}
	pthread_exit(NULL);	
	return;
}

void *montorUartLoopThread()
{
	
	int localreadflag=-1;
	char localUartreadBuf[1024]={0};
	int uartReadCount=0;
	char tmpLocalreadLen=0;
	char tmpLocalReadBuf[512]={0};
	char s[200] = {0};
	unsigned int i = 0;
	
	
	
	DBG("Creat thread_ montor uart loop ok\n");
	while(1)
	{
		sleep(1);
		if(0 != get_usb_status())
		{
			uartbt_state=UARTBT_DOWNLINE;
			continue;				
		}
		else
		{
			uartbt_state=UARTBT_IDLE;
		}
		
		if(uartbt_state != UARTBT_IDLE)
		{
			continue;
		}
		
		while(1)
		{
			memset(localUartreadBuf,0,1024);
			#if 0
			if(uartReadCount > 8)
			{
				break;
			}
			#endif
			localreadflag = read(uart_fd, localUartreadBuf, 1024);
			if(localreadflag == 0)
			{
				uartReadCount++;
				
				//DBG("montor uart read 0 count=%d\n",uartReadCount);
				//continue;
			}
			else if(localreadflag > 0)
			{
				//DBG("montor uart read %s\n",localUartreadBuf);

				while((tmpLocalreadLen = read(uart_fd,tmpLocalReadBuf,512))>0)
				{
					
					memcpy(localUartreadBuf+localreadflag,tmpLocalReadBuf,tmpLocalreadLen);
					localreadflag+=tmpLocalreadLen;
					memset(tmpLocalReadBuf,0,512);
					
				}

				//localUartreadBuf  is the uart content , localreadflag is the size
				for(i = 0; i < localreadflag; i++)
				{
					uartRecvBuf[p_in] = localUartreadBuf[i];
					p_in = ptr_plus(p_in,1);
				}
				hex2str(localUartreadBuf,localreadflag,s);
				DBG(PURPLE"loop montor uart read %s\n"NONE,s);
				memset(s,0,200);

			}
			else
			{
				uartReadCount++;
				//continue;
			}
			usleep(1000 * 20);
		}		
		uartbt_state=UARTBT_IDLE;
		continue;
	}
	pthread_exit(NULL);
}

void *cacheRouteListThread()
{
	DBG("Create cacheRouteListThread ok\n");
	int ret = 0;
	unsigned char i = 0, j = 0;

	mkos_aplist_init();
	
	while(1)
	{
		ret = commuGetSystemConfig();
		if((ret == 1)||(g_duringSetWifiLogic == 1))// have configured or during configure logic
		{
			sleep(3);
		}
		else
		{
			DBG(YELLOW"cache route list\n"NONE);
			char buf[8192] = {0};

			j++;
			startScan();
			sleep(8);
			getApList(buf);
			pthread_mutex_lock(&mut);
			if(j == 1)
			{
				praseRouteList(buf,cache_route_list2[i],&cache_route_list_tmp_cnt2[i]);
			}
			else
			{
				cache_route_list_cnt2[i] = cache_route_list_tmp_cnt2[i];
				praseRouteList(buf,cache_route_list2[i],&cache_route_list_cnt2[i]);
			}
			
			cache_route_list_status2[i] = j;
			
			if(i)
			{
				cache_route_list_status2[0] = 0;
				cache_route_list_tmp_cnt2[0] = 0;
				cache_route_list_cnt2[0] = 0;
			}
			else
			{
				cache_route_list_status2[1] = 0;
				cache_route_list_tmp_cnt2[1] = 0;
				cache_route_list_cnt2[1] = 0;
			}
			pthread_mutex_unlock(&mut);
			//sleep(3);

			
			if(j>= 2)//每个缓存放2次
			{
				j = 0;
				i++;
				if(i >= 2)
				{
					i = 0;
				}
				mkos_aplist_init();//清lib使用的buffer
			}
				
			/*for(; i < 2; i++)
			{
				DBG(YELLOW"cache route list\n"NONE);

				int ret =0;
				char buf[8192]={0};//{"0xabcE9998CE4B88A-5G\n157\nWPA2/AES\n-68\n112233445C\n\0"};//

				ret = commuGetSystemConfig();
				if((ret == 1)||(g_duringSetWifiLogic == 1))// have configured or during configure logic
				{
					break;
				}
				commuStartScanAp();
				ret = commuGetApSiteList(buf);// in this function already have sleep(3);
				pthread_mutex_lock(&mut);
				praseRouteList(buf,cache_route_list2[i],&cache_route_list_tmp_cnt2[i]);
				cache_route_list_status2[i] = 1;
				cache_route_list_cnt2[i] += cache_route_list_tmp_cnt2[i];
				pthread_mutex_unlock(&mut);

				memset(buf,0,sizeof(buf));
				sleep(8);
				ret = commuGetApSiteListSecond(buf);
				pthread_mutex_lock(&mut);
				praseRouteList(buf,cache_route_list2[i],&cache_route_list_cnt2[i]);
				cache_route_list_status2[i] = 2;
				
				if(i)
				{
					cache_route_list_status2[0] = 0;
					cache_route_list_tmp_cnt2[0] = 0;
					cache_route_list_cnt2[0] = 0;
				}
				else
				{
					cache_route_list_status2[1] = 0;
					cache_route_list_tmp_cnt2[1] = 0;
					cache_route_list_cnt2[1] = 0;
				}
				pthread_mutex_unlock(&mut);
				sleep(30);
			}
			if(i >= 2)
			{
				i = 0;
			}*/
			
		}
	}
	pthread_exit(NULL);
}

int main(int argc, char *argv[])  
{
/*
	int i=0;
	int readByte=0;
	int btot=0;
	int waitCommandCount=0;
	
  	unsigned char mainBuf[BUFFER_SIZE]={0};
	unsigned char readBuf[BUFFER_SIZE]={0};
	int comSendPacketLen=0;
	unsigned char comSendPacketBuf[1024]={0};
	char commandBuf[1024]={0};
	char strFileName[64]={0};
	char strFileSize[64]={0};
	char strInfoHeader[64]={0};
	int intFileSize=0;
	int intRecFileEndFlag=0;
	
	
	unsigned char tmpBuf[512]={0};
	char *tmpBp=NULL;

	int buff_send_ack=0;
	char *pBuff=NULL;
	int readByteCnt=0;
	int rev_status =0;

	int mainServerSocketTimeoutCount=0;

	struct stat sdcaeddir;
	*/


	pthread_mutex_init(&mut,NULL);
	memset(&thread, 0, sizeof(thread));
/*
	if(get_nvram_para_all() == 0)
	{
		printf("\r\n get nvram para error so exit process!!!");
		return 0;
	}
	*/
	if(argc == 2 && !strcmp(argv[1],"test"))
	{
		DBG("for test so disable ");

		int ret =0;
		char aplistarr[8192]={0};
		uartbt_state=UARTBT_IDLE;

		commuStartScanAp();
		sleep(3);
		ret = commuGetApSiteList(aplistarr);
		printf("\r\n %d--value=%s^^^^^\r\n",ret,aplistarr);

		memset(aplistarr,0,sizeof(aplistarr));
		sleep(8);
		ret = commuGetApSiteListSecond(aplistarr);
		printf("\r\n %d--value2=%s^^^^^\r\n",ret,aplistarr);
		return 0;
	}
	else if(argc >= 5 && !strcmp(argv[1],"connect"))
	{
		DBG("for test to conect ");

		int ret =0;
		
		/*T_MKOS_APCLI_PARA testg24 = 
			{
				.ssid = "TP_LINK_3019",
				.ch = "11",
				.security = "WPA1PSKWPA2PSK/TKIPAES",
				.key = "nicaiba!",
				.timeout = 5
			};
		T_MKOS_APCLI_PARA testg58 = 
			{
				.ssid = "TP_LINK_3019_5G",
				.ch = "157",
				.security = "WPA1PSKWPA2PSK/TKIPAES",
				.key = "nicaiba!",
				.timeout = 5
			};*/

		T_MKOS_APCLI_PARA testg24;
				
		uartbt_state=UARTBT_IDLE;

		strcpy(testg24.ssid,argv[2]);
		strcpy(testg24.ch,argv[3]);
		strcpy(testg24.security,argv[4]);
		strcpy(testg24.key,argv[5]);
		testg24.timeout = 5;
		commuConnectAp(&testg24,NULL);
		commuSaveConnectApInfo(&testg24,NULL);
		return 0;
	}
	else if(argc == 2 && !strcmp(argv[1],"constatus"))
	{
		DBG("for show conect status\r\n");
		T_MKOS_APCLI_STATUS_INFO g24status;
		T_MKOS_APCLI_STATUS_INFO g58status;
		memset(&g24status,0,sizeof(g24status));
		memset(&g58status,0,sizeof(g58status));

		commuGetConnectApStatus(&g24status,&g58status);

		if(g24status.is_enable)
		{
			DBG("2.4g CLI is Enable:\r\n");
			if(g24status.status == -9)
			{
				DBG("2.4g connect is stop\r\n");
			}
			else if(g24status.status == 0)
			{
				DBG("2.4g is connecting\r\n");
			}
			else if(g24status.status == 1)
			{
				DBG("2.4g is connected ok\r\n");
			}
			else
			{
				DBG("2.4g not defined code\r\n");
			}
			DBG("2.4g connect ssid =%s\r\n",g24status.ssid);
			DBG("2.4g connect chan =%s\r\n",g24status.ch);
			DBG("2.4g connect ip =%s\r\n",g24status.ip);
			DBG("2.4g connect mask =%s\r\n",g24status.nm);
		}
		else
		{
			DBG("2.4g CLI is Disabled:\r\n");
		}


		if(g58status.is_enable)
		{
			DBG("5.8g CLI is Enable:\r\n");
			if(g58status.status == -9)
			{
				DBG("5.8g connect is stop\r\n");
			}
			else if(g58status.status == 0)
			{
				DBG("5.8g is connecting\r\n");
			}
			else if(g58status.status == 1)
			{
				DBG("5.8g is connected ok\r\n");
			}
			else
			{
				DBG("5.8g not defined code\r\n");
			}
			DBG("5.8g connect ssid =%s\n",g58status.ssid);
			DBG("5.8g connect chan =%s\n",g58status.ch);
			DBG("5.8g connect ip =%s\n",g58status.ip);
			DBG("5.8g connect mask =%s\n",g58status.nm);
		}
		else
		{
			DBG("5.8g CLI is Disabled:\r\n");
		}
		return 0;
	}
	else if(argc == 2 && !strcmp(argv[1],"stop"))
	{
		DBG("for stop connect\n");

		int ret =0;
		
		uartbt_state=UARTBT_IDLE;

		T_MKOS_APCLI_STATUS_INFO g24status;
		T_MKOS_APCLI_STATUS_INFO g58status;
		memset(&g24status,0,sizeof(g24status));
		memset(&g58status,0,sizeof(g58status));

		commuGetConnectApStatus(&g24status,&g58status);

		if(g24status.is_enable)
		{
			DBG("2.4g CLI is Enable:\r\n");
			if(g24status.status == -9)
			{
				DBG("2.4g connect is stop\r\n");
			}
			else if(g24status.status == 0)
			{
				DBG("2.4g is connecting\r\n");
			}
			else if(g24status.status == 1)
			{
				DBG("2.4g is connected ok\r\n");
			}
			else
			{
				DBG("2.4g not defined code\r\n");
			}
			DBG("2.4g connect ssid =%s\r\n",g24status.ssid);
			DBG("2.4g connect chan =%s\r\n",g24status.ch);
			DBG("2.4g connect ip =%s\r\n",g24status.ip);
			DBG("2.4g connect mask =%s\r\n",g24status.nm);
		}
		else
		{
			DBG("2.4g CLI is Disabled:\r\n");
		}


		if(g58status.is_enable)
		{
			DBG("5.8g CLI is Enable:\r\n");
			if(g58status.status == -9)
			{
				DBG("5.8g connect is stop\r\n");
			}
			else if(g58status.status == 0)
			{
				DBG("5.8g is connecting\r\n");
			}
			else if(g58status.status == 1)
			{
				DBG("5.8g is connected ok\r\n");
			}
			else
			{
				DBG("5.8g not defined code\r\n");
			}
			DBG("5.8g connect ssid =%s\n",g58status.ssid);
			DBG("5.8g connect chan =%s\n",g58status.ch);
			DBG("5.8g connect ip =%s\n",g58status.ip);
			DBG("5.8g connect mask =%s\n",g58status.nm);
		}
		else
		{
			DBG("5.8g CLI is Disabled:\r\n");
		}
		commuStopConnectAp(2);

		sleep(2);

		DBG("after stop connect ");


		memset(&g24status,0,sizeof(g24status));
		memset(&g58status,0,sizeof(g58status));

		commuGetConnectApStatus(&g24status,&g58status);

		if(g24status.is_enable)
		{
			DBG("2.4g CLI is Enable:\r\n");
			if(g24status.status == -9)
			{
				DBG("2.4g connect is stop\r\n");
			}
			else if(g24status.status == 0)
			{
				DBG("2.4g is connecting\r\n");
			}
			else if(g24status.status == 1)
			{
				DBG("2.4g is connected ok\r\n");
			}
			else
			{
				DBG("2.4g not defined code\r\n");
			}
			DBG("2.4g connect ssid =%s\r\n",g24status.ssid);
			DBG("2.4g connect chan =%s\r\n",g24status.ch);
			DBG("2.4g connect ip =%s\r\n",g24status.ip);
			DBG("2.4g connect mask =%s\r\n",g24status.nm);
		}
		else
		{
			DBG("2.4g CLI is Disabled:\r\n");
		}


		if(g58status.is_enable)
		{
			DBG("5.8g CLI is Enable:\r\n");
			if(g58status.status == -9)
			{
				DBG("5.8g connect is stop\r\n");
			}
			else if(g58status.status == 0)
			{
				DBG("5.8g is connecting\r\n");
			}
			else if(g58status.status == 1)
			{
				DBG("5.8g is connected ok\r\n");
			}
			else
			{
				DBG("5.8g not defined code\r\n");
			}
			DBG("5.8g connect ssid =%s\n",g58status.ssid);
			DBG("5.8g connect chan =%s\n",g58status.ch);
			DBG("5.8g connect ip =%s\n",g58status.ip);
			DBG("5.8g connect mask =%s\n",g58status.nm);
		}
		else
		{
			DBG("5.8g CLI is Disabled:\r\n");
		}
		return 0;
	}
	else if(argc == 6 && !strcmp(argv[1],"extap2g"))
	{
		DBG("set 2.4g ext ap\n");

		T_MKOS_APCLI_PARA testg24 = 
					{
						.ssid = "hou dong fang",
						.ch = "11",
						.security = "WPA1PSKWPA2PSK/TKIPAES",
						.key = "12345678",
						.timeout = 5
					};

		int ret =0;
		char ssid[64]={0},security[32],pass[64],ch[8];
		
		uartbt_state=UARTBT_IDLE;

		strcpy(ssid,argv[2]);
		strcpy(security,argv[3]);
		strcpy(pass,argv[4]);
		strcpy(ch,argv[5]);
		commuSetExtAp(0,"1","0",ssid,security,pass,ch);
		return 0;
	}
	else if(argc == 6 && !strcmp(argv[1],"extap5g"))
	{
		DBG("set 5.8g ext ap\n");

		int ret =0;
		char ssid[64]={0},security[32],pass[64],ch[8];
		
		uartbt_state=UARTBT_IDLE;

		strcpy(ssid,argv[2]);
		strcpy(security,argv[3]);
		strcpy(pass,argv[4]);
		strcpy(ch,argv[5]);
		commuSetExtAp(1,"1","0",ssid,security,pass,ch);
		return 0;
	}
	else if(argc == 6 && !strcmp(argv[1],"extapall"))
	{
		DBG("set 2.4g and 5.8g ext ap\n");

		int ret =0;
		char ssid[64]={0},security[32],pass[64],ch[8];
		char newssid[64]={0};
		
		uartbt_state=UARTBT_IDLE;

		strcpy(ssid,argv[2]);
		strcpy(security,argv[3]);
		strcpy(pass,argv[4]);
		strcpy(ch,argv[5]);
		sprintf(newssid,"%s-24g",ssid);
		commuSetExtAp(0,"1","0",newssid,security,pass,NULL);
		sprintf(newssid,"%s-58g",ssid);
		commuSetExtAp(1,"1","0",newssid,security,pass,NULL);
		return 0;
	}
	else if(argc == 3 && !strcmp(argv[1],"useapcli"))
	{
		DBG("select 2.4g or 5.8g apcli to sue\n");

		int ret =0;
		int mode =0;
		
		uartbt_state=UARTBT_IDLE;

		mode = atoi(argv[2]);
		commuSelectAplientInterfaceToUse(mode);
		return 0;
	}
	else if(argc == 2 && !strcmp(argv[1],"getextap"))
	{
		DBG("get 2.4g and 5.8g ext ap\n");

		int ret =0;
		char ssid[64]={0},security[32]={0},pass[64]={0},ch[8]={0},en[8]={0},hidden[8]={0};
		char newssid[64]={0};
		
		uartbt_state=UARTBT_IDLE;

		commuGetExtAp(0,en,hidden,ssid,security,pass,ch);
		printf("2.4g ext ap info:\nen=%s,hidden=%s,ssid=%s,sec=%s,pass=%s,channel=%s\n",en,hidden,ssid,security,pass,ch);
		memset(en,0,sizeof(en));
		memset(hidden,0,sizeof(hidden));
		memset(ssid,0,sizeof(ssid));
		memset(security,0,sizeof(security));
		memset(pass,0,sizeof(pass));
		memset(ch,0,sizeof(ch));
		commuGetExtAp(1,en,hidden,ssid,security,pass,ch);
		printf("5.8g ext ap info:\nen=%s,hidden=%s,ssid=%s,sec=%s,pass=%s,channel=%s\n",en,hidden,ssid,security,pass,ch);
		return 0;
	}
	
	else if(argc == 2 && !strcmp(argv[1],"apcliinfo"))
	{
		DBG("\nfor show ap cli para info\n");
		T_MKOS_APCLI_PARA g24para;
		T_MKOS_APCLI_PARA g58para;
		memset(&g24para,0,sizeof(g24para));
		memset(&g58para,0,sizeof(g58para));
		commuGetConnectApInfo(&g24para,&g58para);
		printf("apcli para 2.4g : enable=%s,ssid=%s,key=%s\n",g24para.enable,g24para.ssid,g24para.key);
		printf("apcli para 5.8g : enable=%s,ssid=%s,key=%s\n",g58para.enable,g58para.ssid,g58para.key);
		return 0;
	}
	else if(argc == 4 && !strcmp(argv[1],"addmacname"))
	{
		printf("add mac name entry:\n");
		commuAddMacNameEntry(argv[2],argv[3]);
		return 0;
	}
	else if(argc == 2 && !strcmp(argv[1],"showmacname"))
	{
		char buflist[512]={0};
		printf("show mac name list:\n");
		commuMacNameInfoList(buflist);
		printf("%s\n",buflist);
		return 0;
	}
	else if(argc == 5 && !strcmp(argv[1],"runlantest"))
	{
		printf("run speed test lan:\n");
		T_MKOS_APCLI_LAN_SPEEDTEST_INFO result;
		
		commuGetBrSpeedTestStatusRunLAN(atoi(argv[2]),atoi(argv[3]),atoi(argv[4]),&result);
		
		printf("result: %d--%d--%f--%f--%f--%f--%d\n",result.sends,result.losts,result.maxRtt,result.minRtt,result.averRtt,result.allRtt,result.rssi);
		return 0;
	}
	else if(argc == 6 && !strcmp(argv[1],"singletest"))
	{
		printf("single speed test lan:\n");
		T_MKOS_APCLI_LAN_SPEEDTEST_INFO result;
		
		commuGetSingleSpeedTestStatusSetupLAN(atoi(argv[2]),atoi(argv[3]),atoi(argv[4]),atoi(argv[5]),&result);
		
		printf("result: %d--%d--%f--%f--%f--%f--%d\n",result.sends,result.losts,result.maxRtt,result.minRtt,result.averRtt,result.allRtt,result.rssi);
		return 0;
	}
	else if(argc == 2 && !strcmp(argv[1],"testled"))
	{
		DBG(YELLOW"commuSetLedControl() test\n"NONE);
		//during config, set led blink
		int ret = 0;
		// 白色
		// off all led
		ret = commuSetLedControl(LED2,0,1);
		ret = commuSetLedControl(LED3,0,1);
		ret = commuSetLedControl(LED4,0,1);
		// blink
		ret = commuSetLedControl(LED2,5,5);
		ret = commuSetLedControl(LED3,5,5);
		ret = commuSetLedControl(LED4,5,5);
		sleep(5);
		// 红色
		// off all led
		ret = commuSetLedControl(LED2,0,1);
		ret = commuSetLedControl(LED3,0,1);
		ret = commuSetLedControl(LED4,0,1);
		// blink
		ret = commuSetLedControl(LED2,5,5);
		ret = commuSetLedControl(LED3,0,5);
		ret = commuSetLedControl(LED4,0,5);
		sleep(5);
		// 蓝色
		// off all led
		ret = commuSetLedControl(LED2,0,1);
		ret = commuSetLedControl(LED3,0,1);
		ret = commuSetLedControl(LED4,0,1);
		// blink
		ret = commuSetLedControl(LED2,0,5);
		ret = commuSetLedControl(LED3,5,5);
		ret = commuSetLedControl(LED4,0,5);
		sleep(5);
		// 绿色
		// off all led
		ret = commuSetLedControl(LED2,0,1);
		ret = commuSetLedControl(LED3,0,1);
		ret = commuSetLedControl(LED4,0,1);
		// blink
		ret = commuSetLedControl(LED2,0,5);
		ret = commuSetLedControl(LED3,0,5);
		ret = commuSetLedControl(LED4,5,5);
		sleep(5);
		// 黄色
		// off all led
		ret = commuSetLedControl(LED2,0,1);
		ret = commuSetLedControl(LED3,0,1);
		ret = commuSetLedControl(LED4,0,1);
		// blink
		ret = commuSetLedControl(LED2,5,5);
		ret = commuSetLedControl(LED3,5,5);
		ret = commuSetLedControl(LED4,0,5);
		sleep(5);
		
		if(ret != 0)
		{
			DBG(RED"commuSetLedControl() error\n"NONE);
		}
		return 0;
	}
	else
	{
		while(1)
		{
			if(init_usb_uart() == 0)
			{
				uartbt_state=UARTBT_IDLE;
				break;
			}
			sleep(1);
		}
	}

	// start local tcp server thread
	while(1)
	{
		if(pthread_create(&localServerThread, NULL, setWifiLogicThread, NULL) == 0)
		{
			break;
		}
		else
		{
			 DBG("Creat thread_local_server erro\n");
		}
		sleep(2);
	}

	// start montor uart  loop
	while(1)
	{
		if(pthread_create(&uartbtMontorThread, NULL, montorUartLoopThread, NULL) == 0)
		{
			break;
		}
		else
		{
			 DBG("Creat montorUartLoopThread erro\n");
		}
		sleep(2);
	}
	
	// start route list cache thread
	while(1)
	{
		if(pthread_create(&routeListCacheThread, NULL, cacheRouteListThread, NULL) == 0)
		{
			break;
		}
		else
		{
			 DBG("Creat cacheRouteListThread erro\n");
		}
		sleep(2);
	}
	
	/*主循�?*/
	while(1)
	{

		//pause();
		//sleep(2);

		//for example, write uart
		
		int localwriteflag=-1;
		char pBuff[32]={0};
		int readByteCnt = 0;
		
		//printer_state=PRINTER_BUSY;
		while(1)
		{
			//strcpy(pBuff,"testtouartwrite!!");
			//readByteCnt = sizeof(pBuff);
			//localwriteflag=write(uart_fd, pBuff,readByteCnt);
			//DBG("send data: %s\n",pBuff);
			
			if(ps_out != ps_in)
			{
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
				localwriteflag = 0;
				localwriteflag = write(uart_fd, buf, tmp);
				hex2str(buf,tmp,bufstr);
				DBG(CYAN"%s\n"NONE,bufstr);
				//usleep(1000 * 20);
				if(localwriteflag >= 0)
				{
					ps_out = ptr_plus(ps_out,localwriteflag);
				}
				else
				{
					DBG(RED"uart send error: %d"NONE,errno);
				}
				continue;
		
			}
			
			usleep(1000 * 20);
			
		}
		//printer_state=PRINTER_IDLE;
		
	}
	
	uart_close(uart_fd);  
	return TRUE;  
}  

