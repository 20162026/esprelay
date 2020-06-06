#include "mdf_common.h" //pagrindinė ESPMDF biblioteka
#include "sdkconfig.h"	//ESPMDF nustatymai
//libesphttpd ir ESPFS bibliotekos
//#include <libesphttpd/esp.h>
#include "libesphttpd/httpd.h"
#include "espfs.h"
#include "espfs_image.h"
#include "libesphttpd/httpd-espfs.h"
#include "libesphttpd/cgiflash.h"
#include "libesphttpd/auth.h"
#include "libesphttpd/captdns.h"
#include "libesphttpd/httpd-freertos.h"
#include "libesphttpd/route.h"

//papildomos periferijos ir FreeRTOS
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "freertos/event_groups.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"


char my_hostname[16] = "esphttpd";
#define DEFAULT_WIFI_STA_SSID "HUAWEI-579E"
#define DEFAULT_WIFI_STA_PASS "dder9546"
static const char *TAG = "test_settings";

//nvs settings~~~~~~~~~~~~~~~
static struct nvsdata_t {
    char topic[31];
    char mesh_id[7];
    char mesh_password[65];
    char router_ssid[33];
    char router_password[65];
	char AP_ssid[33];
    char AP_password[65];
	char conf_password[33];
	char mqtturl[65];
	uint8_t wifimode;
	uint8_t debug;
}nvsdata;

//struct nvsdata_t nvsdata;
nvs_handle data_nvs_handle;
#define NVS_NAMESPACE "nvs"
#define NVS_DATA_KEY "nvsdata"

//web settings~~~~~~~~~~~~~~~
#define LISTEN_PORT     443u
#define MAX_CONNECTIONS 16u
static char connectionMemory[sizeof(RtosConnType) * MAX_CONNECTIONS];
static HttpdFreertosInstance httpdFreertosInstance;

//ota settings~~~~~~~~~~~~~~~
#define OTA_FLASH_SIZE_K 3840
#define OTA_TAGNAME "generic"
CgiUploadFlashDef uploadParams={
	.type=CGIFLASH_TYPE_FW,
	.fw1Pos=0x10000,
	.fw2Pos=((OTA_FLASH_SIZE_K*1024)/2)+0x10000,
	.fwSize=((OTA_FLASH_SIZE_K*1024)/2)-0x10000,
	.tagName=OTA_TAGNAME
};

///nvs functions~~~~~~~~~~~~~~~
static void SaveNvsData(){ //išsaugo struktūrą į NVS atmintį
	mdf_err_t err;
err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &data_nvs_handle);
if (err != MDF_OK)
	{
		MDF_LOGE( "Error (%d) opening NVS handle!", err);
	}
	else
	{
		MDF_LOGW( "saving nvs data");
		err = nvs_set_blob(data_nvs_handle, NVS_DATA_KEY, &nvsdata, sizeof(nvsdata));
		if(!err)
	 	{
	 		MDF_LOGI( "nvs commit");
			MDF_ERROR_ASSERT(nvs_commit(data_nvs_handle));
		}
		else MDF_LOGE( "Error (%d) reading NVS!", err);
	}

nvs_close(data_nvs_handle);

}

static void ReadNvsData(){	//perskaito NVS atmintyje išsaugotą nustatymų struktūrą
mdf_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &data_nvs_handle);

    if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Error (%s) opening NVS handle!\n", esp_err_to_name(err));
	}
	else
	{
		size_t nvssize=sizeof(struct nvsdata_t);
		// Read NVS
		MDF_LOGI("Reading network initialization from NVS");
		err = nvs_get_blob(data_nvs_handle, NVS_DATA_KEY, &nvsdata, &nvssize);
		MDF_LOGW("reading nvs size: %d", nvssize);
		if(err!=MDF_OK||nvssize!=sizeof(struct nvsdata_t)){
			MDF_LOGW("nvs init not found or corrupted, loading default values. error:0x%X",err);
			strcpy( nvsdata.router_ssid,CONFIG_ROUTER_SSID);
            strcpy( nvsdata.router_password,CONFIG_ROUTER_PASSWORD);
            strcpy( nvsdata.mesh_id,CONFIG_MESH_ID);
            strcpy( nvsdata.mesh_password,CONFIG_MESH_PASSWORD);
			strcpy(nvsdata.mqtturl,CONFIG_MQTT_URL);
            strcpy(nvsdata.AP_ssid,"ESPtest");
            strcpy(nvsdata.AP_password,"esptest1");
            strcpy(nvsdata.topic,"gBridge/u1/");
			strcpy(nvsdata.conf_password,"esptest");
			nvsdata.wifimode=0;
			nvsdata.debug=4;
		}
	}

    nvs_close(data_nvs_handle);

}

///web functions~~~~~~~~~~~~~~~
int myPassFn(HttpdConnData *connData, int no, char *user, int userLen, char *pass, int passLen) {//HTML LOGIN
	if (no==0) {
		strcpy(user, "admin");
		strcpy(pass, nvsdata.conf_password);
		return 1;
	}
	return 0;
}

static void customHeaders_cacheForever(HttpdConnData *connData)//statiniai css/js failai
{
	httpdHeader(connData, "Cache-Control", "max-age=365000000, public, immutable");
	httpdHeader(connData, "Content-Type", httpdGetMimetype(connData->url));
	MDF_LOGV( "customHeaders_cacheForever");
}
HttpdCgiExArg CgiOptionsEspfsStatic = {
    .headerCb = &customHeaders_cacheForever,
	.basepath = "/static/"
};

CgiStatus ICACHE_FLASH_ATTR cgiSETwifi(HttpdConnData *connData) {//wifi WEB nustatymai
	int len;
	int len2;
	char buff[65];
	char buff2[65];
	uint8_t change=0;
	if (connData->isConnectionClosed) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}
	else{
		len = httpdFindArg(connData->post.buff, "essid", buff, sizeof(buff));
		if(len > 1){
			len2 = httpdFindArg(connData->post.buff, "passwd", buff2, sizeof(buff2));
			if(len2 > 1){
				if(len<33&&len2<65){//reiketu patikrinti ar visi char tinkami
					change=1;
					MDF_LOGI( "ssid: %s size: %d ",buff,len-1);
					MDF_LOGI( "psw: %s size: %d",buff2,len2-1);
					strcpy(nvsdata.router_ssid,buff);
					strcpy(nvsdata.router_password,buff2);
				}
			}
   		}else {
			len = httpdFindArg(connData->post.buff, "apssid", buff, sizeof(buff));
			if(len > 1){
				len2 = httpdFindArg(connData->post.buff, "appasswd", buff2, sizeof(buff2));
				if(len2 > 1){
					if(len<33&&len2>7&&len2<65){//reiketu patikrinti ar visi char tinkami
						change=1;
						MDF_LOGI( "AP_ssid: %s size: %d ",buff,len-1);
						MDF_LOGI( "AP_psw: %s size: %d ",buff2,len2-1);
						strcpy(nvsdata.AP_ssid,buff);
						strcpy(nvsdata.AP_password,buff2);
					}  
				}
			}else{
				len = httpdFindArg(connData->post.buff, "mode", buff, sizeof(buff));
				if(len > 1){
					change=1;
					if(!strcmp(buff,"STAmode")){
						nvsdata.wifimode=1;
						MDF_LOGI("wifi mode: %s size %d",buff,len-1);
					}else nvsdata.wifimode=0;
				} 
			}
					 
		}
	}
	if(change)SaveNvsData();
	httpdRedirect(connData, "./settings.tpl");
	return HTTPD_CGI_DONE;
}

CgiStatus ICACHE_FLASH_ATTR cgiSETmesh(HttpdConnData *connData) {//ESPmesh WEB nustatymai
	int len;
	int len2;
	char buff[65];
	char buff2[65];
	uint8_t change=0;
	if (connData->isConnectionClosed) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}

	len = httpdFindArg(connData->post.buff, "topic", buff, sizeof(buff));
    if(len  > 1){
		if(len<32){//reiketu patikrinti ar visi char tinkami
			change=1;
			strcpy(nvsdata.topic,buff);
			MDF_LOGI( "topic %s size %d",buff,len-1);
		}
    }else{ 
		len = httpdFindArg(connData->post.buff, "mesh_id", buff, sizeof(buff));
		if(len > 1){
			len2 = httpdFindArg(connData->post.buff, "mesh_password", buff2, sizeof(buff2));
			if(len2  > 1){
				if(len==7&&len2<65){//reiketu patikrinti ar visi char tinkami
					change=1;
					MDF_LOGI( "MESH_ID: %s size: %d ",buff,len-1);
					MDF_LOGI( "MESH_PW: %s size: %d ",buff2,len2-1);
					strcpy(nvsdata.mesh_id,buff);
					strcpy(nvsdata.mesh_password,buff2);
				}
			}
		}else{ 
			len = httpdFindArg(connData->post.buff, "mqtturl", buff, sizeof(buff));
			if(len > 1){
				change=1;
				MDF_LOGI("mqtt url: %s size: %d ",buff,len-1);
				strcpy(nvsdata.mqtturl,buff);
			}
		}
	}
		 
	if(change)SaveNvsData();
	httpdRedirect(connData, "./settings.tpl");
	return HTTPD_CGI_DONE;
}

CgiStatus ICACHE_FLASH_ATTR cgiSET(HttpdConnData *connData) {//bendri WEB nustatymai
	int len;
	char buff[65];
	uint8_t change=0;
	if (connData->isConnectionClosed) {
		//Connection aborted. Clean up.
		return HTTPD_CGI_DONE;
	}
	len = httpdFindArg(connData->post.buff, "conf_password", buff, sizeof(buff));
	if(len > 1){
		change=1;
		MDF_LOGI( "conf_password: %s size: %d ",buff,len-1);
		strcpy(nvsdata.conf_password,buff);
	}else{
		len = httpdFindArg(connData->post.buff, "debugmode", buff, sizeof(buff));
		if(len > 1&&(buff[0]-'0')<7){
			change=1;
			MDF_LOGI( "debugmode: %s size: %d ",buff,len-1);
			nvsdata.debug=buff[0]-'0';
		}
	}

	if(change)SaveNvsData();
	httpdRedirect(connData, "./settings.tpl");
	return HTTPD_CGI_DONE;
}

CgiStatus ICACHE_FLASH_ATTR tplSET(HttpdConnData *connData, char *token, void **arg) {//WEB dinaminės vertės
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;

	strcpy(buff, "UnKnOwN");
	if (strcmp(token, "wifissid")==0) {
		sprintf(buff, nvsdata.router_ssid);
	}else
		if (strcmp(token, "APssid")==0) {
		sprintf(buff, nvsdata.AP_ssid);
	} else
		if (strcmp(token, "topic")==0) {
		sprintf(buff, nvsdata.topic);
	}else
		if (strcmp(token, "meshid")==0) {
		sprintf(buff, nvsdata.mesh_id);
	}else
		if (strcmp(token, "mqtturl")==0) {
		sprintf(buff, nvsdata.mqtturl);
	}else
		if (strcmp(token, "memdebug")==0) {
		buff[0]= nvsdata.debug+'0';
		buff[1]='\0';
	}
	

	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}

CgiStatus ICACHE_FLASH_ATTR tplSETindex(HttpdConnData *connData, char *token, void **arg) {// WEB dinaminės vertės
	char buff[128];
	if (token==NULL) return HTTPD_CGI_DONE;
	uint8_t sta_mac[6] = { 0 };
	esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

	strcpy(buff, "UnKnOwN");
	if (strcmp(token, "STAMAC")==0) {
		sprintf(buff, "%02x%02x%02x%02x%02x%02x",MAC2STR(sta_mac));
	}
	httpdSend(connData, buff, -1);
	return HTTPD_CGI_DONE;
}

HttpdBuiltInUrl builtInUrls[] = { ///URL HANDLER
	ROUTE_CGI_ARG("*", cgiRedirectApClientToHostname, "esp32.nonet"),
	ROUTE_REDIRECT("/", "/index.tpl"),
	ROUTE_TPL("/index.tpl", tplSETindex),

	{"/set/*", authBasic, myPassFn},
	ROUTE_REDIRECT("/set", "/set/settings.tpl"),
	ROUTE_REDIRECT("/set/", "/set/settings.tpl"),
	ROUTE_TPL("/set/settings.tpl", tplSET),
	ROUTE_CGI("/set/settingswifi.cgi", cgiSETwifi),
	ROUTE_CGI("/set/settingsmesh.cgi", cgiSETmesh),
	ROUTE_CGI("/set/settings.cgi", cgiSET),

		{"/flash/*", authBasic, myPassFn},
	ROUTE_REDIRECT("/flash", "/flash/index.html"),
	ROUTE_REDIRECT("/flash/", "/flash/index.html"),
	ROUTE_CGI("/flash/flashinfo.json", cgiGetFlashInfo),
	ROUTE_CGI("/flash/setboot", cgiSetBoot),
	ROUTE_CGI_ARG("/flash/upload", cgiUploadFirmware, &uploadParams),
	ROUTE_CGI_ARG("/flash/erase", cgiEraseFlash, &uploadParams),
	ROUTE_CGI("/flash/reboot", cgiRebootFirmware),

	// Files in /static dir are assumed to never change, so send headers to encourage browser to cache forever.
	ROUTE_FILE_EX("/static/*", &CgiOptionsEspfsStatic),
	ROUTE_FILESYSTEM(),

	ROUTE_END()
};

void EspFwebInit(){//WEB init
	///ssl cert 
	extern const unsigned char cacert_der_start[] asm("_binary_certificate_der_start");
	extern const unsigned char cacert_der_end[]   asm("_binary_certificate_der_end");
	const size_t cacert_der_bytes = cacert_der_end - cacert_der_start;
	extern const unsigned char prvtkey_der_start[] asm("_binary_key_der_start");
	extern const unsigned char prvtkey_der_end[]   asm("_binary_key_der_end");
	const size_t prvtkey_der_bytes = prvtkey_der_end - prvtkey_der_start;
	EspFsConfig espfs_conf = {.memAddr = espfs_image_bin,};
	EspFs* fs = espFsInit(&espfs_conf);
    httpdRegisterEspfs(fs);
	tcpip_adapter_init();
	httpdFreertosInit(&httpdFreertosInstance,//web config
	                  builtInUrls,
	                  LISTEN_PORT,
	                  connectionMemory,
	                  MAX_CONNECTIONS,
	                  HTTPD_FLAG_SSL);
	httpdFreertosSslInit(&httpdFreertosInstance); // configure libesphttpd for ssl
 	httpdFreertosSslSetCertificateAndKey(&httpdFreertosInstance,
                                        cacert_der_start, cacert_der_bytes,
                                       prvtkey_der_start, prvtkey_der_bytes);
	httpdFreertosStart(&httpdFreertosInstance);//start web
}

///general functions~~~~~~~~~~~~~~~
static mdf_err_t app_event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id) {
	case SYSTEM_EVENT_STA_START:
		tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, my_hostname);
		esp_wifi_connect(); /* Calling this unconditionally would interfere with the WiFi CGI. */ //i should look a this
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
	{
		tcpip_adapter_ip_info_t sta_ip_info;
		wifi_config_t sta_conf;
		printf("~~~~~STA~~~~~" "\n");
		if (esp_wifi_get_config(TCPIP_ADAPTER_IF_STA, &sta_conf) == MDF_OK) {
			printf("ssid: %s" "\n", sta_conf.sta.ssid);
		}

		if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_STA, &sta_ip_info) == MDF_OK) {
			printf("IP:" IPSTR "\n", IP2STR(&sta_ip_info.ip));
			printf("MASK:" IPSTR "\n", IP2STR(&sta_ip_info.netmask));
			printf("GW:" IPSTR "\n", IP2STR(&sta_ip_info.gw));
		}
		printf("~~~~~~~~~~~~~" "\n");
	}
	break;
	case SYSTEM_EVENT_STA_CONNECTED:
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		/* This is a workaround as ESP32 WiFi libs don't currently
           auto-reassociate. */
		/* Skip reconnect if disconnect was deliberate or authentication      *\
        \* failed.                                                            */
		switch(event->event_info.disconnected.reason){
		case WIFI_REASON_ASSOC_LEAVE:
		case WIFI_REASON_AUTH_FAIL:
			break;
		default:
			esp_wifi_connect();
			break;
		}
		break;
	case SYSTEM_EVENT_AP_START:
		{
			tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_AP, my_hostname);
			tcpip_adapter_ip_info_t ap_ip_info;
			wifi_config_t ap_conf;
			printf("~~~~~AP~~~~~" "\n");
			if (esp_wifi_get_config(TCPIP_ADAPTER_IF_AP, &ap_conf) == MDF_OK) {
				printf("ssid: %s" "\n", ap_conf.ap.ssid);
				if (ap_conf.ap.authmode != WIFI_AUTH_OPEN) printf("pass: %s" "\n", ap_conf.ap.password);
			}

			if (tcpip_adapter_get_ip_info(TCPIP_ADAPTER_IF_AP, &ap_ip_info) == MDF_OK) {
				printf("IP:" IPSTR "\n", IP2STR(&ap_ip_info.ip));
				printf("MASK:" IPSTR "\n", IP2STR(&ap_ip_info.netmask));
				printf("GW:" IPSTR "\n", IP2STR(&ap_ip_info.gw));
			}
			printf("~~~~~~~~~~~~" "\n");
		}
		break;
		case SYSTEM_EVENT_AP_STACONNECTED:
			MDF_LOGI( "station:" MACSTR" join,AID=%d",
					MAC2STR(event->event_info.sta_connected.mac),
					event->event_info.sta_connected.aid);

			break;
		case SYSTEM_EVENT_AP_STADISCONNECTED:
			MDF_LOGI( "station:" MACSTR"leave,AID=%d",
					MAC2STR(event->event_info.sta_disconnected.mac),
					event->event_info.sta_disconnected.aid);

			break;
		case SYSTEM_EVENT_SCAN_DONE:

			break;
		default:
			break;
	}

	return MDF_OK;
}

void init_wifi()//wifi init
	{
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	MDF_ERROR_ASSERT( esp_wifi_init(&cfg) );

	if(nvsdata.wifimode==1){
		wifi_config_t wifi_config = {
			.sta = {
			},
		};
		uint8_t len= strlen(nvsdata.router_ssid);
		uint8_t plen= strlen(nvsdata.router_password);
		memcpy(wifi_config.sta.ssid,nvsdata.router_ssid,len);
		memcpy(wifi_config.sta.password,nvsdata.router_password,plen);

		MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
		MDF_ERROR_ASSERT(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
	}
	else if(nvsdata.wifimode==0){
		tcpip_adapter_init();
		wifi_config_t wifi_config = {
			.ap = {
			.channel = 1,
			.max_connection = 2,
			.authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
		};
		uint8_t len= strlen(nvsdata.AP_ssid);
		memcpy(wifi_config.ap.ssid,nvsdata.AP_ssid,len);
		wifi_config.ap.ssid_len=len;
		uint8_t plen= strlen(nvsdata.AP_password);
		memcpy(wifi_config.ap.password,nvsdata.AP_password,plen);

		MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_AP));
		MDF_ERROR_ASSERT(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
	}
   
    MDF_ERROR_ASSERT(esp_wifi_start());
	
}

static void print_system_info_timercb(void *timer){//memory debug

	  if (!heap_caps_check_integrity_all(true)) {
        MDF_LOGE("At least one heap is corrupt");
    }

    mdf_mem_print_heap();
    mdf_mem_print_record();
    mdf_mem_print_task();
}

void app_main(void) {
	
	mdf_err_t err;

	// Init NVS
	err = nvs_flash_init();
	if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		// NVS partition was truncated and needs to be erased
		// Retry nvs_flash_init
		MDF_ERROR_ASSERT(nvs_flash_erase());
		err = nvs_flash_init();
	}
	MDF_ERROR_ASSERT(err);
	ReadNvsData();
	if(nvsdata.debug<6)esp_log_level_set("test_settings", nvsdata.debug);
    else {esp_log_level_set("test_settings", ESP_LOG_VERBOSE);printf("verb");}

	EspFwebInit();

	MDF_LOGE("WIFIMODE %d",nvsdata.wifimode);

	MDF_ERROR_ASSERT(esp_event_loop_init(app_event_handler, NULL));
	init_wifi(); // Start Wifi, restore factory wifi settings if not initialized

	printf("\nWEB SERViNG\n");

	if(nvsdata.debug==6){
		TimerHandle_t timer = xTimerCreate("print_system_info", 5000 / portTICK_RATE_MS,true, NULL, print_system_info_timercb);
		xTimerStart(timer, 0);
	}
}
