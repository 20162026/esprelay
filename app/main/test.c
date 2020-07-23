// Copyright 2017 Espressif Systems (Shanghai) PTE LTD
// Modifications copyright (C) 2020 tk20162026@gmail.com

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mdf_common.h"//pagr. espmdf biblioteka
#include "mesh_mqtt_handle2.h"//mqtt biblioteka
#include "mwifi.h"//ESPmesh biblioteka
#include "driver/uart.h"//uart biblioteka
#include "cJSON.h"//json biblioteka
#include "driver/pcnt.h"//pulse counter biblioteka
#include "mbedtls/md.h"//TLS biblioteka
#include "esp_ota_ops.h"//OTA atnaujinimų biblioteka
#include "esp_partition.h"//OTA reikalinga biblioteka
#include "driver/gpio.h"//bendros paskirties išvadų bilbioteka
extern uint8_t temprature_sens_read();//vidinės temperatūros nuskaitymas

#define EX_UART_NUM UART_NUM_2
#define FLOAT_NAN ((float)0xffffffff)
#define BUF_SIZE (128)          //uart2
#define RD_BUF_SIZE (BUF_SIZE)  //uart2 read BUFFER

///pulse counter nustatymai
#define PCNT_TEST_UNIT      PCNT_UNIT_0
#define PCNT_INPUT_SIG_IO   GPIO_NUM_33

//uart naudojamas tik RX
#define UART_TX             GPIO_NUM_21
#define UART_RX             GPIO_NUM_25
//relė
#define RELAY_PIN           GPIO_NUM_32 

typedef struct  {
    float voltage;
    float current; 
    float wattage;
    float kWh;
}hlw_data_t;

static struct nvsdata_t {//nustatymų struct
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

static nvs_handle data_nvs_handle;
static QueueHandle_t uart2_queue, hlw_queue;
static const char *TAG = "test_mqtt";

static uint8_t HLWparsing(uint8_t* dtmp, size_t *event_size) ///HLW8032 informacijos apdorojimas
{

    hlw_data_t hlw={};
    uint hlwfail = 0 ;
    int16_t count = 0 ;
    uint32_t PFparam =0;
    float acof=0.5f;//sroves koeficientas
    float vcof=1.8677f;//įtampos koeficientas
    
    if(*event_size!=24){
    if(*event_size%24!=0){uart_flush_input(EX_UART_NUM);MDF_LOGW("HLW UART FLUSH");}
        hlwfail=1;}

    uint8_t checksum=0;
    for(int i = 2;i<*event_size-1;i++)checksum+=dtmp[i];

    if((checksum)!=dtmp[*event_size-1]){;MDF_LOGW("HLW checksum failed");hlwfail=1;}
    if(dtmp[0]==0xAA||((dtmp[0]&0xA0)&&dtmp[0]&0x01)){;MDF_LOGW("HLW error correction failed REGs are not usabl");hlwfail=1;}


    if(hlwfail){
                        MDF_LOGE("HLW parsing failed");
    }else{
    
        if( !((dtmp[0]&0xA0)&&(dtmp[0]&0x08)) && (dtmp[20]&0x40))
        hlw.voltage=((float)((dtmp[2]<<16) + (dtmp[3]<<8) + dtmp[4])/(float)((dtmp[5]<<16) + (dtmp[6]<<8) + dtmp[7])*vcof);
        else MDF_LOGI("Voltage reg overflow or not ready");

        if( !((dtmp[0]&0xA0)&&(dtmp[0]&0x04)) && (dtmp[20]&0x20))
        hlw.current=(float)((dtmp[8]<<16) + (dtmp[9]<<8) + dtmp[10])/(float)((dtmp[11]<<16) + (dtmp[12]<<8) + dtmp[13])*acof;
        else MDF_LOGI("Current reg overflow or not ready");

        if( !((dtmp[0]&0xA0)&&(dtmp[0]&0x02)) && (dtmp[20]&0x10))
        hlw.wattage=(float)((dtmp[14]<<16) + (dtmp[15]<<8) + dtmp[16])/(float)((dtmp[17]<<16) + (dtmp[18]<<8) + dtmp[19])*acof*vcof;
        else MDF_LOGV("Wattage reg overflow or not ready");
        
        PFparam=(uint32_t)((dtmp[14]<<16) + (dtmp[15]<<8) + dtmp[16]);

        pcnt_get_counter_value(PCNT_TEST_UNIT, &count);

        hlw.kWh=(float)(count)/((1/(float)PFparam)*(1.0/(vcof*acof))*3600000000000.0f);
       
        pcnt_counter_clear(PCNT_TEST_UNIT);
        
        xQueueSend(hlw_queue, (void * )&hlw ,(portTickType)portMAX_DELAY);
    }
    return 0;
}

void root_write_task(void *arg) //ROOT WRITE TO MQTT
{
    mdf_err_t ret = MDF_OK;
    char *data = NULL;
    size_t size = MWIFI_PAYLOAD_LEN;
    uint8_t src_addr[MWIFI_ADDR_LEN] = { 0x0 };
    mwifi_data_type_t data_type = { 0x0 };

    MDF_LOGI("Root write task is running");
    

    while (mwifi_is_connected() && esp_mesh_get_layer() == MESH_ROOT) {
        if (!mesh_mqtt_is_connect()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        ret = mwifi_root_read(src_addr, &data_type, &data, &size, portMAX_DELAY);
        MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_read", mdf_err_to_name(ret));
        ret = mesh_mqtt_write(src_addr, data, size, data_type.custom);
        MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mesh_mqtt_publish", mdf_err_to_name(ret));

MEM_FREE:
        MDF_FREE(data);
    }

    MDF_LOGW("Root write task is exit");
    mesh_mqtt_stop();
    vTaskDelete(NULL);
}

void root_read_task(void *arg) ///ROOT READ FROM MQTT
{
    mdf_err_t ret = MDF_OK;

    MDF_LOGI("Root read task is running");

    while (mwifi_is_connected() && esp_mesh_get_layer() == MESH_ROOT) {
        if (!mesh_mqtt_is_connect()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }

        mqtt_data_t *request = NULL;
        mwifi_data_type_t data_type = { 0x0 };
        data_type.compression=1;

        ret = mesh_mqtt_read(&request, pdMS_TO_TICKS(500));
        if (ret != MDF_OK) {
            continue;
        }

        ret = mwifi_root_write(request->addr, 1, &data_type, request->data, request->data_len, true);

        MDF_ERROR_GOTO(ret != MDF_OK, MEM_FREE, "<%s> mwifi_root_write", mdf_err_to_name(ret));

MEM_FREE:
        MDF_FREE(request->data);
        MDF_FREE(request);
    }

    MDF_LOGW("Root read task is exit");
    mesh_mqtt_stop();
    vTaskDelete(NULL);
}

static void node_read_task(void *arg) //NODE READ FROM MESH
{
    mdf_err_t ret = MDF_OK;
    char *data = MDF_MALLOC(MWIFI_PAYLOAD_LEN);
    size_t size = MWIFI_PAYLOAD_LEN;
    mwifi_data_type_t data_type = { 0x0 };
    mwifi_data_type_t data_type_send = { 0x0 };
    uint8_t src_addr[MWIFI_ADDR_LEN] = { 0x0 };
    

    uint32_t factory_salt=0;
    char shaResult[32];
    TickType_t time_old=0;

    //relay gpio setup
    gpio_pad_select_gpio(RELAY_PIN);
    gpio_set_direction(RELAY_PIN, GPIO_MODE_OUTPUT);

    MDF_LOGI("Node read task is running");

    for (;;) {
        if (!mwifi_is_connected()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue; 
        }
        size = MWIFI_PAYLOAD_LEN;
        memset(data, 0, MWIFI_PAYLOAD_LEN);
        ret = mwifi_read(src_addr, &data_type, data, &size, portMAX_DELAY);
        
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_read", mdf_err_to_name(ret));

       // JSON data parsing and respons
        MDF_LOGI("Node receive: " MACSTR ", size: %d, data: %s", MAC2STR(src_addr), size, data);

        if(xTaskGetTickCount()-time_old<2000)MDF_LOGW("Node read: parsing timeout");//check if 1000 ticks have passed 1s
        else{
            cJSON *json_data=NULL, *json_function=NULL,*json_adr=NULL;
            cJSON *obj= cJSON_Parse(data);
            
            json_data=cJSON_GetObjectItemCaseSensitive(obj, "data");
            json_function=cJSON_GetObjectItemCaseSensitive(obj, "function");
            json_adr=cJSON_GetObjectItemCaseSensitive(obj, "dest_adr");
            MDF_ERROR_CONTINUE(json_data==NULL||json_function==NULL||json_adr==NULL, "Parse JSON parsing error");

            if(strcmp("onoff",json_function->valuestring)==0)///relay control
            {
                if(strlen(json_data->valuestring)==1&&(json_data->valuestring[0]=='0'||json_data->valuestring[0]=='1')){
                gpio_set_level(RELAY_PIN,json_data->valuestring[0]-'0');
                data_type_send.compression=1;
                data_type_send.custom=MESH_MQTT_DATA_RESPOND;
                ret = mwifi_write(NULL, &data_type_send, data, size, true); ///respons to root
                vTaskDelay(1000 / portTICK_RATE_MS);
                }
                MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
            }else 
            if(strcmp("factory",json_function->valuestring)==0){///factory reset
                if(factory_salt==0){
                    //generate factory salt
                    factory_salt=esp_random();
                    char payload[65];
                    sprintf(payload, "%s%u",nvsdata.conf_password,factory_salt);
                    mbedtls_md_context_t ctx;
                    mbedtls_md_type_t md_type = MBEDTLS_MD_SHA256;
                    const size_t payloadLength = strlen(payload);
                    mbedtls_md_init(&ctx);
                    mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(md_type), 0);
                    mbedtls_md_starts(&ctx);
                    mbedtls_md_update(&ctx, (const unsigned char *) payload, payloadLength);
                    mbedtls_md_finish(&ctx, (unsigned char *) shaResult);
                    mbedtls_md_free(&ctx);
                    MDF_LOGE("factory reset password set  num %u  hash:  ",factory_salt);
                    for(int i= 0; i< sizeof(shaResult); i++){
                        char str[3];
                        sprintf(str, "%02x", (int)shaResult[i]);
                        printf(str);
                        }
                    printf("\n");
                }

                if(!memcmp(json_data->valuestring,shaResult,32)){
                    MDF_LOGW("factory reset password accepted...rebooting...");
                    esp_ota_set_boot_partition(esp_partition_find_first(ESP_PARTITION_TYPE_APP,ESP_PARTITION_SUBTYPE_APP_FACTORY,NULL ));
                    esp_restart();
                }else{
                    char *factorydata=NULL;
                    cJSON *obj_factory = cJSON_CreateObject();
                    cJSON_AddStringToObject(obj_factory, "function", json_function->valuestring);

                    char buffer [33];
                    sprintf (buffer, "%u",factory_salt );
                    cJSON_AddStringToObject(obj_factory, "data", buffer);//data is factory salt in deci
                    cJSON_AddStringToObject(obj_factory, "dest_adr", json_adr->valuestring);
                    int factory_size=asprintf(&factorydata,cJSON_PrintUnformatted(obj_factory));
                    cJSON_Delete(obj_factory);

                    MDF_LOGI(" factory %d AND %s \n",factory_size,factorydata);

                    data_type_send.compression=1;
                    data_type_send.custom=MESH_MQTT_DATA_RESPOND;
                    ret = mwifi_write(NULL, &data_type_send, factorydata, factory_size, true); ///respons to root
                    MDF_FREE(factorydata);
                    MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
                }
            }else{
                ////custom function code
                MDF_LOGW("unknown mqtt function");
            }
            cJSON_Delete(obj);
            time_old=xTaskGetTickCount();
        }     
    }

    MDF_LOGW("Node read task is exit");
    MDF_FREE(data);
    vTaskDelete(NULL);
}

static void node_write_task(void *arg) ///NODE WRITE TO MESH
{
    mdf_err_t ret = MDF_OK;
    size_t size = 0;
    char *data = NULL;
    mwifi_data_type_t data_type = { 0x0 };
    data_type.custom = MESH_MQTT_DATA_JSON;
    data_type.compression = 1;
    uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
    mesh_addr_t parent_mac = { 0 };

    MDF_LOGI("Node task is running");
    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);

    hlw_data_t hlw;
    float voltage=0.0f;
    float current=0.0f;
    float wattage=0.0f;
    float kwh=0.0f;
    uint8_t quecount;

    for (;;) {
        if (!mwifi_is_connected() || !mwifi_get_root_status()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            continue;
        }
        //@brief Send device heartbeat information to mqtt server throught root node.

        voltage =0;
        current =0;
        wattage =0;
        quecount =0;
        kwh=0;
    
        while(uxQueueMessagesWaiting(hlw_queue)){
            xQueueReceive(hlw_queue,(void*) &hlw,0);
            voltage+=hlw.voltage;
            current+=hlw.current;
            wattage+=hlw.wattage;
            kwh+=hlw.kWh;
            quecount++;
        }
            voltage /=quecount;
            current /=quecount;
            wattage /=quecount;

        esp_mesh_get_parent_bssid(&parent_mac);
        size = asprintf(&data, "{\"self\": \"%02x%02x%02x%02x%02x%02x\", \"parent\":\"%02x%02x%02x%02x%02x%02x\",\"layer\":%d,\"itemp\":%d,\"V\":%e,\"A\":%e,\"W\":%e,\"kwh\":%e}",
        MAC2STR(sta_mac), MAC2STR(parent_mac.addr), esp_mesh_get_layer(),(temprature_sens_read()-32)*5/9,
        voltage,current,wattage,kwh
        );
        ret = mwifi_write(NULL, &data_type, data, size, true);
        MDF_FREE(data);
        MDF_ERROR_CONTINUE(ret != MDF_OK, "<%s> mwifi_write", mdf_err_to_name(ret));
        vTaskDelay(3000 / portTICK_RATE_MS);//heartbeat delay
    }

    MDF_LOGW("Node task is exit");
    vTaskDelete(NULL);
}

static void print_system_info_timercb(void *timer)//įtaiso būsenos išvedimas į uart
{   
    uint8_t primary = 0;
    wifi_second_chan_t second = 0;
    mesh_addr_t parent_bssid = { 0 };
    uint8_t sta_mac[MWIFI_ADDR_LEN] = { 0 };
    mesh_assoc_t mesh_assoc = { 0x0 };
    wifi_sta_list_t wifi_sta_list = { 0x0 };

    esp_wifi_get_mac(ESP_IF_WIFI_STA, sta_mac);
    esp_wifi_ap_get_sta_list(&wifi_sta_list);
    esp_wifi_get_channel(&primary, &second);
    esp_wifi_vnd_mesh_get(&mesh_assoc);
    esp_mesh_get_parent_bssid(&parent_bssid);
   
    MDF_LOGI("System information, channel: %d, layer: %d, self mac: " MACSTR ", parent bssid: " MACSTR
             ", parent rssi: %d, node num: %d, free heap: %u, internal temp: %d",
             primary,
             esp_mesh_get_layer(), MAC2STR(sta_mac), MAC2STR(parent_bssid.addr),
             mesh_assoc.rssi, esp_mesh_get_total_node_num(), esp_get_free_heap_size(),(temprature_sens_read()-32)*5/9 );

    for (int i = 0; i < wifi_sta_list.num; i++) {
        MDF_LOGI("Child mac: " MACSTR, MAC2STR(wifi_sta_list.sta[i].mac));
    }

if(nvsdata.debug==6){
    if (!heap_caps_check_integrity_all(true)) {
        MDF_LOGE("At least one heap is corrupt");
    }
    mdf_mem_print_heap();
    mdf_mem_print_record();
    mdf_mem_print_task();
} 
}

static mdf_err_t wifi_init()///wifi init
{
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    tcpip_adapter_init();
    MDF_ERROR_ASSERT(esp_event_loop_init(NULL, NULL));
    MDF_ERROR_ASSERT(esp_wifi_init(&cfg));
    MDF_ERROR_ASSERT(esp_wifi_set_storage(WIFI_STORAGE_FLASH));
    MDF_ERROR_ASSERT(esp_wifi_set_mode(WIFI_MODE_STA));
    MDF_ERROR_ASSERT(esp_wifi_set_ps(WIFI_PS_NONE));
    MDF_ERROR_ASSERT(esp_mesh_set_6m_rate(false));
    MDF_ERROR_ASSERT(esp_wifi_start());
    return MDF_OK;
}

static void pcnt_init(void) //pulse counter init
{

    pcnt_config_t pcnt_config = {
        // Set PCNT input signal 
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = PCNT_TEST_UNIT,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
    };


    pcnt_unit_config(&pcnt_config);
    pcnt_set_filter_value(PCNT_TEST_UNIT, 100);
    pcnt_filter_enable(PCNT_TEST_UNIT);
    pcnt_counter_pause(PCNT_TEST_UNIT);
    pcnt_counter_clear(PCNT_TEST_UNIT);
    pcnt_counter_resume(PCNT_TEST_UNIT);
}

static mdf_err_t uart2_init() //uart init 8032
{
    uart_config_t uart_config = {
        .baud_rate = 4800,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_EVEN,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    MDF_ERROR_ASSERT(uart_param_config(EX_UART_NUM, &uart_config));
    MDF_ERROR_ASSERT(uart_set_pin(EX_UART_NUM,  UART_TX,  UART_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    MDF_ERROR_ASSERT(uart_driver_install(EX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart2_queue, 0));
    return MDF_OK;
}

static mdf_err_t event_loop_cb(mdf_event_loop_t event, void *ctx)//FreeRTOS pertrauktys
{
    MDF_LOGI("event_loop_cb, event: %d", event);

    switch (event) {
        case MDF_EVENT_MWIFI_STARTED:
            MDF_LOGI("MESH is started");
            break;

        case MDF_EVENT_MWIFI_PARENT_CONNECTED:
            MDF_LOGI("Parent is connected on station interface");
            break;

        case MDF_EVENT_MWIFI_PARENT_DISCONNECTED:
            MDF_LOGI("Parent is disconnected on station interface");
            if (esp_mesh_is_root())mesh_mqtt_stop(); 
            break;

        case MDF_EVENT_MWIFI_ROUTING_TABLE_ADD:
        case MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE:
            MDF_LOGI("MDF_EVENT_MWIFI_ROUTING_TABLE_REMOVE, total_num: %d", esp_mesh_get_total_node_num());
            if (esp_mesh_is_root() && mwifi_get_root_status()) {
                mdf_err_t err = mesh_mqtt_update_topo();
                if (err != MDF_OK) {
                    MDF_LOGE("Update topo failed");
                }
            }
            break;

        case MDF_EVENT_MWIFI_ROOT_GOT_IP: {
            MDF_LOGI("Root obtains the IP address. It is posted by LwIP stack automatically");
            mesh_mqtt_start(nvsdata.mqtturl,nvsdata.topic,strlen(nvsdata.topic));/////change topic and topic len
            break;
        }

        case MDF_EVENT_CUSTOM_MQTT_CONNECT:
            MDF_LOGI("MQTT connect");
            mdf_err_t err = mesh_mqtt_update_topo();

            if (err != MDF_OK) {
                MDF_LOGE("Update topo failed");
            }

            err = mesh_mqtt_subscribe();

            if (err != MDF_OK) {
                MDF_LOGE("Subscribe failed");
            }

            mwifi_post_root_status(true);

            xTaskCreate(root_write_task, "root_write", 4 * 1024,
                        NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
            xTaskCreate(root_read_task, "root_read", 4 * 1024,
                        NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
            break;

        case MDF_EVENT_CUSTOM_MQTT_DISCONNECT:
            MDF_LOGI("MQTT disconnected");
            mwifi_post_root_status(false);
            break;

        default:
            break;
    }

    return MDF_OK;
}

static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    //init counter
    pcnt_init();

    for(;;) {
    //wait for mwifi
        if (!mwifi_is_connected() || !mwifi_get_root_status()) {
            vTaskDelay(500 / portTICK_RATE_MS);
            uart_flush_input(EX_UART_NUM);
            continue;
        }

        //Waiting for UART event.
        if(xQueueReceive(uart2_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            switch(event.type) {
                //Event of UART receving data
                case UART_DATA:
                   // MDF_LOGW("[UART DATA]: %d", event.size);
                    uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);  
                    HLWparsing(dtmp,&event.size);
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    MDF_LOGW("hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart2_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    MDF_LOGW("ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart2_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                MDF_LOGV("uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    MDF_LOGW("uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    MDF_LOGW("uart frame error");
                    break;
                //Others
                default:
                    MDF_LOGV("uart event type: %d", event.type);
                    break;
                }
        }
    }

    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

#define NVS_NAMESPACE "nvs"
#define NVS_DATA_KEY "nvsdata"

static void ReadNvsData()//nustatimų nuskaitymas iš NVS
{
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
		if(err!=MDF_OK||nvssize!=sizeof(struct nvsdata_t)){//if error load hardcoded default values
			MDF_LOGW("nvs init not found or corrupted, loading default values. error:%d",err);
			strcpy( nvsdata.router_ssid,CONFIG_ROUTER_SSID);
            strcpy( nvsdata.router_password,CONFIG_ROUTER_PASSWORD);
            strcpy( nvsdata.mesh_id,CONFIG_MESH_ID);
            strcpy( nvsdata.mesh_password,CONFIG_MESH_PASSWORD);
            strcpy(nvsdata.mqtturl,CONFIG_MQTT_URL);
            strcpy(nvsdata.AP_ssid,"esptest");
            strcpy(nvsdata.AP_password,"esptest");
            strcpy(nvsdata.topic,"gBridge/u1/");
            strcpy(nvsdata.conf_password,"esptest");
            nvsdata.wifimode=0;
            nvsdata.debug=3;
		}
	}

    nvs_close(data_nvs_handle);

}

void app_main()
{
    mdf_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        MDF_ERROR_ASSERT(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    MDF_ERROR_ASSERT(ret);


    mwifi_init_config_t cfg = MWIFI_INIT_CONFIG_DEFAULT();
    ReadNvsData();
    mwifi_config_t config ={};
    memcpy(config.mesh_id,nvsdata.mesh_id,6);
    memcpy(config.mesh_password,nvsdata.mesh_password,strlen(nvsdata.mesh_password));
    memcpy(config.router_ssid,nvsdata.router_ssid,strlen(nvsdata.router_ssid));
    memcpy(config.router_password,nvsdata.router_password,strlen(nvsdata.router_password));
    

    if(nvsdata.debug<6)esp_log_level_set("*", nvsdata.debug);//informacijos išvedimo lygio nustatymas
    else esp_log_level_set("*", ESP_LOG_INFO);

    //wifi, mesh, uart init
    MDF_ERROR_ASSERT(mdf_event_loop_init(event_loop_cb));
    MDF_ERROR_ASSERT(wifi_init());
    MDF_ERROR_ASSERT(mwifi_init(&cfg));
    MDF_ERROR_ASSERT(mwifi_set_config(&config));
    MDF_ERROR_ASSERT(mwifi_start());
    MDF_ERROR_ASSERT(uart2_init());
   


    //creating node tasks
    xTaskCreate(node_write_task, "node_write_task"  , 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    xTaskCreate(node_read_task, "node_read_task"    , 4 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);

    hlw_queue=xQueueCreate(100, sizeof(hlw_data_t));
    if(hlw_queue!=NULL)xTaskCreate(uart_event_task, "uart_event_task"  , 2 * 1024, NULL, CONFIG_MDF_TASK_DEFAULT_PRIOTY, NULL);
    else MDF_LOGE("FAILD TO CREATE HLW QUE");
    
    //įtaiso būseon laikmatis
    TimerHandle_t timer = xTimerCreate("print_system_info", 10000 / portTICK_RATE_MS,true, NULL, print_system_info_timercb);
    xTimerStart(timer, 0);
}
