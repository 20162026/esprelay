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

#include "mesh_mqtt_handle2.h"
#include "cJSON.h"
#include "esp_wifi.h"
#include "mbedtls/base64.h"
#include "mlink.h"
#include "mwifi.h"

#define MWIFI_ADDR_LEN          (6) //Length of MAC address

static struct mesh_mqtt {
    xQueueHandle queue; // mqtt receive data queue
    esp_mqtt_client_handle_t client; 
    bool is_connected;
    uint8_t addr[6];
    char publish_topic[48];
    char topo_topic[48];
    char sub_topic[48];
    char respond_topic[48];
    char topic_begin[48];
} g_mesh_mqtt;


static const char *TAG = "mesh_mqtt_handle2";

static mqtt_data_t *mesh_mqtt_parse_data(const char *topic, size_t topic_size, const char *payload, size_t payload_size)
{
        if(topic==NULL)return NULL;
        char *fu_str = strstr(topic, g_mesh_mqtt.topic_begin);
        assert(fu_str != NULL);
        fu_str+=strlen(g_mesh_mqtt.topic_begin);
        char *mac_str = strchr(fu_str,'/');
        assert(mac_str != NULL);
        *mac_str = '\0';
        char *mac_str_end = strchr(mac_str+1,'/');
        assert(mac_str_end != NULL);
        *mac_str_end='\0';

        if(strlen(mac_str+1)>12||strlen(fu_str)>20){
            MDF_LOGE("mqtt msg overflow: mac %d/12  fu %d/20",strlen(mac_str+1),strlen(fu_str));
            return NULL;
        }

        mqtt_data_t *request = MDF_CALLOC(1, sizeof(mqtt_data_t));
        if (request == NULL) {
        MDF_LOGE("No memory");
        return request;
        }

        mlink_mac_str2hex(mac_str+1, request->addr);
        if(payload_size>32){
            MDF_LOGE("mqtt data too long: %d limited to 32",payload_size);
            payload_size=32;
        }
        char *data_temp=(char *)MDF_MALLOC(payload_size+1);
        if(data_temp==NULL){
            request=NULL;
            goto _cleardatatemp;
        }
        memcpy(data_temp, payload,payload_size);
        data_temp[payload_size]='\0';
    
        cJSON *obj = cJSON_CreateObject();
        if(obj==NULL){
            request=NULL;
            goto _clearjson;
        }

        cJSON_AddStringToObject(obj, "function", fu_str);
        cJSON_AddStringToObject(obj, "data", data_temp);
        cJSON_AddStringToObject(obj, "dest_adr", mac_str+1);
        char *jsondata = cJSON_PrintUnformatted(obj);
        
        request->data=(char *)MDF_MALLOC(strlen(jsondata)+1);
        strcpy(request->data,jsondata);
        request->data_len=strlen(jsondata);

        _clearjson:
        cJSON_Delete(obj);
        _cleardatatemp:
        MDF_FREE(data_temp);
        
        return request;
}

static esp_err_t mqtt_event_handler(esp_mqtt_event_handle_t event)
{
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            MDF_LOGD("MQTT_EVENT_CONNECTED");
            g_mesh_mqtt.is_connected = true;
            mdf_event_loop_send(MDF_EVENT_CUSTOM_MQTT_CONNECT, NULL);
            break;

        case MQTT_EVENT_DISCONNECTED:
            MDF_LOGD("MQTT_EVENT_DISCONNECTED");
            g_mesh_mqtt.is_connected = false;
            mdf_event_loop_send(MDF_EVENT_CUSTOM_MQTT_DISCONNECT, NULL);
            break;

        case MQTT_EVENT_SUBSCRIBED:
            MDF_LOGD("MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_UNSUBSCRIBED:
            MDF_LOGD("MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_PUBLISHED:
            MDF_LOGD("MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;

        case MQTT_EVENT_DATA: {
            MDF_LOGD("MQTT_EVENT_DATA, topic: %.*s, data: %.*s",event->topic_len, event->topic, event->data_len, event->data);
            mqtt_data_t *item = mesh_mqtt_parse_data(event->topic, event->topic_len, event->data, event->data_len);
            if (item == NULL) {
                break;
            }
            if (xQueueSend(g_mesh_mqtt.queue, &item, 0) != pdPASS) {
                MDF_LOGD("Send receive queue failed");
                MDF_FREE(item->data);
                MDF_FREE(item);
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            MDF_LOGD("MQTT_EVENT_ERROR");
            break;

        default:
            MDF_LOGD("Other event id:%d", event->event_id);
            break;
    }

    return ESP_OK;
}

bool mesh_mqtt_is_connect()
{
    return g_mesh_mqtt.is_connected;
}

mdf_err_t mesh_mqtt_subscribe()
{
    int msg_id = esp_mqtt_client_subscribe(g_mesh_mqtt.client, g_mesh_mqtt.sub_topic, 0);
    MDF_ERROR_CHECK(msg_id < 0, MDF_FAIL, "Subscribe failed");
    return MDF_OK;
}

mdf_err_t mesh_mqtt_unsubscribe()
{
    int msg_id = esp_mqtt_client_unsubscribe(g_mesh_mqtt.client, g_mesh_mqtt.sub_topic);

    if (msg_id > 0) {
        MDF_LOGI("Unsubscribe: %s, msg_id = %d", g_mesh_mqtt.sub_topic, msg_id);
        return MDF_OK;
    } else {
        MDF_LOGI("Unsubscribe: %s failed", g_mesh_mqtt.sub_topic);
        return MDF_FAIL;
    }
}

mdf_err_t mesh_mqtt_update_topo()
{
    MDF_ERROR_CHECK(g_mesh_mqtt.client == NULL, MDF_ERR_INVALID_STATE, "MQTT client has not started");

    mdf_err_t ret = MDF_FAIL;
    char mac_addr[13];

    int table_size = esp_mesh_get_routing_table_size();
    mesh_addr_t *route_table = MDF_CALLOC(table_size, sizeof(mesh_addr_t));
    ESP_ERROR_CHECK(esp_mesh_get_routing_table(route_table, table_size * sizeof(mesh_addr_t), &table_size));
   
    cJSON *topolog = cJSON_CreateObject();
    cJSON *obj = cJSON_CreateArray();
    MDF_ERROR_GOTO(obj == NULL||topolog == NULL, _no_mem, "Create json object failed");
    cJSON_AddItemToObject(topolog,"topology",obj);

    for (int i = 0; i < table_size; i++) {
        mlink_mac_hex2str(route_table[i].addr, mac_addr);
        cJSON *item = cJSON_CreateString(mac_addr);
        MDF_ERROR_GOTO(item == NULL, _no_mem, "Create string object failed");
        cJSON_AddItemToArray(obj, item);
    }

    MDF_FREE(route_table);
    
    char *str = cJSON_PrintUnformatted(topolog);
    MDF_ERROR_GOTO(str == NULL, _no_mem, "Print JSON failed");

    esp_mqtt_client_publish(g_mesh_mqtt.client, g_mesh_mqtt.topo_topic, str, strlen(str), 0, 0);
    MDF_FREE(str);
    ret = MDF_OK;
_no_mem:
    cJSON_Delete(topolog);
    return ret;
}

mdf_err_t mesh_mqtt_write(uint8_t *addr, const char *data, size_t size, mesh_mqtt_publish_data_type_t type)
{
    MDF_PARAM_CHECK(addr);
    MDF_PARAM_CHECK(data);
    MDF_ERROR_CHECK(type >= MESH_MQTT_DATA_TYPE_MAX, MDF_ERR_INVALID_ARG, "Unknow data type");
    MDF_ERROR_CHECK(g_mesh_mqtt.client == NULL, MDF_ERR_INVALID_STATE, "MQTT client has not started");

    mdf_err_t ret = MDF_FAIL;

    char mac_str[13];
    mlink_mac_hex2str(addr, mac_str);

 if(type==MESH_MQTT_DATA_JSON)
{           
           // cJSON *obj = cJSON_CreateObject();
          //  MDF_ERROR_GOTO(obj == NULL, _no_mem, "Create JSON failed");
          //  cJSON *src_addr = cJSON_AddStringToObject(obj, "addr", mac_str);
           // MDF_ERROR_GOTO(src_addr == NULL, _no_mem, "Add string to JSON failed");
           // char *buffer = MDF_MALLOC(size + 1);
          //  MDF_ERROR_GOTO(buffer == NULL, _no_mem, "Allocate mem failed");
         //   memcpy(buffer, data, size);
          //  buffer[size] = '\0';
            //cJSON_AddStringToObject(obj, "type", "json");
           // cJSON *src_data = cJSON_AddRawToObject(obj, "data", buffer);
           // MDF_FREE(buffer);
           // MDF_ERROR_GOTO(src_data == NULL, _no_mem, "Add data to JSON failed");
           // char *payload = cJSON_PrintUnformatted(obj);
            //MDF_ERROR_GOTO(payload == NULL, _no_mem, "Print JSON failed");
            esp_mqtt_client_publish(g_mesh_mqtt.client, g_mesh_mqtt.publish_topic, data, size, 0, 0);
           // MDF_FREE(payload);
            ret = MDF_OK;
           // _no_mem:
           // cJSON_Delete(obj);
        }
        
else if(type==MESH_MQTT_DATA_RESPOND)
{
            // {"function":"onoff","data":"1","dest_adr":"246f28af3540"} example
            cJSON *json_data=NULL, *json_function=NULL,*json_adr=NULL;
            cJSON *obj= cJSON_Parse(data);

             MDF_ERROR_GOTO(obj == NULL, _exit_respond, "Parse JSON error");
            
            json_data=cJSON_GetObjectItemCaseSensitive(obj, "data");
            json_function=cJSON_GetObjectItemCaseSensitive(obj, "function");
            json_adr=cJSON_GetObjectItemCaseSensitive(obj, "dest_adr");

            MDF_ERROR_GOTO(json_data==NULL||json_function==NULL||json_adr==NULL, _exit_respond, "Parse JSON parsing error");
           // MDF_LOGE("json d:%s f:%s    adr:%s", json_data->valuestring,json_function->valuestring,json_adr->valuestring);


            char buffer_topic[100];
            int topic_size=strlen(g_mesh_mqtt.respond_topic)+strlen(json_function->valuestring)+strlen(json_adr->valuestring);
            if(topic_size>100){MDF_LOGE("failed to allocate topic buffer");strcpy(buffer_topic,"gBridge/test");}
            else snprintf(buffer_topic, topic_size, g_mesh_mqtt.respond_topic,json_function->valuestring,json_adr->valuestring);
            char *buffer = (char *)MDF_MALLOC(size);
            if(buffer==NULL){
                MDF_LOGE("failed to allocate buffer");
                return ret;
            }

            MDF_ERROR_GOTO(buffer==NULL, _exit_respond, "Parse JSON buffer allocation error");
            memcpy(buffer, data, size);
            esp_mqtt_client_publish(g_mesh_mqtt.client, buffer_topic, json_data->valuestring, strlen(json_data->valuestring), 0, 0);
            MDF_FREE(buffer);

            ret = MDF_OK;
_exit_respond:
            cJSON_Delete(obj);
}

    return ret;
}

mdf_err_t mesh_mqtt_read(mqtt_data_t **request, TickType_t wait_ticks)
{
    MDF_PARAM_CHECK(request);
    MDF_ERROR_CHECK(g_mesh_mqtt.client == NULL, MDF_ERR_INVALID_STATE, "MQTT client has not started");

    if (xQueueReceive(g_mesh_mqtt.queue, request, wait_ticks) != pdPASS) {
        return MDF_ERR_TIMEOUT;
    }

    return MDF_OK;
}

mdf_err_t mesh_mqtt_start(char *url, char *topic, size_t topicsize)
{
    MDF_PARAM_CHECK(url);
    MDF_ERROR_CHECK(g_mesh_mqtt.client != NULL, MDF_ERR_INVALID_STATE, "MQTT client is already running");

    esp_mqtt_client_config_t mqtt_cfg = {
        .uri = url,
        .event_handle = mqtt_event_handler,
        // .client_cert_pem = (const char *)client_cert_pem_start,
        // .client_key_pem = (const char *)client_key_pem_start,
    };
    MDF_ERROR_ASSERT(esp_read_mac(g_mesh_mqtt.addr, ESP_MAC_WIFI_STA));

    if(strlen(topic)<30&&strlen(topic)>3)strcpy(g_mesh_mqtt.topic_begin,topic);
    else {strcpy(g_mesh_mqtt.topic_begin,"gBridge/test/");
    MDF_LOGE("topic too long using default topic: gBridge/test/...");
    }

    strcpy(g_mesh_mqtt.sub_topic,g_mesh_mqtt.topic_begin);
    strcpy(g_mesh_mqtt.respond_topic,g_mesh_mqtt.topic_begin);
    strcpy(g_mesh_mqtt.publish_topic,g_mesh_mqtt.topic_begin);
    strcpy(g_mesh_mqtt.topo_topic,g_mesh_mqtt.topic_begin);

    strcat(g_mesh_mqtt.sub_topic,"+/+/set");
    strcat(g_mesh_mqtt.respond_topic,"%s/%s/status");
    strcat(g_mesh_mqtt.publish_topic,"heartbeat");
    strcat(g_mesh_mqtt.topo_topic,"topo");

    g_mesh_mqtt.queue = xQueueCreate(3, sizeof(mqtt_data_t *));
    g_mesh_mqtt.client = esp_mqtt_client_init(&mqtt_cfg);
    MDF_ERROR_ASSERT(esp_mqtt_client_start(g_mesh_mqtt.client));

    return MDF_OK;
}

mdf_err_t mesh_mqtt_stop()
{
    MDF_ERROR_CHECK(g_mesh_mqtt.client == NULL, MDF_ERR_INVALID_STATE, "MQTT client has not been started");
    mqtt_data_t *item;

    if (uxQueueMessagesWaiting(g_mesh_mqtt.queue)) {
        if (xQueueReceive(g_mesh_mqtt.queue, &item, 0)) {
            MDF_FREE(item);
        }
    }

    vQueueDelete(g_mesh_mqtt.queue);
    g_mesh_mqtt.queue = NULL;

    esp_mqtt_client_stop(g_mesh_mqtt.client);
    esp_mqtt_client_destroy(g_mesh_mqtt.client);
    g_mesh_mqtt.client = NULL;

    return MDF_OK;
}
