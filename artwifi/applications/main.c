/*
 * Copyright (c) 2006-2020, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2020-09-02     RT-Thread    first version
 */

#include <rtthread.h>
#include <rtdevice.h>
#include "drv_common.h"
#include "ui.h"
#include "rtthread.h"
#include "mqtt_api.h"
#define LED_PIN GET_PIN(I, 8)


static rt_thread_t al_send_thread;
static rt_thread_t al_recv_thread;

void *mqtt_client;

extern void wlan_autoconnect_init(void);


/* MQTT客户端事件回调函数 */
static void event_handler(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    switch (msg->event_type)
    {
        case IOTX_MQTT_EVENT_DISCONNECT:
            rt_kprintf("MQTT client disconnected\n");
            break;
        case IOTX_MQTT_EVENT_SUBCRIBE_SUCCESS:
            rt_kprintf("MQTT client subscribe success\n");
            break;
        case IOTX_MQTT_EVENT_SUBCRIBE_NACK:
            rt_kprintf("MQTT client subscribe fail\n");
            break;
        case IOTX_MQTT_EVENT_PUBLISH_SUCCESS:
            rt_kprintf("MQTT client publish success\n");
            break;
        case IOTX_MQTT_EVENT_PUBLISH_NACK:
            rt_kprintf("MQTT client publish fail\n");
            break;
        case IOTX_MQTT_EVENT_PUBLISH_RECEIVED://在IOT_MQTT_Yield函数接收到数据后 设置该标志位触发该事件
            rt_kprintf("MQTT client received message\n");
            break;
        default:
            break;
    }
}


void Ali_Topic_Handle(void *pcontext, void *pclient, iotx_mqtt_event_msg_pt msg)
{
    // 处理MQTT主题消息
    iotx_mqtt_topic_info_pt topic_info = (iotx_mqtt_topic_info_pt)msg->msg;
    rt_kprintf("recv message:%s\n", topic_info->ptopic);

}


void Ali_Init()
{
    static iotx_mqtt_param_t       mqtt_params;

    memset(&mqtt_params, 0x0, sizeof(mqtt_params));
    mqtt_params.handle_event.h_fp = event_handler;

    mqtt_client = IOT_MQTT_Construct(&mqtt_params);
    if (mqtt_client == NULL)
    {
        rt_kprintf("MQTT client create failed\n");
        return;
    }
    int rc = IOT_MQTT_CheckStateNormal(mqtt_client);
    if(rc == 1) rt_kprintf("MQTT client status is success\n",rc);
    else rt_kprintf("MQTT client status is failed,err: %d");
}


void Ali_Send(void* parameter)
{
    char *topic = "/sys/if15bhaF5KF/art-pi/thing/event/property/post";
//    char *payload = "{"
//            "\"id\":\"123\","
//            "\"version\":\"1.0\","
//            "\"params\":{"
//                "\"GeoLocation\":{"
//                    "\"Longitude\":1.12,"
//                    "\"Latitude\":1.13"
//                    "},"
//                "\"demo\":\"1000\""
//                 "},"
//            "\"method\":\"thing.event.property.post\""
//            "}";
    char *payload = "{"
            "\"id\":\"123\","
                        "\"version\":\"1.0\","
                        "\"params\":{"
                            "\"Longitude\":1,"
                            "\"Latitude\":1,"
                            "\"demo\":\"1005\""
                             "},"
                        "\"method\":\"thing.event.property.post\""
            "}";
    int rc;
    iotx_mqtt_topic_info_t mesg;
    mesg.dup= 0 ;
    mesg.packet_id=0;
    mesg.payload=payload;
    mesg.payload_len= strlen(payload);
    mesg.ptopic= topic;
    mesg.qos=IOTX_MQTT_QOS0;
    mesg.retain= 0 ;
    mesg.topic_len =strlen(topic);
        rc = IOT_MQTT_Publish(mqtt_client, topic, &mesg);

        if (rc < 0)
        {
            rt_kprintf("publish failed, res = %d", rc);
            return;
        }
        rt_thread_delay(1000);
}


void Ali_Recv(void* parameter)
{
    int rc;
    char *topic = "/sys/if15bhaF5KF/art-pi/thing/event/property/post_reply";
    rc =IOT_MQTT_Subscribe(mqtt_client, topic, IOTX_MQTT_QOS0, Ali_Topic_Handle, NULL);
    if (rc < 0)
    {
        rt_kprintf("MQTT subscribe failed: %d\n", rc);
        return;
    }
    while(1)
    {
        IOT_MQTT_Yield(mqtt_client,100);
    }
}

int ALI(void)
{
    /* set LED0 pin mode to output */
    Ali_Init();
    al_send_thread = rt_thread_create("ali_send", Ali_Send,NULL, 4096, 10, 20);
    al_recv_thread = rt_thread_create("ali_recv", Ali_Recv,NULL, 4096, 10, 20);

    rt_thread_startup(al_send_thread);
    rt_thread_startup(al_recv_thread);

    return 0;

}

int main(void)
{
    rt_uint32_t count = 1;
    rt_pin_mode(LED_PIN, PIN_MODE_OUTPUT);

    /* init Wi-Fi auto connect feature */
    wlan_autoconnect_init();
    /* enable auto reconnect on WLAN device */
    rt_wlan_config_autoreconnect(RT_TRUE);
    ui_init();
    while(count++)
    {
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_HIGH);
        rt_thread_mdelay(500);
        rt_pin_write(LED_PIN, PIN_LOW);
    }
    return RT_EOK;
}

#include "stm32h7xx.h"
static int vtor_config(void)
{
    /* Vector Table Relocation in Internal QSPI_FLASH */
    SCB->VTOR = QSPI_BASE;
    return 0;
}
INIT_BOARD_EXPORT(vtor_config);
MSH_CMD_EXPORT(ALI, AL);

