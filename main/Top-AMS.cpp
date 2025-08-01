#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#include <stdio.h>
#include <stdint.h>
#include <stddef.h>

#include "esp_system.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_netif.h"

#include "esp_log.h"
#include "mqtt_client.h"
#include "esp_tls.h"
#include "esp_ota_ops.h"
#include <sys/param.h>
#include "esp_wifi.h"
#include "ble_set.h"
#include <stdlib.h>

#include "include/tools.hpp"
#include "include/espIO.hpp"
#include "include/bambu.hpp"
#include "include/ArduinoJson.hpp"
#include "include/smartwifi.h"

#if __has_include("config.hpp")
#include "config.hpp"
#else
#endif
#include <portmacro.h>

TaskHandle_t Task1_handle;
TaskHandle_t Task2_handle;
TaskHandle_t Task3_handle;
TaskHandle_t Task4_handle;
TaskHandle_t Task5_handle; // 主程序
TaskHandle_t Task6_handle; // debug


std::string gcode_state;
int z=0;//没连上
int hw_switch = 0;
int print_error;
int state = 0;
int work = 0;
int callback_count = 0;
static uint32_t bed_target_temper = 1;
int sequence_id = -1;
int ams_status = -1;
// std::atomic<bool> pause_lock{false}; // 暂停锁
// int old_extruder = 0;
int new_extruder1 = 0;
int extruder = 1; // 1-16,初始通道默认为1
int vd = 0;       // 微动信号量

inline constexpr int 正常 = 0;
inline constexpr int 退料完成需要退线 = 260;
inline constexpr int 退料完成 = 0; // 同正常
inline constexpr int 进料检查 = 262;
inline constexpr int 进料冲刷 = 263; // 推测
inline constexpr int 进料完成 = 768;


void jin(int A,int B)
{
 esp::gpio_out(config::motors[A- 1].forward, true);
                vTaskDelay(B / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
                esp::gpio_out(config::motors[A - 1].forward, false);
               // esp::gpio_out(config::motors[A- 1].backward, true);
                //vTaskDelay(60 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
               // esp::gpio_out(config::motors[A - 1].backward, false);
}

void tui(int A,int B)
{
 esp::gpio_out(config::motors[A- 1].backward, true);
                vTaskDelay(B / portTICK_PERIOD_MS); // 微动退料的时间可以自己改
                esp::gpio_out(config::motors[A - 1].backward, false);
                //esp::gpio_out(config::motors[A- 1].forward, true);
               // vTaskDelay(30 / portTICK_PERIOD_MS); // 微动退料的时间可以自己改
                //esp::gpio_out(config::motors[A - 1].forward, false);
}



void Task1(void *param)
{
    esp::gpio_set_in(config::input);
    int a = 0;
    while (true)
    {
        int level = gpio_get_level(config::input); // 设定4号口为缓冲驱动
                                                   // fpr("a的值:",a);
        if (vd == 0)
        {

            if (level == 0)
            {
                xSemaphoreTake(lock_x, portMAX_DELAY);
                int now_extruder = extruder;
                xSemaphoreGive(lock_x);
                jin(now_extruder, 200);
                if (a > 0)
                {
                    a = 0;
                }
            }
            if (a > 500 && gcode_state == "RUNNING" && strcmp(ble_read13, "开") == 0)
            {

                xSemaphoreTake(lock_x, portMAX_DELAY);
                int now_extruder = extruder;
                xSemaphoreGive(lock_x);
                 jin(now_extruder, 500);
                a = 0;
            }

            if (gcode_state == "PREPARE")
            {
                a = -13000;
            }
            if (a > 50000)
            {
                a = 0;
            }

            else
            {
                a++;
            }
        }
        vTaskDelay(20 / portTICK_PERIOD_MS); // 延时1000ms=1s,使系统执行其他任务删了就寄了
    }
} // 微动缓冲程序

void publish(esp_mqtt_client_handle_t client, const std::string &msg)
{

    mstd::delay(2s);
    fpr("发送消息:", msg);
    int msg_id = esp_mqtt_client_publish(client, topic_publish, msg.c_str(), msg.size(), 0, 0);
    if (msg_id < 0)
        fpr("发送失败");
        
        
    else
        fpr("发送成功,消息id=", msg_id);
    // fpr(TAG, "binary sent with msg_id=%d", msg_id);

    mstd::delay(2s); // 发布函数
}

void delay(int a, esp_mqtt_client_handle_t client)


{
    for (size_t i = 0; i < 100; i++)
    {
        if (ams_status == a)
        {
            // fpr("延时结束", ams_status);
            break;
        }
        if (i % 15 == 0 && i >= 15)
        {
            publish(client, bambu::msg::get_status);
        }
        if (i > 90)
        {
            i = 16;
        }

        // fpr("延时", ams_status);
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 延时1000ms=1s,使系统执行其他任务删了就寄了
    }
}



void xuliao(esp_mqtt_client_handle_t client)
{ 
     
    xSemaphoreGive(ble_read_x);
    publish(client, bambu::msg::runGcode(std::string("M109 S250")));
   
    //fpr("自动续料中");
    write_nvs(ble_read7, BLE_NAMSPACE, BLE_READ7_KEY);
    int old_extruder = atoi(ble_read7);

    xSemaphoreTake(lock_x, portMAX_DELAY);
    str_save(ble_read12, "自动续料中");
    extruder = atoi(ble_read7);
    xSemaphoreGive(lock_x);

    // fpr(extruder);
    //fpr("电机动");
    esp::gpio_out(config::motors[old_extruder - 1].forward, true);

    for (size_t i = 0; i < 240; i++)
    {
        if (hw_switch == 1)
        {
            break;
        }
        if (i % 5 == 0 && i >= 15)
        {
             //fpr("循环");
            publish(client, bambu::msg::get_status);
            esp::gpio_out(config::motors[old_extruder - 1].forward, false);

            esp::gpio_out(config::motors[old_extruder - 1].backward, true);
            mstd::delay(300ms);
            esp::gpio_out(config::motors[old_extruder - 1].backward, false);
            esp::gpio_out(config::motors[old_extruder - 1].forward, true);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
        str_save(ble_read12, "还没到");
    }

    esp::gpio_out(config::motors[old_extruder - 1].forward, false);
    str_save(ble_read12, "到了");
    fpr(hw_switch);
    if (hw_switch == 1)
    {

        str_save(ble_read12, "进料");
        fpr("rrr");
        print_error = 0;
        publish(client, bambu::msg::print_resume); // 继续打印
        vTaskDelay(3000 / portTICK_PERIOD_MS);
        jin(old_extruder, 4000);
        str_save(ble_read12, "ams正常");
        // fpr(extruder);
    }
    else
    {
        fpr("error");
        str_save(ble_read12, "卡料");
    }
}

void qieliao(esp_mqtt_client_handle_t client, char a[20])
{ // 切换耗材

    str_save(ble_read14, "空");
    str_save(ble_read12, "切换耗材中");
    publish(client, bambu::msg::get_status);
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    if (hw_switch == 0)
    {

        xSemaphoreTake(lock_x, portMAX_DELAY);
        extruder = atoi(a);
        xSemaphoreGive(lock_x);

        str_save(ble_read7, a);
        write_nvs(ble_read7, BLE_NAMSPACE, BLE_READ7_KEY);
        esp::gpio_out(config::motors[atoi(a) - 1].forward, true);
        publish(client, bambu::msg::load);

        for (size_t i = 0; i < 240; i++)
        {
            if (hw_switch == 1)
            {
                break;
            }

            if (i % 10 == 0 && i >= 20)
            {
                publish(client, bambu::msg::get_status);

                esp::gpio_out(config::motors[atoi(a) - 1].forward, false);

                esp::gpio_out(config::motors[atoi(a) - 1].backward, true);
                mstd::delay(500ms);
                esp::gpio_out(config::motors[atoi(a) - 1].backward, false);
                esp::gpio_out(config::motors[atoi(a) - 1].forward, true);
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            str_save(ble_read12, "进料中");
        }
        if (hw_switch == 1)

        {
            esp::gpio_out(config::motors[atoi(a) - 1].forward, false);

            str_save(ble_read12, "料线到达");
            int A = 0;
            for (size_t i = 0; i < 100; i++)
            {

                if (ams_status == 262)
                {
                    if (A == 0)
                    {
                        

                        jin(atoi(a), 3000);
                        A = 1;
                    }

                    vTaskDelay(2000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
                    publish(client, bambu::msg::click_done);
                }
                if (ams_status == 263)
                {
                    if (A == 1)
                    {
                     jin(atoi(a), 3000);
                        A = 2;
                    }
                    vTaskDelay(2000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改

                    publish(client, bambu::msg::click_done);
                }
                if (ams_status == 进料完成 || ams_status == 正常)
                {
                    break;
                }
                if (i % 5 == 0 && i >= 35)
                {
                    publish(client, bambu::msg::get_status);
                }
                vTaskDelay(1000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            }
            publish(client, bambu::msg::runGcode(std::string("M109 S200")));
            str_save(ble_read12, "ams正常");
        }
        else
        {
            fpr("error");
            esp::gpio_out(config::motors[atoi(a) - 1].forward, false);
            str_save(ble_read12, "卡料");
        }
    }

    else if (hw_switch == 1)
    {

        xSemaphoreTake(lock_x, portMAX_DELAY);
        int old_extruder = extruder;
        xSemaphoreGive(lock_x);

        str_save(ble_read12, "退出耗材");
        tui(old_extruder, tm);
        publish(client, bambu::msg::runGcode(std::string("M109 S250")));
        vd = 1;
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        publish(client, bambu::msg::uload);
        fpr("发送了退料命令,等待退料完成");
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        delay(退料完成需要退线, client); // 等待退料完成
        esp::gpio_out(config::motors[old_extruder - 1].backward, true);
        mstd::delay(std::chrono::seconds(atoi(ble_read17))); // 明确使用秒为单位
        esp::gpio_out(config::motors[old_extruder - 1].backward, false);
        delay(退料完成, client); // 等待退料完成
        xSemaphoreTake(lock_x, portMAX_DELAY);
        extruder = atoi(a);
        xSemaphoreGive(lock_x);

        str_save(ble_read7, a);
        write_nvs(ble_read7, BLE_NAMSPACE, BLE_READ7_KEY);
        esp::gpio_out(config::motors[atoi(a) - 1].forward, true);
        publish(client, bambu::msg::load);
        for (size_t i = 0; i < 240; i++)
        {
            if (i % 5 == 0 && i >= 15)
            {
                publish(client, bambu::msg::get_status);

                esp::gpio_out(config::motors[atoi(a) - 1].forward, false);

                esp::gpio_out(config::motors[atoi(a) - 1].backward, true);
                mstd::delay(500ms);
                esp::gpio_out(config::motors[atoi(a) - 1].backward, false);
                esp::gpio_out(config::motors[atoi(a) - 1].forward, true);
            }

            if (hw_switch == 1)
            {
                break;
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            str_save(ble_read12, "进料中");
        }
        vd = 0;
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        esp::gpio_out(config::motors[atoi(a) - 1].forward, false);

        if (hw_switch == 1)
        {

            str_save(ble_read12, "料线到达");
            int A = 0;
            for (size_t i = 0; i < 100; i++)
            {

                if (ams_status == 262)
                {
                    if (A == 0)
                    {
                       jin(atoi(a), 3000);
                        A = 1;
                    }

                    vTaskDelay(2000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
                    publish(client, bambu::msg::click_done);
                }
                if (ams_status == 263)
                {
                    if (A == 1)
                    {
                        jin(atoi(a), 3000);
                        A = 2;
                    }
                    vTaskDelay(2000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改

                    publish(client, bambu::msg::click_done);
                }
                if (ams_status == 进料完成 || ams_status == 正常)
                {
                    break;
                }

                if (i % 5 == 0 && i >= 35)
                {
                    publish(client, bambu::msg::get_status);
                }

                vTaskDelay(1000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            }

            publish(client, bambu::msg::runGcode(std::string("M109 S200")));
            str_save(ble_read12, "ams正常");
        }
        else
        {
            fpr("error");
            str_save(ble_read12, "卡料");
        }
    }
}

void callback_fun(esp_mqtt_client_handle_t client, const std::string &json)
{ // 接受到信息的回调

    using namespace ArduinoJson;
    JsonDocument doc;
    DeserializationError error = deserializeJson(doc, json);

    bed_target_temper = doc["print"]["bed_target_temper"] | bed_target_temper;
    gcode_state = doc["print"]["gcode_state"] | gcode_state;
    print_error = doc["print"]["print_error"] | print_error;
    hw_switch = doc["print"]["hw_switch_state"] | hw_switch;
    ams_status = doc["print"]["ams_status"] | ams_status;
    state++;
    callback_count++;
  
    //fpr("打印状态", print_error);
   // fpr("耗材状态", hw_switch);
    //fpr("ams状态", bed_target_temper);



    if (callback_count > 90000)
    {
        callback_count = 6;
    }

    if (state > 90000)
    {
        state = 0;
    }

    if (state > 30 && gcode_state != "PAUSE")
    {
        publish(client, bambu::msg::get_status);
    
        state = 0;
    }

    if (state > 120 && gcode_state == "PAUSE")
    {
        publish(client, bambu::msg::get_status);
     
        state = 0;
    }

    if (bed_target_temper > 0 && bed_target_temper < 17)
    { // 读到的温度是通道
        if (gcode_state == "PAUSE")
        {
            xSemaphoreTake(lock_x, portMAX_DELAY);

            if (extruder != bed_target_temper)
            {
                xSemaphoreGive(lock_x);
            xSemaphoreTake(ble_read_x, portMAX_DELAY);
            str_save(ble_read12,"换色准备"); // 蓝牙输出debug
            xSemaphoreGive(ble_read_x);
                new_extruder1 = bed_target_temper;
                work = 1;                                  // 唤醒
                bed_target_temper = bed_target_temper_max; // 必要,恢复温度后,MQTT的更新可能不
            }
            if (extruder == bed_target_temper && callback_count > 5)
            {
                xSemaphoreGive(lock_x);
                 xSemaphoreTake(ble_read_x, portMAX_DELAY);
            str_save(ble_read12,"同一耗材"); // 蓝牙输出debug
            xSemaphoreGive(ble_read_x);
                publish(client, bambu::msg::runGcode(std::string("M190 S") + std::to_string(bed_target_temper_max))); // 恢复原来的热床温度
                mstd::delay(1000ms);                                                                                  // 确保暂停动作完成
                publish(client, bambu::msg::print_resume);                                                            // 无须换料
                callback_count = -30;
                bed_target_temper = bed_target_temper_max; // 必要,恢复温度后,MQTT的更新可能不及时
            }
        }
    }
    else
    {
        if (bed_target_temper == 0)
        {
            bed_target_temper_max = 0; // 打印结束or冷打印版
            std::string ble_read18_str = std::to_string(bed_target_temper_max);
            strncpy(ble_read18, ble_read18_str.c_str(), sizeof(ble_read18) - 1);
            ble_read18[sizeof(ble_read18) - 1] = '\0';
            write_nvs(ble_read18, BLE_NAMSPACE, BLE_READ18_KEY);
        }
        else

        {

            bed_target_temper_max = std::max(bed_target_temper, bed_target_temper_max); // 不同材料可能底板温度不一样,这里选择维持最高的
            std::string ble_read18_str = std::to_string(bed_target_temper_max);
            strncpy(ble_read18, ble_read18_str.c_str(), sizeof(ble_read18) - 1);
            ble_read18[sizeof(ble_read18) - 1] = '\0';
            write_nvs(ble_read18, BLE_NAMSPACE, BLE_READ18_KEY);
        }
    }

} // callback

// mqtt事件循环处理(call_back)
void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    auto TAG = "MQTT ";
    fpr("\n事件分发");
    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
    esp_mqtt_client_handle_t client = event->client;
    int msg_id = -1;
    switch (esp_mqtt_event_id_t(event_id))
    {
    case MQTT_EVENT_CONNECTED:
        fpr(TAG, "MQTT_EVENT_CONNECTED（MQTT连接成功）");
        xSemaphoreTake(ble_read_x, portMAX_DELAY);
        str_save(ble_read12, "ams正常");
        xSemaphoreGive(ble_read_x);
        msg_id = esp_mqtt_client_subscribe(client, topic_subscribe, 1);
        fpr(TAG, "发送订阅成功，msg_id=", msg_id);
   
        break;
    case MQTT_EVENT_DISCONNECTED:
        fpr(TAG, "MQTT_EVENT_DISCONNECTED（MQTT断开连接）");
         xSemaphoreTake(ble_read_x, portMAX_DELAY);
            str_save(ble_read12,"ams断开"); // 蓝牙输出debug
            xSemaphoreGive(ble_read_x);
        fpr(ble_read9);
        


        break;
    case MQTT_EVENT_BEFORE_CONNECT:
        fpr("连接前");
        break;
    case MQTT_EVENT_SUBSCRIBED:
        fpr(TAG, "MQTT_EVENT_SUBSCRIBED（MQTT订阅成功），msg_id=", event->msg_id);
             vTaskDelay(3000 / portTICK_PERIOD_MS);
          publish(client, bambu::msg::get_status);

        break;
    case MQTT_EVENT_UNSUBSCRIBED:
        fpr(TAG, "MQTT_EVENT_UNSUBSCRIBED（MQTT取消订阅成功），msg_id=", event->msg_id);

         xSemaphoreTake(ble_read_x, portMAX_DELAY);
            str_save(ble_read12,"ams断开"); // 蓝牙输出debug
            xSemaphoreGive(ble_read_x);
        break;
    case MQTT_EVENT_PUBLISHED:
        fpr(TAG, "MQTT_EVENT_PUBLISHED（MQTT消息发布成功），msg_id=", event->msg_id);

        break;
    case MQTT_EVENT_DATA:
        // fpr(TAG,"MQTT_EVENT_DATA（接收到MQTT消息）");
        // printf("主题=%.*s\r\n",event->topic_len,event->topic);

        printf("%.*s\r\n", event->data_len, event->data);
        callback_fun(client, std::string(event->data));
            xSemaphoreTake(ble_read_x, portMAX_DELAY);
            z=0;
             if (strcmp(ble_read12, "ams超时") == 0)
   {
       str_save(ble_read12,"ams正常"); // 蓝牙输出debug
   }
            
            xSemaphoreGive(ble_read_x);

        break;
    case MQTT_EVENT_ERROR:

        fpr(TAG, "MQTT_EVENT_ERROR（MQTT事件错误）");
        if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT)
        {
             xSemaphoreTake(ble_read_x, portMAX_DELAY);
            str_save(ble_read12,"断开"); // 蓝牙输出debug
            xSemaphoreGive(ble_read_x);
            fpr(TAG, "从esp-tls报告的最后错误代码：", event->error_handle->esp_tls_last_esp_err);
            fpr(TAG, "TLS堆栈最后错误号：", event->error_handle->esp_tls_stack_err);
            fpr(TAG, "最后捕获的errno：", event->error_handle->esp_transport_sock_errno,
                strerror(event->error_handle->esp_transport_sock_errno));
        }
        else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED)
        {
            fpr(TAG, "连接被拒绝错误：", event->error_handle->connect_return_code);
             xSemaphoreTake(ble_read_x, portMAX_DELAY);
            str_save(ble_read12,"断开"); // 蓝牙输出debug
            xSemaphoreGive(ble_read_x);
        }
        else
        {
            fpr(TAG, "未知的错误类型：", event->error_handle->error_type);
             xSemaphoreTake(ble_read_x, portMAX_DELAY);
            str_save(ble_read12,"断开"); // 蓝牙输出debug
            xSemaphoreGive(ble_read_x);
        }
        break;
    default:
        fpr(TAG, "其他事件id:", event->event_id);
        str_save(ble_read12,"断开"); // 蓝牙输出debug
        break;
    }
} // mqtt_event_handler

//{"print":{"nozzle_temper":214.5,"bed_temper":36.03125,"mc_print_stage":"3","print_error":50364437,"wifi_signal":"-36dBm","gcode_state":"PAUSE","home_flag":847201687,"mc_print_sub_stage":0,"command":"push_status","msg":1,"sequence_id":"767"}}
//{"print":{"nozzle_temper":227.78125,"bed_temper":32.75,"mc_print_stage":"3",断料"hw_switch_state":0,"print_error":50364437,"wifi_signal":"-40dBm","gcode_state":"PAUSE","stg_cur":0,"home_flag":847201687,"command":"push_status","msg":1,"sequence_id":"843"}}

[[nodiscard]] esp_mqtt_client_handle_t mqtt_app_start()
{
    esp_mqtt_client_config_t mqtt_cfg{};
    // Broker 配置
    mqtt_cfg.broker.address.uri = mqtt_server;
    mqtt_cfg.broker.verification.skip_cert_common_name_check = true;
    mqtt_cfg.credentials.username = mqtt_username;
    mqtt_cfg.credentials.authentication.password = ble_read9;

    // 关键优化配置
    mqtt_cfg.buffer.size = 4096;                  // 增大接收缓冲区
    mqtt_cfg.buffer.out_size = 2048;              // 发送缓冲区
    mqtt_cfg.network.reconnect_timeout_ms = 5000; // 5秒重连
    mqtt_cfg.task.stack_size = 6144;              // 增大任务栈
    mqtt_cfg.task.priority = 5;                   // 提高任务优先级

    fpr("[APP] Free memory: ", esp_get_free_heap_size(), "bytes");
    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, MQTT_EVENT_ANY, mqtt_event_handler, nullptr);
    esp_mqtt_client_start(client);
    return client;
}

void Task2(void *arg) // 同步通道
{                     // 吧蓝牙读到的通道信息同步

    while (true)
    {

        if (F == 1)
        {
            xSemaphoreTake(lock_x, portMAX_DELAY);

            extruder = atoi(ble_read7);
            F = 0;
            xSemaphoreGive(lock_x);
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // 延迟1秒
    }
}

void Task3(void *param) // 手机交互
{
    // 手机app控制进退料esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)param;
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)param;
    fpr("Task3");
    while (true)
    {

        if (strcmp(ble_read14, "1退") == 0)
        {
            // publish(client, bambu::msg::runGcode(std::string("M190 S") + std::to_string(bed_target_temper_max) + std::string("\nM211 S \nM211 X1 Y1 Z1\nM1002 push_ref_mode\nG91 \nG1 Z-1.0 F900\nM1002 pop_ref_mode\nM211 R\nG1 E-20 F900\nM109 S250\n"))); // 恢复原来的热床温
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[0].backward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[0].backward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "1进") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[0].forward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[0].forward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "2退") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[1].backward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[1].backward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "2进") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[1].forward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[1].forward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "3退") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[2].backward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[2].backward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "3进") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[2].forward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[2].forward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "4退") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[3].backward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[3].backward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "4进") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[3].forward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[3].forward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "5退") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[4].backward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[4].backward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "5进") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[4].forward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[4].forward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "6退") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[5].backward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[5].backward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "6进") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[5].forward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[5].forward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "7退") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[6].backward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[6].backward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "7进") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[6].forward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[6].forward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "8退") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[7].backward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[7].backward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "8进") == 0)
        {
            str_save(ble_read14, "空");
            esp::gpio_out(config::motors[7].forward, true);
            vTaskDelay(8000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            esp::gpio_out(config::motors[7].forward, false);
            // fpr(extruder);
        }

        if (strcmp(ble_read14, "1切") == 0)
        {

            qieliao(client, "1");
        }

        if (strcmp(ble_read14, "2切") == 0)
        {

            qieliao(client, "2");
        }

        if (strcmp(ble_read14, "3切") == 0)
        {

            qieliao(client, "3");
        }

        if (strcmp(ble_read14, "4切") == 0)
        {

            qieliao(client, "4");
        }

        if (strcmp(ble_read14, "5切") == 0)
        {

            qieliao(client, "5");
        }

        if (strcmp(ble_read14, "6切") == 0)
        {

            qieliao(client, "6");
        }

        if (strcmp(ble_read14, "7切") == 0)
        {

            qieliao(client, "7");
        }

        if (strcmp(ble_read14, "8切") == 0)
        {

            qieliao(client, "8");
        }

        if (strcmp(ble_read14, "退料") == 0)
        {

            str_save(ble_read14, "空");
            str_save(ble_read12, "退料");
            publish(client, bambu::msg::get_status);
            vTaskDelay(3000 / portTICK_PERIOD_MS);

            if (hw_switch == 1)
            {
                xSemaphoreTake(lock_x, portMAX_DELAY);
                int old_extruder = extruder;
                xSemaphoreGive(lock_x);

                // int old_extruder = extruder;

                publish(client, bambu::msg::runGcode(std::string("M109 S250")));
                vTaskDelay(1000 / portTICK_PERIOD_MS);
                vTaskSuspend(Task1_handle); // 关闭前进微动防止意外
                tui(old_extruder, 300);
                publish(client, bambu::msg::uload);

                delay(退料完成需要退线, client); // 等待退料完成
                esp::gpio_out(config::motors[old_extruder - 1].backward, true);
                mstd::delay(std::chrono::seconds(atoi(ble_read17))); // 明确使用秒为单位
                esp::gpio_out(config::motors[old_extruder - 1].backward, false);

                esp::gpio_out(config::motors[old_extruder - 1].forward, true);
                vTaskDelay(60 / portTICK_PERIOD_MS); // 延时1000ms=1s,使系统执行其他任务删了就寄了
                  esp::gpio_out(config::motors[old_extruder - 1].forward, false);
                delay(退料完成, client); // 等待退料完成

                vTaskResume(Task1_handle); // 恢复微动
                str_save(ble_read12, "耗材已退出");
            }
            else
            {

                fpr("error");
                str_save(ble_read12, "没有料");
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS); // 延时1000ms=1s,使系统执行其他任务删了就寄了
    }
}

void Task4(void *param) // 续料

{
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)param;
    fpr("Task3");

    while (1)
    {

        if (print_error == 50364437 ) // 自动续料
        {
            fpr("续料开始");
            xSemaphoreTake(ble_read_x, portMAX_DELAY);
           str_save(ble_read12, "缺料"); // 蓝牙输出debug
            xSemaphoreGive(ble_read_x);
          
            if (extruder == 1 && strcmp(ble_read1, "空") != 0)
            {
                xSemaphoreTake(ble_read_x, portMAX_DELAY);
                if (strcmp(ble_read1, ble_read2) == 0)
                {
                    str_save(ble_read1, "空");
                    str_save(ble_read7, "2");
                    ble_nvs_write();
                    fpr("12");
                    xuliao(client);
                }
                else if (strcmp(ble_read1, ble_read3) == 0)
                {
                    str_save(ble_read1, "空");
                    str_save(ble_read7, "3");
                    ble_nvs_write();
                    fpr("13");
                    xuliao(client);
                }
                else if (strcmp(ble_read1, ble_read4) == 0)
                {
                    str_save(ble_read1, "空");
                    str_save(ble_read7, "4");
                    ble_nvs_write();
                    fpr("14");
                    xuliao(client);
                }
                else if (strcmp(ble_read1, ble_read5) == 0)
                {
                    str_save(ble_read1, "空");
                    str_save(ble_read7, "5");
                    ble_nvs_write();
                    fpr("15");
                    xuliao(client);
                }
                else if (strcmp(ble_read1, ble_read6) == 0)
                {
                    str_save(ble_read1, "空");
                    str_save(ble_read7, "6");
                    ble_nvs_write();
                    fpr("16");
                    xuliao(client);
                }
                else if (strcmp(ble_read1, ble_read15) == 0)
                {
                    str_save(ble_read1, "空");
                    str_save(ble_read7, "7");
                    ble_nvs_write();
                    fpr("17");
                    xuliao(client);
                }
                else if (strcmp(ble_read1, ble_read16) == 0)
                {
                    str_save(ble_read1, "空");
                    str_save(ble_read7, "8");
                    ble_nvs_write();
                    fpr("18");
                    xuliao(client);
                }
                else
                {
                    str_save(ble_read1, "空"); // 没料了自己手动换吧
                    fpr("请手动");
                    ble_nvs_write();
                    print_error = 0;
                }
                xSemaphoreGive(ble_read_x);
            }
            else if (extruder == 2 && strcmp(ble_read2, "空") != 0)
            {
                xSemaphoreTake(ble_read_x, portMAX_DELAY);
                if (strcmp(ble_read2, ble_read1) == 0)
                {
                    str_save(ble_read2, "空");
                    str_save(ble_read7, "1");
                    ble_nvs_write();
                    fpr("21");
                    xuliao(client);
                }
                else if (strcmp(ble_read2, ble_read3) == 0)
                {
                    str_save(ble_read2, "空");
                    str_save(ble_read7, "3");
                    ble_nvs_write();
                    fpr("23");
                    xuliao(client);
                }
                else if (strcmp(ble_read2, ble_read4) == 0)
                {
                    str_save(ble_read2, "空");
                    str_save(ble_read7, "4");
                    ble_nvs_write();
                    fpr("24");
                    xuliao(client);
                }
                else if (strcmp(ble_read2, ble_read5) == 0)
                {
                    str_save(ble_read2, "空");
                    str_save(ble_read7, "5");
                    ble_nvs_write();
                    fpr("25");
                    xuliao(client);
                }
                else if (strcmp(ble_read2, ble_read6) == 0)
                {
                    str_save(ble_read2, "空");
                    str_save(ble_read7, "6");
                    ble_nvs_write();
                    fpr("26");
                    xuliao(client);
                }
                else if (strcmp(ble_read2, ble_read15) == 0)
                {
                    str_save(ble_read2, "空");
                    str_save(ble_read7, "7");
                    ble_nvs_write();
                    fpr("27");
                    xuliao(client);
                }
                else if (strcmp(ble_read2, ble_read16) == 0)
                {
                    str_save(ble_read2, "空");
                    str_save(ble_read7, "8");
                    ble_nvs_write();
                    fpr("28");
                    xuliao(client);
                }
                else
                {
                    str_save(ble_read2, "空"); // 没料了自己手动换吧
                    fpr("请手动");
                    ble_nvs_write();
                    print_error = 0;
                }
                xSemaphoreGive(ble_read_x);
            }
            else if (extruder == 3 && strcmp(ble_read3, "空") != 0)
            {
                xSemaphoreTake(ble_read_x, portMAX_DELAY);
                if (strcmp(ble_read3, ble_read1) == 0)
                {
                    str_save(ble_read3, "空");
                    str_save(ble_read7, "1");
                    ble_nvs_write();
                    fpr("31");
                    xuliao(client);
                }
                else if (strcmp(ble_read3, ble_read2) == 0)
                {
                    str_save(ble_read3, "空");
                    str_save(ble_read7, "2");
                    ble_nvs_write();
                    fpr("32");
                    xuliao(client);
                }
                else if (strcmp(ble_read3, ble_read4) == 0)
                {
                    str_save(ble_read3, "空");
                    str_save(ble_read7, "4");
                    ble_nvs_write();
                    fpr("34");
                    xuliao(client);
                }
                else if (strcmp(ble_read3, ble_read5) == 0)
                {
                    str_save(ble_read3, "空");
                    str_save(ble_read7, "5");
                    ble_nvs_write();
                    fpr("35");
                    xuliao(client);
                }
                else if (strcmp(ble_read3, ble_read6) == 0)
                {
                    str_save(ble_read3, "空");
                    str_save(ble_read7, "6");
                    ble_nvs_write();
                    fpr("36");
                    xuliao(client);
                }
                else if (strcmp(ble_read3, ble_read15) == 0)
                {
                    str_save(ble_read3, "空");
                    str_save(ble_read7, "7");
                    ble_nvs_write();
                    fpr("37");
                    xuliao(client);
                }
                else if (strcmp(ble_read3, ble_read16) == 0)
                {
                    str_save(ble_read3, "空");
                    str_save(ble_read7, "8");
                    ble_nvs_write();
                    fpr("38");
                    xuliao(client);
                }
                else
                {
                    str_save(ble_read3, "空"); // 没料了自己手动换吧
                    fpr("请手动");
                    ble_nvs_write();
                    print_error = 0;
                }
                xSemaphoreGive(ble_read_x);
            }
            else if (extruder == 4 && strcmp(ble_read4, "空") != 0)
            {
                xSemaphoreTake(ble_read_x, portMAX_DELAY);
                if (strcmp(ble_read4, ble_read1) == 0)
                {
                    str_save(ble_read4, "空");
                    str_save(ble_read7, "1");
                    ble_nvs_write();
                    fpr("41");
                    xuliao(client);
                }
                else if (strcmp(ble_read4, ble_read2) == 0)
                {
                    str_save(ble_read4, "空");
                    str_save(ble_read7, "2");
                    ble_nvs_write();
                    fpr("42");
                    xuliao(client);
                }
                else if (strcmp(ble_read4, ble_read3) == 0)
                {
                    str_save(ble_read4, "空");
                    str_save(ble_read7, "3");
                    ble_nvs_write();
                    fpr("43");
                    xuliao(client);
                }
                else if (strcmp(ble_read4, ble_read5) == 0)
                {
                    str_save(ble_read4, "空");
                    str_save(ble_read7, "5");
                    ble_nvs_write();
                    fpr("45");
                    xuliao(client);
                }
                else if (strcmp(ble_read4, ble_read6) == 0)
                {
                    str_save(ble_read4, "空");
                    str_save(ble_read7, "6");
                    ble_nvs_write();
                    fpr("46");
                    xuliao(client);
                }
                else if (strcmp(ble_read4, ble_read15) == 0)
                {
                    str_save(ble_read4, "空");
                    str_save(ble_read7, "7");
                    ble_nvs_write();
                    fpr("47");
                    xuliao(client);
                }
                else if (strcmp(ble_read4, ble_read16) == 0)
                {
                    str_save(ble_read4, "空");
                    str_save(ble_read7, "8");
                    ble_nvs_write();
                    fpr("48");
                    xuliao(client);
                }
                else
                {
                    str_save(ble_read4, "空"); // 没料了自己手动换吧
                    ble_nvs_write();
                    fpr("请手动");
                    print_error = 0;
                }
                xSemaphoreGive(ble_read_x);
            }
            else if (extruder == 5 && strcmp(ble_read5, "空") != 0)
            {
                xSemaphoreTake(ble_read_x, portMAX_DELAY);
                if (strcmp(ble_read5, ble_read1) == 0)
                {
                    str_save(ble_read5, "空");
                    str_save(ble_read7, "1");
                    ble_nvs_write();
                    fpr("51");
                    xuliao(client);
                }
                else if (strcmp(ble_read5, ble_read2) == 0)
                {
                    str_save(ble_read5, "空");
                    str_save(ble_read7, "2");
                    ble_nvs_write();
                    fpr("52");
                    xuliao(client);
                }
                else if (strcmp(ble_read5, ble_read3) == 0)
                {
                    str_save(ble_read5, "空");
                    str_save(ble_read7, "3");
                    ble_nvs_write();
                    fpr("53");
                    xuliao(client);
                }
                else if (strcmp(ble_read5, ble_read4) == 0)
                {
                    str_save(ble_read5, "空");
                    str_save(ble_read7, "4");
                    ble_nvs_write();
                    fpr("54");
                    xuliao(client);
                }
                else if (strcmp(ble_read5, ble_read6) == 0)
                {
                    str_save(ble_read5, "空");
                    str_save(ble_read7, "6");
                    ble_nvs_write();
                    fpr("56");
                    xuliao(client);
                }
                else if (strcmp(ble_read5, ble_read15) == 0)
                {
                    str_save(ble_read5, "空");
                    str_save(ble_read7, "7");
                    ble_nvs_write();
                    fpr("57");
                    xuliao(client);
                }
                else if (strcmp(ble_read5, ble_read16) == 0)
                {
                    str_save(ble_read5, "空");
                    str_save(ble_read7, "8");
                    ble_nvs_write();
                    fpr("58");
                    xuliao(client);
                }
                else
                {
                    str_save(ble_read5, "空"); // 没料了自己手动换吧
                    ble_nvs_write();
                    fpr("请手动");
                    print_error = 0;
                }
                xSemaphoreGive(ble_read_x);
            }
            else if (extruder == 6 && strcmp(ble_read6, "空") != 0)
            {
                xSemaphoreTake(ble_read_x, portMAX_DELAY);
                if (strcmp(ble_read6, ble_read1) == 0)
                {
                    str_save(ble_read6, "空");
                    str_save(ble_read7, "1");
                    ble_nvs_write();
                    fpr("61");
                    xuliao(client);
                }
                else if (strcmp(ble_read6, ble_read2) == 0)
                {
                    str_save(ble_read6, "空");
                    str_save(ble_read7, "2");
                    ble_nvs_write();
                    fpr("62");
                    xuliao(client);
                }
                else if (strcmp(ble_read6, ble_read3) == 0)
                {
                    str_save(ble_read6, "空");
                    str_save(ble_read7, "3");
                    ble_nvs_write();
                    fpr("63");
                    xuliao(client);
                }
                else if (strcmp(ble_read6, ble_read4) == 0)
                {
                    str_save(ble_read6, "空");
                    str_save(ble_read7, "4");
                    ble_nvs_write();
                    fpr("64");
                    xuliao(client);
                }
                else if (strcmp(ble_read6, ble_read5) == 0)
                {
                    str_save(ble_read6, "空");
                    str_save(ble_read7, "5");
                    ble_nvs_write();
                    fpr("65");
                    xuliao(client);
                }
                else if (strcmp(ble_read6, ble_read15) == 0)
                {
                    str_save(ble_read6, "空");
                    str_save(ble_read7, "7");
                    ble_nvs_write();
                    fpr("67");
                    xuliao(client);
                }
                else if (strcmp(ble_read6, ble_read16) == 0)
                {
                    str_save(ble_read6, "空");
                    str_save(ble_read7, "8");
                    ble_nvs_write();
                    fpr("68");
                    xuliao(client);
                }
                else
                {
                    str_save(ble_read6, "空"); // 没料了自己手动换吧
                    ble_nvs_write();
                    fpr("请手动");
                    print_error = 0;
                }
                xSemaphoreGive(ble_read_x);
            }
            else if (extruder == 7 && strcmp(ble_read15, "空") != 0)
            {
                xSemaphoreTake(ble_read_x, portMAX_DELAY);
                if (strcmp(ble_read15, ble_read1) == 0)
                {
                    str_save(ble_read15, "空");
                    str_save(ble_read7, "1");
                    ble_nvs_write();
                    fpr("71");
                    xuliao(client);
                }
                else if (strcmp(ble_read15, ble_read2) == 0)
                {
                    str_save(ble_read15, "空");
                    str_save(ble_read7, "2");
                    ble_nvs_write();
                    fpr("72");
                    xuliao(client);
                }
                else if (strcmp(ble_read15, ble_read3) == 0)
                {
                    str_save(ble_read15, "空");
                    str_save(ble_read7, "3");
                    ble_nvs_write();
                    fpr("73");
                    xuliao(client);
                }
                else if (strcmp(ble_read15, ble_read4) == 0)
                {
                    str_save(ble_read15, "空");
                    str_save(ble_read7, "4");
                    ble_nvs_write();
                    fpr("74");
                    xuliao(client);
                }
                else if (strcmp(ble_read15, ble_read5) == 0)
                {
                    str_save(ble_read15, "空");
                    str_save(ble_read7, "5");
                    ble_nvs_write();
                    fpr("75");
                    xuliao(client);
                }
                else if (strcmp(ble_read15, ble_read6) == 0)
                {
                    str_save(ble_read15, "空");
                    str_save(ble_read7, "6");
                    ble_nvs_write();
                    fpr("76");
                    xuliao(client);
                }
                else if (strcmp(ble_read15, ble_read16) == 0)
                {
                    str_save(ble_read15, "空");
                    str_save(ble_read7, "8");
                    ble_nvs_write();
                    fpr("78");
                    xuliao(client);
                }
                else
                {
                    str_save(ble_read15, "空"); // 没料了自己手动换吧
                    ble_nvs_write();
                    fpr("请手动");
                    print_error = 0;
                }
                xSemaphoreGive(ble_read_x);
            }
            else if (extruder == 8 && strcmp(ble_read16, "空") != 0)
            {
                xSemaphoreTake(ble_read_x, portMAX_DELAY);
                if (strcmp(ble_read16, ble_read1) == 0)
                {
                    str_save(ble_read16, "空");
                    str_save(ble_read7, "1");
                    ble_nvs_write();
                    fpr("81");
                    xuliao(client);
                }
                else if (strcmp(ble_read16, ble_read2) == 0)
                {
                    str_save(ble_read16, "空");
                    str_save(ble_read7, "2");
                    ble_nvs_write();
                    fpr("82");
                    xuliao(client);
                }
                else if (strcmp(ble_read16, ble_read3) == 0)
                {
                    str_save(ble_read16, "空");
                    str_save(ble_read7, "3");
                    ble_nvs_write();
                    fpr("83");
                    xuliao(client);
                }
                else if (strcmp(ble_read16, ble_read4) == 0)
                {
                    str_save(ble_read16, "空");
                    str_save(ble_read7, "4");
                    ble_nvs_write();
                    fpr("84");
                    xuliao(client);
                }
                else if (strcmp(ble_read16, ble_read5) == 0)
                {
                    str_save(ble_read16, "空");
                    str_save(ble_read7, "5");
                    ble_nvs_write();
                    fpr("85");
                    xuliao(client);
                }
                else if (strcmp(ble_read16, ble_read6) == 0)
                {
                    str_save(ble_read16, "空");
                    str_save(ble_read7, "6");
                    ble_nvs_write();
                    fpr("86");
                    xuliao(client);
                }
                else if (strcmp(ble_read16, ble_read15) == 0)
                {
                    str_save(ble_read16, "空");
                    str_save(ble_read7, "7");
                    ble_nvs_write();
                    fpr("87");
                    xuliao(client);
                }
                else
                {
                    str_save(ble_read16, "空"); // 没料了自己手动换吧
                    ble_nvs_write();
                    fpr("请手动");
                    print_error = 0;
                }
                xSemaphoreGive(ble_read_x);
            }
            else if (strcmp(ble_read1, "空") == 0 && strcmp(ble_read2, "空") == 0 &&
                     strcmp(ble_read3, "空") == 0 && strcmp(ble_read4, "空") == 0 &&
                     strcmp(ble_read5, "空") == 0 && strcmp(ble_read6, "空") == 0 &&
                     strcmp(ble_read7, "空") == 0 && strcmp(ble_read8, "空") == 0) // 8大皆空
            {
                fpr("没料");
                ble_nvs_write();
                print_error = 0;
            }
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void Task5(void *param) // 多色换料
{
    esp_mqtt_client_handle_t client = (esp_mqtt_client_handle_t)param;

    while (true)
    {
        // fpr("通道值", extruder);
        if (work == 1)
        {
            xSemaphoreTake(ble_read_x, portMAX_DELAY);
            str_save(ble_read12, "换色中");
            xSemaphoreGive(ble_read_x);
            vd = 1;
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            xSemaphoreTake(lock_x, portMAX_DELAY);
            int old_extruder = extruder;
            xSemaphoreGive(lock_x);
            // fpr("e退通道数：", extruder);
            tui(old_extruder, tm);
            publish(client, bambu::msg::runGcode(std::string("M190 S") + std::to_string(bed_target_temper_max) + std::string("\nM211 S \nM211 X1 Y1 Z1\nM1002 push_ref_mode\nG91 \nG1 Z-5.0 F900\nM1002 pop_ref_mode\nM211 R\nG1 E-17 F900\nM109 S250\n"))); // 恢复原来的热床温
            vTaskDelay(1000 / portTICK_PERIOD_MS);                                                                                                                                                                                                           // 延迟1秒
            publish(client, bambu::msg::uload);
            vTaskDelay(1000 / portTICK_PERIOD_MS); // 延迟1秒

            delay(退料完成需要退线, client);
            esp::gpio_out(config::motors[old_extruder - 1].backward, true);
            mstd::delay(std::chrono::seconds(atoi(ble_read17))); // 明确使用秒为单位
            esp::gpio_out(config::motors[old_extruder - 1].backward, false);
            delay(退料完成, client);
            xSemaphoreTake(lock_x, portMAX_DELAY);
            extruder = new_extruder1;

            old_extruder = new_extruder1;

            char temp[20];
            snprintf(temp, sizeof(temp), "%d", extruder);

            xSemaphoreGive(lock_x);

            str_save(ble_read7, temp);
            write_nvs(ble_read7, BLE_NAMSPACE, BLE_READ7_KEY);

            vTaskDelay(1000 / portTICK_PERIOD_MS); // 延迟1秒
            publish(client, bambu::msg::runGcode(std::string("M109 S250")));
            str_save(ble_read12, "进料中");
            fpr("进料中");
            esp::gpio_out(config::motors[old_extruder - 1].forward, true);

            for (size_t i = 0; i < 240; i++)
            {

                if (i % 5 == 0 && i >= 15)
                {
                    publish(client, bambu::msg::get_status);

                    esp::gpio_out(config::motors[old_extruder - 1].forward, false);

                    esp::gpio_out(config::motors[old_extruder - 1].backward, true);
                    mstd::delay(500ms);
                    esp::gpio_out(config::motors[old_extruder - 1].backward, false);
                    esp::gpio_out(config::motors[old_extruder - 1].forward, true);
                }
                if (hw_switch == 1)
                {
                    fpr("料线到达");
                    break;
                }
                fpr("还没到");
                vTaskDelay(1000 / portTICK_PERIOD_MS); // 微动进料的时间可以自己改
            }

            esp::gpio_out(config::motors[old_extruder - 1].forward, false);
            vd = 0;

            if (hw_switch == 1)
            {

                str_save(ble_read12, "料线到达");
                publish(client, bambu::msg::print_resume); // 继续打印
                vTaskDelay(3000 / portTICK_PERIOD_MS);
               
               jin (old_extruder,3000);
                str_save(ble_read12, "ams正常");
                work = 0;
                fpr("结束");
            }
            else
            {
                fpr("error");
                str_save(ble_read12, "卡料");

                work = 0;
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);

    } // while
} // work换料程序

void Task6(void *param) // debug
{
while (1)
{


   xSemaphoreTake(ble_read_x, portMAX_DELAY);
   int g=400;
   int t=0;
   int h=0;
   int m=1000;
   int b=0;
          z++;
   if (strcmp(gcode_state.c_str(), "PAUSE") == 0)
   {
       g=100;
   }
    if (strcmp(gcode_state.c_str(), "RUNNING") == 0)
   {
       g=200;
   }
      if (strcmp(gcode_state.c_str(), "PREPARE") == 0)
   {
       g=300;
   }
      if (bed_target_temper  < 20)
      {
        t=10;
      }
      if (bed_target_temper  > 20)
      {
        t=20;
      }
      if (hw_switch  ==  0)
      {
        h=1;
      }
      if (hw_switch  ==  1)
      {
        h=2;
      }
      if (z>5)
      {
        m=1000;

      }
      if (z<=5)
      {
       m=2000;

      }
      
      
      
      b=g+t+h+m;

      // 将整数 b 转换为字符串
   std::string b_str = std::to_string(b);

   str_save(ble_read19, b_str.c_str()); // 蓝牙输出debug
   //fpr(b_str.c_str());

    
            xSemaphoreGive(ble_read_x); 
if (z>10)
{
      xSemaphoreTake(ble_read_x, portMAX_DELAY);
            str_save(ble_read12,"ams超时"); // 蓝牙输出debug
            xSemaphoreGive(ble_read_x);
}

          
           
            

            vTaskDelay(1000 / portTICK_PERIOD_MS); // 延迟1秒
}


}

extern "C" void app_main(void)
{
    esp::ams_gpio();
    ble_read_x = xSemaphoreCreateMutex(); // 互斥锁
    lock_x = xSemaphoreCreateMutex();     // 互斥锁2
    bool A = true;
    ble_start();
    ble_nvs_read(); // 读取蓝牙存储的数据

    if (strcmp(ble_read11, "yes") == 0)
    { // 是否要配网通过蓝牙控制

        A = false;
        esp_wifi_restore();
    }

    wifi_smart(A);
    snprintf(mqtt_server, sizeof(mqtt_server), "mqtts://%s:8883", ble_read8);
    snprintf(topic_subscribe, sizeof(topic_subscribe), "device/%s/report", ble_read10);
    snprintf(topic_publish, sizeof(topic_publish), "device/%s/request", ble_read10); // 将信息写入

    xTaskCreate(Task1, "Task1", 2048, NULL, 1, &Task1_handle); // 微动任务
    xTaskCreate(Task2, "Task2", 2048, NULL, 1, &Task2_handle); //
        xTaskCreate(Task6, "Task6", 2048, NULL, 1, &Task6_handle); //


    // xTaskCreate(Task4,"Task4",2048,NULL,1,&Task4_handle);//退料

    bed_target_temper_max = atoi(ble_read18);
    xSemaphoreTake(lock_x, portMAX_DELAY);
    extruder = atoi(ble_read7);
    xSemaphoreGive(lock_x);

    auto client = mqtt_app_start();
    xTaskCreate(Task3, "Task3", 4096, (void *)client, 1, &Task3_handle); // 传递client
    xTaskCreate(Task4, "Task4", 4096, (void *)client, 1, &Task4_handle); // 传递client
    xTaskCreate(Task5, "Task5", 4096, (void *)client, 1, &Task5_handle); // 传递client

}
