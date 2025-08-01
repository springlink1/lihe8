#include <stdio.h>
#include <string.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_smartconfig.h>
#include <esp_netif.h>
#include <esp_err.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lwip/err.h>
#include <lwip/sys.h>
#include "smartconfig_ack.h"
#include "include/espIO.hpp"
TaskHandle_t led_task;
// 定义日志标签
#define TAG_WIFI "smartconfig"

// 定义存储WiFi配置的命名空间和键名称
#define NVS_WIFI_NAMESPACE_NAME         "DEV_WIFI"
#define NVS_SSID_KEY                    "ssid"
#define NVS_PASSWORD_KEY                "password"

// 定义用于存储SSID和密码的静态字符数组
static char s_ssid_value[33] = {0};  // SSID最多32个字符
static char s_password_value[65] = {0};  // 密码最多64个字符


/** 从NVS中读取SSID
 * @param ssid 读到的ssid
 * @param maxlen 外部存储ssid数组的最大值
 * @return 读取到的字节数
*/
static size_t read_nvs_ssid(char* ssid,int maxlen)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret_val = ESP_FAIL;
    size_t required_size = 0;
    ESP_ERROR_CHECK(nvs_open(NVS_WIFI_NAMESPACE_NAME, NVS_READWRITE, &nvs_handle));
    ret_val = nvs_get_str(nvs_handle, NVS_SSID_KEY, NULL, &required_size);
    if(ret_val == ESP_OK && required_size <= maxlen)
    {
        nvs_get_str(nvs_handle,NVS_SSID_KEY,ssid,&required_size);
    }
    else
        required_size = 0;
    nvs_close(nvs_handle);
    return required_size;
}

/** 写入SSID到NVS中
 * @param ssid 需写入的ssid
 * @return ESP_OK or ESP_FAIL
*/
static esp_err_t write_nvs_ssid(char* ssid)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    ESP_ERROR_CHECK(nvs_open(NVS_WIFI_NAMESPACE_NAME, NVS_READWRITE, &nvs_handle));
    
    ret = nvs_set_str(nvs_handle, NVS_SSID_KEY, ssid);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return ret;
}

/** 从NVS中读取PASSWORD
 * @param ssid 读到的password
 * @param maxlen 外部存储password数组的最大值
 * @return 读取到的字节数
*/
static size_t read_nvs_password(char* pwd,int maxlen)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret_val = ESP_FAIL;
    size_t required_size = 0;
    ESP_ERROR_CHECK(nvs_open(NVS_WIFI_NAMESPACE_NAME, NVS_READWRITE, &nvs_handle));
    ret_val = nvs_get_str(nvs_handle, NVS_PASSWORD_KEY, NULL, &required_size);
    if(ret_val == ESP_OK && required_size <= maxlen)
    {
        nvs_get_str(nvs_handle,NVS_SSID_KEY,pwd,&required_size);
    }
    else 
        required_size = 0;
    nvs_close(nvs_handle);
    return required_size;
}

/** 写入PASSWORD到NVS中
 * @param pwd 需写入的password
 * @return ESP_OK or ESP_FAIL
*/
static esp_err_t write_nvs_password(char* pwd)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    ESP_ERROR_CHECK(nvs_open(NVS_WIFI_NAMESPACE_NAME, NVS_READWRITE, &nvs_handle));
    ret = nvs_set_str(nvs_handle, NVS_PASSWORD_KEY, pwd);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return ret;
}




// 事件处理函数，用于处理WiFi和SmartConfig相关的事件
void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
   if(event_base == WIFI_EVENT)  // 检查事件是否属于WiFi事件    
   {
      switch(event_id)  // 根据不同的WiFi事件ID进行处理
      {
         case WIFI_EVENT_STA_START:  // WiFi STA模式启动事件
            esp_wifi_connect();  // 连接WiFi
            ESP_LOGI(TAG_WIFI, "WIFI_EVENT_STA_START");  // 记录日志信息
            break;
         case WIFI_EVENT_STA_DISCONNECTED:  // WiFi STA模式断开连接事件
            vTaskDelay(pdMS_TO_TICKS(3000));  // 延迟3秒后再次尝试连接
            esp_wifi_connect();  // 重新连接WiFi
            ESP_LOGI(TAG_WIFI, "WIFI_EVENT_STA_DISCONNECTED");  // 记录日志信息
             xSemaphoreTake(ble_read_x, portMAX_DELAY);
	        str_save(ble_read12, "WIFI fail");
            xSemaphoreGive(ble_read_x);	 
            break;
         case WIFI_EVENT_STA_CONNECTED:  // WiFi STA模式连接成功事件
            xSemaphoreTake(ble_read_x, portMAX_DELAY);
	        str_save(ble_read12, "mqtt error");
            xSemaphoreGive(ble_read_x);	

            str_save(ble_read11,"no");
			write_nvs(ble_read11,BLE_NAMSPACE,BLE_READ11_KEY); 
            ESP_LOGI(TAG_WIFI, "WIFI_EVENT_STA_CONNECTED");  // 记录日志信息

           
            break;
         default:
            break;
      }
   }
   else if (event_base == IP_EVENT)  // 检查事件是否属于IP事件
   {
      switch(event_id)  // 根据不同的IP事件ID进行处理
      {
         case IP_EVENT_STA_GOT_IP:  // 获取到IP地址事件
         {
            ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;  // 获取事件数据
            ESP_LOGI(TAG_WIFI, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));  // 记录获取到的IP地址
            break;
         }
         default:
            break;
      }
   }
   else if (event_base == SC_EVENT)  // 检查事件是否属于SmartConfig事件
   {
      switch(event_id)  // 根据不同的SmartConfig事件ID进行处理
        {
         case SC_EVENT_SCAN_DONE:  // SmartConfig扫描完成事件
            ESP_LOGI(TAG_WIFI, "SC_EVENT_SCAN_DONE");  // 记录日志信息
            break;
         case SC_EVENT_FOUND_CHANNEL:  // SmartConfig找到可用信道事件
            ESP_LOGI(TAG_WIFI, "SC_EVENT_FOUND_CHANNEL");  // 记录日志信息
            break;
         case SC_EVENT_GOT_SSID_PSWD:  // SmartConfig获取到SSID和密码事件
            xSemaphoreTake(ble_read_x, portMAX_DELAY);
	        str_save(ble_read12, "mqtt error");
            xSemaphoreGive(ble_read_x);
            str_save(ble_read11,"no");
			write_nvs(ble_read11,BLE_NAMSPACE,BLE_READ11_KEY);

            smartconfig_event_got_ssid_pswd_t* event = (smartconfig_event_got_ssid_pswd_t*) event_data;  // 获取事件数据
            wifi_config_t wifi_config={0};  // 初始化WiFi配置结构体
            memset(&wifi_config, 0, sizeof(wifi_config_t));  // 清空WiFi配置结构体
            strncpy((char*)wifi_config.sta.ssid, (char*)event->ssid, 32);  // 复制SSID到配置结构体，最多32个字符
            strncpy((char*)wifi_config.sta.password, (char*)event->password, 64);  // 复制密码到配置结构体，最多64个字符
            
            // 如果设置了BSSID，则使用指定的BSSID进行连接
            if (wifi_config.sta.bssid_set)
            {
                memcpy(wifi_config.sta.bssid, event->bssid, 6);  // 复制BSSID
                esp_wifi_disconnect();  // 断开当前连接
                esp_wifi_set_config(WIFI_IF_STA , &wifi_config);  // 设置新的WiFi配置
                esp_wifi_connect();  // 连接WiFi
            }
            
            ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA , &wifi_config));  // 设置WiFi配置，返回错误检查
  // 设置WiFi配置，返回错误检查
            ESP_LOGI(TAG_WIFI, "SSID:%s", event->ssid);  // 记录获取到的SSID
            ESP_LOGI(TAG_WIFI, "PASSWORD:%s", event->password);  // 记录获取到的密码
            ESP_LOGI(TAG_WIFI, "SC_EVENT_GOT_SSID_PSWD");  // 记录日志信息
            snprintf(s_ssid_value,33,"%s",(char*)event->ssid);
            snprintf(s_password_value,65,"%s",(char*)event->password);
            

            write_nvs_ssid(s_ssid_value);   //将ssid写入NVS
            write_nvs_password(s_password_value);   //将password写入NV   
            
           
	
             vTaskDelete(led_task);  
             esp::gpio_out(GPIO_NUM_13,false);     
            break;
        
        }
    }   
}


void ledtask(void *arg)
{
    while(1)
    {
        esp::gpio_out(GPIO_NUM_13,false);    // 点亮LED
        vTaskDelay(pdMS_TO_TICKS(1000));  // 延迟1秒
        esp::gpio_out(GPIO_NUM_13,true);   // 熄灭LED
        vTaskDelay(pdMS_TO_TICKS(1000));  // 延迟1秒


    }
    

}


void wifi_smart(bool A)
{

  ESP_ERROR_CHECK(nvs_flash_init());  // 初始化NVS闪存，返回错误检查



  ESP_ERROR_CHECK(esp_netif_init());  // 初始化网络接口，返回错误检查
  ESP_ERROR_CHECK(esp_event_loop_create_default());  // 创建默认事件循环，返回错误检查
  esp_netif_create_default_wifi_sta();  // 创建默认的WiFi STA网络接口
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();  // 获取默认的WiFi初始化配置
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));  // 初始化WiFi，返回错误检查

  // 注册WiFi事件处理函数
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL));
  // 注册IP事件处理函数
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, event_handler, NULL));
  // 注册SmartConfig事件处理函数
  ESP_ERROR_CHECK(esp_event_handler_register(SC_EVENT, ESP_EVENT_ANY_ID, event_handler, NULL));

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));  // 设置WiFi模式为STA，返回错误检查

   //从NVS中读出SSID
    read_nvs_ssid(s_ssid_value,32);

    //从NVS中读取PASSWORD
    read_nvs_password(s_password_value,64);
    ESP_LOGI(TAG_WIFI, "SSID:%s", s_ssid_value);  // 记录获取到的SSID
    ESP_LOGI(TAG_WIFI, "PASSWORD:%s", s_password_value);  // 记录获取到的密码
   
  if(A)    //通过SSID第一个字节是否是0，判断是否读取成功，然后设置wifi_config_t
    {
       wifi_config_t wifi_config = 
{
    .sta = 
    { 
        .threshold = { .authmode = WIFI_AUTH_WPA2_PSK }, // 修改这里
        .pmf_cfg = 
        {
            .capable = true,
            .required = false
        },
    },
};

       
        snprintf((char*)wifi_config.sta.ssid,32,(char*)s_ssid_value);
        snprintf((char*)wifi_config.sta.password,64,(char*)s_password_value);
        ESP_ERROR_CHECK( esp_wifi_start() );
        

    }

    

else  // 如果没有读取到SSID，则使用SmartConfig进行配网
  {
    xTaskCreate(ledtask, "led", 2048, NULL, 3, &led_task);  // 创建灯事件
    ESP_ERROR_CHECK( esp_wifi_start() );
    ESP_LOGI(TAG_WIFI, "Starting SmartConfig");
    
  esp_smartconfig_set_type(SC_TYPE_ESPTOUCH_AIRKISS);  // 设置SmartConfig类型为ESPTOUCH和AIRKISS
  smartconfig_start_config_t cfg_1 = SMARTCONFIG_START_CONFIG_DEFAULT();  // 获取默认的SmartConfig启动配置
  esp_smartconfig_start(&cfg_1);  // 启动SmartConfig
  }

}






