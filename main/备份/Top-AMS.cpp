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


TaskHandle_t Task1_handle;
TaskHandle_t Task2_handle;
TaskHandle_t Task3_handle;
TaskHandle_t Task4_handle;

std::string gcode_state;
std::string tray_color="1";
std::string tray_type;
int print_error;
static uint32_t bed_target_temper = 1;
uint32_t bed_target_temper_max = 0;
int sequence_id = -1;
std::atomic<int> ams_status = -1;
std::atomic<bool> pause_lock{ false };//暂停锁
int old_extruder=0;
std::atomic<int> extruder = 1;// 1-16,初始通道默认为1 

inline constexpr int 正常 = 0;
inline constexpr int 退料完成需要退线 = 260;
inline constexpr int 退料完成 = 0;//同正常
inline constexpr int 进料检查 = 262;
inline constexpr int 进料冲刷 = 263;//推测
inline constexpr int 进料完成 = 768;

void Task1(void* param){
	esp::gpio_set_in(GPIO_NUM_4);

    while(true) {
		int level = gpio_get_level(GPIO_NUM_4);//设定4号口为缓冲驱动


		if (level == 0){
			int now_extruder = extruder;
			fpr("微动");
			esp::gpio_out(config::motors[now_extruder - 1].forward,true);
		    vTaskDelay(1000/portTICK_PERIOD_MS);//微动进料的时间可以自己改
		    esp::gpio_out(config::motors[now_extruder - 1].forward,false);
			fpr(extruder);


		}
	
			
		vTaskDelay(20/portTICK_PERIOD_MS);//延时1000ms=1s,使系统执行其他任务删了就寄了
	}}//微动缓冲程序

void Task3(void* param){
//手机app控制进退料
    while(true) {
        if(strcmp(ble_read14,"退")==0)
        {
            str_save(ble_read14, "空");
            fpr("退料");
            int now_extruder = extruder;
			fpr("微动");
			esp::gpio_out(config::motors[now_extruder - 1].backward,true);
		    vTaskDelay(20000/portTICK_PERIOD_MS);//微动进料的时间可以自己改
		    esp::gpio_out(config::motors[now_extruder - 1].backward,false);
			fpr(extruder);


        }

          if(strcmp(ble_read14,"进")==0)
        {
            str_save(ble_read14, "空");
            fpr("进料");
            int now_extruder = extruder;
			fpr("微动");
			esp::gpio_out(config::motors[now_extruder - 1].forward,true);
		    vTaskDelay(20000/portTICK_PERIOD_MS);//微动进料的时间可以自己改
		    esp::gpio_out(config::motors[now_extruder - 1].forward,false);
			fpr(extruder);


        }
        vTaskDelay(1000/portTICK_PERIOD_MS);//延时1000ms=1s,使系统执行其他任务删了就寄了
     
}
}








void publish(esp_mqtt_client_handle_t client,const std::string& msg) {

	mstd::delay(2s);
	fpr("发送消息:",msg);
	int msg_id = esp_mqtt_client_publish(client,topic_publish,msg.c_str(),msg.size(),0,0);
	if (msg_id < 0)
		fpr("发送失败");
	else
		fpr("发送成功,消息id=",msg_id);
	// fpr(TAG, "binary sent with msg_id=%d", msg_id);
	
	mstd::delay(2s);//发布函数
}

void xuliao(esp_mqtt_client_handle_t client){//自动续料调用
       
	    str_save(ble_read12, "自动续料中");

	    fpr(extruder);
		//publish(client,bambu::msg::runGcode(std::string("M109 S250")));
		fpr("续料"); 
		write_nvs(ble_read7,BLE_NAMSPACE,BLE_READ7_KEY);
        old_extruder=atoi(ble_read7);
        extruder.store(atoi(ble_read7));
		extruder.notify_one();    //将切换的通道存入nvs 
        fpr(extruder);
        mstd::delay(1s);
		fpr("进线"); 

        esp::gpio_out(config::motors[old_extruder - 1].forward,true); 
		mstd::delay(25s);
		esp::gpio_out(config::motors[old_extruder - 1].forward,false);//提前进线
        
        esp::gpio_out(config::motors[old_extruder - 1].backward,true);
		mstd::delay(500ms);
		esp::gpio_out(config::motors[old_extruder - 1].backward,false);


        esp::gpio_out(config::motors[old_extruder - 1].forward,true); 
		mstd::delay(5s);
		esp::gpio_out(config::motors[old_extruder - 1].forward,false);//提前进线

          
        esp::gpio_out(config::motors[old_extruder - 1].backward,true);
		mstd::delay(500ms);
		esp::gpio_out(config::motors[old_extruder - 1].backward,false);


        esp::gpio_out(config::motors[old_extruder - 1].forward,true); 
		mstd::delay(5s);
		esp::gpio_out(config::motors[old_extruder - 1].forward,false);//提前进线

          
        esp::gpio_out(config::motors[old_extruder - 1].backward,true);
		mstd::delay(500ms);
		esp::gpio_out(config::motors[old_extruder - 1].backward,false);


        esp::gpio_out(config::motors[old_extruder - 1].forward,true); 
		mstd::delay(5s);
		esp::gpio_out(config::motors[old_extruder - 1].forward,false);//提前进线
        
          
        publish(client,bambu::msg::runGcode(std::string("G1 E20 F300")));//进一部分料咬住
		mstd::delay(500ms);
		esp::gpio_out(config::motors[old_extruder - 1].forward,true); 
		mstd::delay(10s);
		//这里可以检查一下线确实退出来了
		esp::gpio_out(config::motors[old_extruder - 1].forward,false);

    
		fpr("rrr");
		print_error=0;
		publish(client,bambu::msg::print_resume);//继续打印

	    str_save(ble_read12, "ams正常");
     
	    fpr(extruder);
		
}

void callback_fun(esp_mqtt_client_handle_t client,const std::string& json) {//接受到信息的回调
	//fpr(json);
	using namespace ArduinoJson;
	JsonDocument doc;
	DeserializationError error = deserializeJson(doc,json);

	//auto sequence_id_now = doc["print"]["sequence_id"].as<int>();
	//if (sequence_id_now <= sequence_id)
	//	return;
	//sequence_id = sequence_id_now;
	//如果有别的地方发送了指令,会有不同的id,要区分发送端,目前似乎没有收到旧消息的情况,可能也有?

	//nozzle_target_temper = doc["print"]["nozzle_target_temper"] | nozzle_target_temper;
	bed_target_temper = doc["print"]["bed_target_temper"] | bed_target_temper;
    gcode_state = doc["print"]["gcode_state"] | "unkonw";
	print_error = doc["print"]["print_error"] | print_error;
    tray_color= doc["print"]["tray_color"] | tray_color;
	tray_type= doc["print"]["tray_type"] | tray_type;

  

   
if (print_error == 50364437 && strcmp(ble_read13, "开") == 0)//自动续料
{
	xSemaphoreTake(ble_read_x, portMAX_DELAY);
	str_save(ble_read12, "缺料");//蓝牙输出debug
    xSemaphoreGive(ble_read_x);
    fpr(extruder);
    if (extruder == 1 && strcmp(ble_read1, "空") != 0)//穷举法比较所有耗材的状态
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
        else  
        {
            str_save(ble_read1, "空"); //没料了自己手动换吧
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
        else  
        {
            str_save(ble_read2, "空"); //没料了自己手动换吧
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
        else  
        {
            str_save(ble_read3, "空"); //没料了自己手动换吧
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
        else  
        {
            str_save(ble_read4, "空"); //没料了自己手动换吧
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
        else  
        {
            str_save(ble_read5, "空"); //没料了自己手动换吧
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
        else  
        {
            str_save(ble_read6, "空"); //没料了自己手动换吧
            ble_nvs_write();
            fpr("请手动");
            print_error = 0;
          
        }
          xSemaphoreGive(ble_read_x);
    }
    else if (strcmp(ble_read1, "空") == 0 && strcmp(ble_read2, "空") == 0 && strcmp(ble_read3, "空") == 0 && strcmp(ble_read4, "空") == 0 && strcmp(ble_read5, "空") == 0 && strcmp(ble_read6, "空") == 0)//4大皆空
    {
        fpr("没料");
        ble_nvs_write();
        print_error = 0;
    }

}





	if (bed_target_temper > 0 && bed_target_temper < 17) {//读到的温度是通道
		if (gcode_state == "PAUSE") {
			mstd::delay(4s);//确保暂停动作完成
			if (bed_target_temper_max > 0) {
				publish(client,bambu::msg::runGcode(std::string("M190 S") + std::to_string(bed_target_temper_max)));//恢复原来的热床温度
		
			}

			if (extruder.exchange(bed_target_temper) != bed_target_temper) {
				fpr("唤醒换料程序");
            
					if (print_error != 50364437)//防止没料的时候启动换料程序
				{
                       	
						sprintf(ble_read7,"%ld",bed_target_temper);							
                        extruder.store(bed_target_temper);
				        write_nvs(ble_read7,BLE_NAMSPACE,BLE_READ7_KEY);
				        pause_lock = true;
				        extruder.notify_one();//唤醒耗材切换

			}
			}
			else if (!pause_lock.load()) {//可能会收到旧消息
				fpr("同一耗材,无需换料");
				publish(client,bambu::msg::print_resume);//无须换料
		
		
			}
			if (bed_target_temper_max > 0)
				bed_target_temper = bed_target_temper_max;//必要,恢复温度后,MQTT的更新可能不及时

		}
		else {
			// publish(client,bambu::msg::get_status);//从第二次暂停开始,PAUSE就不会出现在常态消息里,不知道怎么回事
			//还是会的,只是不一定和温度改变在一条json里
		}
	}
	else
		if (bed_target_temper == 0)
			bed_target_temper_max = 0;//打印结束or冷打印版
		else
			bed_target_temper_max = std::max(bed_target_temper,bed_target_temper_max);//不同材料可能底板温度不一样,这里选择维持最高的

	//int print_error_now = doc["print"]["print_error"] | -1;
	//if (print_error_now != -1) {
	//	fpr_value(print_error_now);
	//	if (print_error.exchange(print_error_now) != print_error_now)//@_@这种有变动才唤醒的地方可以合并一下
	//		print_error.notify_one();
	//}

	int ams_status_now = doc["print"]["ams_status"] | -1;
	if (ams_status_now != -1) {
		fpr("asm_status_now:",ams_status_now);
		if (ams_status.exchange(ams_status_now) != ams_status_now)
			ams_status.notify_one();
	}

}//callback




//mqtt事件循环处理(call_back)
void mqtt_event_handler(void* handler_args,esp_event_base_t base,int32_t event_id,void* event_data) {
	auto TAG = "MQTT ";
	fpr("\n事件分发");
	esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
	esp_mqtt_client_handle_t client = event->client;
	int msg_id = -1;
	switch (esp_mqtt_event_id_t(event_id)) {
	case MQTT_EVENT_CONNECTED:
		fpr(TAG,"MQTT_EVENT_CONNECTED（MQTT连接成功）");
		xSemaphoreTake(ble_read_x, portMAX_DELAY);
	    str_save(ble_read12, "ams正常");
        
        xSemaphoreGive(ble_read_x);	 
        esp::gpio_out(esp::LED_L,true);  
		msg_id = esp_mqtt_client_subscribe(client,topic_subscribe,1);
		fpr(TAG,"发送订阅成功，msg_id=",msg_id);
		break;
	case MQTT_EVENT_DISCONNECTED:
		fpr(TAG,"MQTT_EVENT_DISCONNECTED（MQTT断开连接）");
		break;
	case MQTT_EVENT_BEFORE_CONNECT:
		fpr("连接前");
		break;
	case MQTT_EVENT_SUBSCRIBED:
		fpr(TAG,"MQTT_EVENT_SUBSCRIBED（MQTT订阅成功），msg_id=",event->msg_id);
		break;
	case MQTT_EVENT_UNSUBSCRIBED:
		fpr(TAG,"MQTT_EVENT_UNSUBSCRIBED（MQTT取消订阅成功），msg_id=",event->msg_id);
		break;
	case MQTT_EVENT_PUBLISHED:
		fpr(TAG,"MQTT_EVENT_PUBLISHED（MQTT消息发布成功），msg_id=",event->msg_id);
		break;
	case MQTT_EVENT_DATA:
		// fpr(TAG,"MQTT_EVENT_DATA（接收到MQTT消息）");
		// printf("主题=%.*s\r\n",event->topic_len,event->topic);
		printf("%.*s\r\n",event->data_len,event->data);

		callback_fun(client,std::string(event->data));
		break;
	case MQTT_EVENT_ERROR:
	   
	    
		fpr(TAG,"MQTT_EVENT_ERROR（MQTT事件错误）");
		if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
			fpr(TAG,"从esp-tls报告的最后错误代码：",event->error_handle->esp_tls_last_esp_err);
			fpr(TAG,"TLS堆栈最后错误号：",event->error_handle->esp_tls_stack_err);
			fpr(TAG,"最后捕获的errno：",event->error_handle->esp_transport_sock_errno,
				strerror(event->error_handle->esp_transport_sock_errno));
		}
		else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
			fpr(TAG,"连接被拒绝错误：",event->error_handle->connect_return_code);
		}
		else {
			fpr(TAG,"未知的错误类型：",event->error_handle->error_type);
		}
		break;
	default:
		fpr(TAG,"其他事件id:",event->event_id);
		break;
	}
}//mqtt_event_handler

//{"print":{"nozzle_temper":214.5,"bed_temper":36.03125,"mc_print_stage":"3","print_error":50364437,"wifi_signal":"-36dBm","gcode_state":"PAUSE","home_flag":847201687,"mc_print_sub_stage":0,"command":"push_status","msg":1,"sequence_id":"767"}}
//{"print":{"nozzle_temper":227.78125,"bed_temper":32.75,"mc_print_stage":"3",断料"hw_switch_state":0,"print_error":50364437,"wifi_signal":"-40dBm","gcode_state":"PAUSE","stg_cur":0,"home_flag":847201687,"command":"push_status","msg":1,"sequence_id":"843"}}

[[nodiscard]] esp_mqtt_client_handle_t mqtt_app_start() {

	esp_mqtt_client_config_t mqtt_cfg{};
	mqtt_cfg.broker.address.uri = mqtt_server;
	mqtt_cfg.broker.verification.skip_cert_common_name_check = true;
	mqtt_cfg.credentials.username = mqtt_username;
	mqtt_cfg.credentials.authentication.password = ble_read9;

	fpr("[APP] Free memory: ",esp_get_free_heap_size(),"bytes");
	esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
	esp_mqtt_client_register_event(client,MQTT_EVENT_ANY,mqtt_event_handler,nullptr);
	esp_mqtt_client_start(client);
	return client;
}







void work(esp_mqtt_client_handle_t client) {//需要更好名字
	    old_extruder = extruder;
		while (true) {
		esp::gpio_out(esp::LED_L,true);
		ESP_LOGI("TAG","开始工作%d",F); 
		extruder.wait(old_extruder);
        ESP_LOGI("TAG","存入%d",F); 
		if (print_error == 50364437 ||F==1)//防止各种意外启动换料 任何改变通道的值都需要在这边增加判断
		{
			F=0;
			fpr("续料中");
			old_extruder = extruder;	
			continue;	
			//检测到不是在换料的时间 不换料继续暂停
				
		}	

		xSemaphoreTake(ble_read_x, portMAX_DELAY);
	    str_save(ble_read12, "换色中");
        xSemaphoreGive(ble_read_x);
		esp::gpio_out(esp::LED_L,false);	
		publish(client,bambu::msg::runGcode(std::string("M109 S250")));
		mstd::delay(1s);
        publish(client,bambu::msg::runGcode(std::string("M211 S \nM211 X1 Y1 Z1\nG91 \nG1 Z-5.0 F900\nM211 R\n")));
		vTaskSuspend(Task1_handle);//关闭前进微动防止意外
		 
		esp::gpio_out(config::motors[old_extruder - 1].backward,true);
		mstd::delay(3s);
		esp::gpio_out(config::motors[old_extruder - 1].backward,false);
		//减少压力
        
		publish(client,bambu::msg::runGcode(std::string("G1 E-20 F900")));//长回抽功能 减少拉屎

		       
		mstd::delay(8s);
		esp::gpio_out(config::motors[old_extruder - 1].backward,true);
		mstd::delay(3s);
		esp::gpio_out(config::motors[old_extruder - 1].backward,false);
		//减少压力
		publish(client,bambu::msg::uload);
		fpr("发送了退料命令,等待退料完成");
        mstd::delay(3s);
	
		mstd::atomic_wait_un(ams_status,退料完成需要退线);
		fpr("退料完成,需要退线,等待退线完");
                  
		esp::gpio_out(config::motors[old_extruder - 1].backward,true);
		mstd::delay(18s);
		esp::gpio_out(config::motors[old_extruder - 1].backward,false);
		mstd::atomic_wait_un(ams_status,退料完成);//应该需要这个wait,打印机或者网络偶尔会卡
        vTaskResume (Task1_handle);//恢复微动
		old_extruder = extruder;//切换新的通道
       fpr("电机进",old_extruder);
		esp::gpio_out(config::motors[old_extruder - 1].forward,true); 
		mstd::delay(10s);
		esp::gpio_out(config::motors[old_extruder - 1].forward,false);//提前进线
        
        esp::gpio_out(config::motors[old_extruder - 1].backward,true);
		mstd::delay(500ms);
		esp::gpio_out(config::motors[old_extruder - 1].backward,false);


        esp::gpio_out(config::motors[old_extruder - 1].forward,true); 
		mstd::delay(5s);
		esp::gpio_out(config::motors[old_extruder - 1].forward,false);//提前进线

          
        esp::gpio_out(config::motors[old_extruder - 1].backward,true);
		mstd::delay(500ms);
		esp::gpio_out(config::motors[old_extruder - 1].backward,false);


        esp::gpio_out(config::motors[old_extruder - 1].forward,true); 
		mstd::delay(5s);
		esp::gpio_out(config::motors[old_extruder - 1].forward,false);//提前进线

          
        esp::gpio_out(config::motors[old_extruder - 1].backward,true);
		mstd::delay(500ms);
		esp::gpio_out(config::motors[old_extruder - 1].backward,false);


        esp::gpio_out(config::motors[old_extruder - 1].forward,true); 
		mstd::delay(5s);
		esp::gpio_out(config::motors[old_extruder - 1].forward,false);//提前进线
        
          
        publish(client,bambu::msg::runGcode(std::string("G1 E20 F300")));//进一部分料咬住
		mstd::delay(500ms);
		esp::gpio_out(config::motors[old_extruder - 1].forward,true); 
		mstd::delay(10s);
		//这里可以检查一下线确实退出来了
		esp::gpio_out(config::motors[old_extruder - 1].forward,false);
        
		publish(client,bambu::msg::print_resume);//暂停恢复
        pause_lock = false;//解锁

		xSemaphoreTake(ble_read_x, portMAX_DELAY);
	    str_save(ble_read12, "ams正常");
        xSemaphoreGive(ble_read_x);
	
	
	}//while
}//work换料程序

//似乎外挂托盘的数据也能通过mqtt改动


void Task2(void* arg) {//吧蓝牙读到的通道信息同步
    int previous_ble_read7 = atoi(ble_read7);
	while (true) {
    xSemaphoreTake(ble_read_x, portMAX_DELAY);  

    // 获取当前ble_read7的值 
    int current_ble_read7 = atoi(ble_read7);
  

    // 比较当前值和前一个值
    if (current_ble_read7 != previous_ble_read7) {
        extruder.store(current_ble_read7);
        extruder.notify_one();
        previous_ble_read7 = current_ble_read7; // 更新前一个值为当前值
    }

    xSemaphoreGive(ble_read_x);
    vTaskDelay(1000 / portTICK_PERIOD_MS); // 延迟1秒    
}
	

}  



extern "C" void app_main(void) {
    esp::ams_gpio();
	ble_read_x = xSemaphoreCreateMutex();//互斥锁
    bool A=true;   
    ble_start();
	ble_nvs_read();//读取蓝牙存储的数据

    if(strcmp(ble_read11,"yes")==0){//是否要配网通过蓝牙控制

            A=false;
			esp_wifi_restore();
			str_save(ble_read11,"no");
			write_nvs(ble_read11,BLE_NAMSPACE,BLE_READ11_KEY);
	}

    wifi_smart(A);
    snprintf(mqtt_server, sizeof(mqtt_server), "mqtts://%s:8883", ble_read8);
    snprintf(topic_subscribe, sizeof(topic_subscribe),"device/%s/report",ble_read10);
	snprintf(topic_publish, sizeof(topic_publish),"device/%s/request",ble_read10);//将信息写入
	
	
 


	xTaskCreate(Task1,"Task1",2048,NULL,1,&Task1_handle);//微动任务
	xTaskCreate(Task2,"Task2",2048,NULL,1,&Task2_handle);//
    xTaskCreate(Task3,"Task3",2048,NULL,1,&Task3_handle);//进料
    //xTaskCreate(Task4,"Task4",2048,NULL,1,&Task4_handle);//退料
    	
    
    
    extruder.store(atoi(ble_read7));
	extruder.notify_one();//读取通道数
 
	
       auto client = mqtt_app_start();
           work(client);//开机

}


