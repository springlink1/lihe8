
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"
#include "c_tool.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

SemaphoreHandle_t ble_read_x;//互斥锁
SemaphoreHandle_t lock_x;//互斥锁
int F=0;//检测是否是蓝牙改变了通道






inline const char mqtt_username[20] = "bblp";

//存储蓝牙数据到nvs里保存
static char ble_read1[20]="空";//读取传出的值
static char ble_read2[20]="空";//读取传出的值
static char ble_read3[20]="空";//读取传出的值
static char ble_read4[20]="空";//读取传出的值
static char ble_read5[20]="空";//读取传出的值
static char ble_read6[20]="空";//读取传出的值
static char ble_read7[20]="1";//读取传出的值
static char ble_read8[20]="10";//读取传出的值
static char ble_read9[20]="10";//读取传出的值
static char ble_read10[20]="10";//读取传出的值
static char ble_read11[20]="yes";//写进来的值
static char ble_read12[20]="wifi未连接";//写进来的值
static char ble_read13[20]="关";//写进来的值
static char ble_read14[20]="无";//读取传出的值
static char ble_read15[20]="空";//读取传出的值
static char ble_read16[20]="空";//读取传出的值
static char ble_read17[20]="10";//退料时间
static char ble_read18[20]="0";//读取传出的值
static char ble_read19[20]="0";//读取传出的值



static char mqtt_server[50]="0";//读取传出的值
static char topic_subscribe[50]="0";//读取传出的值
static char topic_publish[50]="0";//读取传出的值

static char ble_w1[20]="00";//写进来的值
static char ble_w2[20]="00";//写进来的值
static char ble_w3[20]="00";//写进来的值
static char ble_w4[20]="00";//写进来的值
static char ble_w5[20]="00";//写进来的值
static char ble_w6[20]="00";//写进来的值
static char ble_w7[20]="00";//写进来的值
static char ble_w8[20]="00";//写进来的值
static char ble_w9[20]="00";//写进来的值
static char ble_w10[20]="00";//写进来的值
static char ble_w11[20]="yes";//写进来的值
static char ble_w12[20]="0";//写进来的值0正常，1缺料，2续料，3换色中
static char ble_w13[20]="0";//写进来的值0正常，1缺料，2续料，3换色中
static char ble_w14[20]="0";//写进来的值0正常，1缺料，2续料，3换色中
static char ble_w15[20]="0";//写进来的值0正常，1缺料，2续料，3换色中
static char ble_w16[20]="0";//写进来的值0正常，1缺料，2续料，3换色中
static char ble_w17[20]="10";//写进来的值
static char ble_w18[20]="10";//写进来的值
static char ble_w19[20]="0";//写进来的值

uint32_t bed_target_temper_max = 0;





//nvs的key
#define BLE_NAMSPACE "ble_set"
#define BLE_READ1_KEY "read1"
#define BLE_READ2_KEY "read2"
#define BLE_READ3_KEY "read3"
#define BLE_READ4_KEY "read4"
#define BLE_READ5_KEY "read5"
#define BLE_READ6_KEY "read6"
#define BLE_READ7_KEY "read7"
#define BLE_READ8_KEY "read8"
#define BLE_READ9_KEY "read9"
#define BLE_READ10_KEY "read10"
#define BLE_READ11_KEY "read11"
#define BLE_READ12_KEY "read12"
#define BLE_READ13_KEY "read13"
#define BLE_READ14_KEY "read14"
#define BLE_READ15_KEY "read15"
#define BLE_READ16_KEY "read16"
#define BLE_READ17_KEY "read17"
#define BLE_READ18_KEY "read18"






static int gl_con_id= 0xFFFF;;//保存蓝牙id
static u_int8_t ble_v1=0;//信息
//gatt的访问接口，一个Profile（APP）对应1个
static uint16_t gl_gatts_if = ESP_GATT_IF_NONE;



/* Attributes State Machine */
enum          //每个服务都要
{
    IDX_SVC,
    IDX_CHAR_A,//服务本身

    IDX_CHAR_VAL_A,
    IDX_CHAR_CFG_A,//1
    IDX_CHAR_B,
    IDX_CHAR_VAL_B,
    IDX_CHAR_CFG_B,//2

    IDX_CHAR_C,
    IDX_CHAR_VAL_C,
    IDX_CHAR_CFG_C,//3

    IDX_CHAR_D,
    IDX_CHAR_VAL_D,
    IDX_CHAR_CFG_D,//4

    IDX_CHAR_E,
    IDX_CHAR_VAL_E,
    IDX_CHAR_CFG_E,//5

    IDX_CHAR_F,
    IDX_CHAR_VAL_F,
    IDX_CHAR_CFG_F,//6

    IDX_CHAR_G,
    IDX_CHAR_VAL_G,
    IDX_CHAR_CFG_G,//当前通道数

    IDX_CHAR_H,
    IDX_CHAR_VAL_H,
    IDX_CHAR_CFG_H,//ip

    IDX_CHAR_I,
    IDX_CHAR_VAL_I,
    IDX_CHAR_CFG_I,//配对码

    IDX_CHAR_J,
    IDX_CHAR_VAL_J,
    IDX_CHAR_CFG_J,//设备序列号

    IDX_CHAR_K,
    IDX_CHAR_VAL_K,
    IDX_CHAR_CFG_K,//是否配网


    IDX_CHAR_L,
    IDX_CHAR_VAL_L,
    IDX_CHAR_CFG_L,//当前状态

    
    IDX_CHAR_M,
    IDX_CHAR_VAL_M,
    IDX_CHAR_CFG_M,//自动换料开关

    IDX_CHAR_N,
    IDX_CHAR_VAL_N,
    IDX_CHAR_CFG_N,//进退料
    
     IDX_CHAR_O,
    IDX_CHAR_VAL_O,
    IDX_CHAR_CFG_O,//7

    IDX_CHAR_P,
    IDX_CHAR_VAL_P,
    IDX_CHAR_CFG_P,//8

     IDX_CHAR_Q,
    IDX_CHAR_VAL_Q,
    IDX_CHAR_CFG_Q,//退料

    IDX_CHAR_R,
    IDX_CHAR_VAL_R,
    IDX_CHAR_CFG_R,//日志
 







    HRS_IDX_NB,//这个值代表枚举的 最大值=======================================
};




















#define GATTS_TABLE_TAG "GATTS_TABLE_DEMO"

#define PROFILE_NUM                 1
#define PROFILE_APP_IDX             0
#define ESP_APP_ID                  0x55
#define SAMPLE_DEVICE_NAME          "ams"//服务名称没什么用=================================================
#define SVC_INST_ID                 0

/* The max length of characteristic value. When the GATT client performs a write or prepare write operation,
*  the data length must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
*/

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 500
#define PREPARE_BUF_MAX_SIZE        1024
#define CHAR_DECLARATION_SIZE       (sizeof(uint8_t))

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t adv_config_done       = 0;

uint16_t heart_rate_handle_table[HRS_IDX_NB];

typedef struct {
    uint8_t                 *prepare_buf;
    int                     prepare_len;
} prepare_type_env_t;

static prepare_type_env_t prepare_write_env;

#define CONFIG_SET_RAW_ADV_DATA
#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    /* Flags */
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
    /* TX Power Level */
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xEB,
    /* Complete 16-bit Service UUIDs */
    0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xFF, 0x00,
    /* Complete Local Name */
    0x08, ESP_BLE_AD_TYPE_NAME_CMPL,
    'T', 'O', 'P', '_', 'A', 'M', 'C'//16进制更改需要计算（0X08） 蓝 牙名称重要===============================================
};



static uint8_t raw_scan_rsp_data[] = {
    /* Flags */
    0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
    /* TX Power Level */
    0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xEB,
    /* Complete 16-bit Service UUIDs */
    0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xFF, 0x00
};

#else
static uint8_t service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

/* The length of adv data must be less than 31 bytes */
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp        = false,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval        = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance          = 0x00,
    .manufacturer_len    = 0,    //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //test_manufacturer,
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp        = true,
    .include_name        = true,
    .include_txpower     = true,
    .min_interval        = 0x0006,
    .max_interval        = 0x0010,
    .appearance          = 0x00,
    .manufacturer_len    = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len    = 0,
    .p_service_data      = NULL,
    .service_uuid_len    = sizeof(service_uuid),
    .p_service_uuid      = service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min         = 0x20,
    .adv_int_max         = 0x40,
    .adv_type            = ADV_TYPE_IND,
    .own_addr_type       = BLE_ADDR_TYPE_PUBLIC,
    .channel_map         = ADV_CHNL_ALL,
    .adv_filter_policy   = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};



static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
					esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst heart_rate_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

/* Service 变动要改4个地方================================================================================================================*/
static const uint16_t GATTS_SERVICE_UUID_TEST      = 0x00FF;//uuid部分
static const uint16_t GATTS_CHAR_UUID_TEST_A       = 0xFF01;
static const uint16_t GATTS_CHAR_UUID_TEST_B       = 0xFF02;
static const uint16_t GATTS_CHAR_UUID_TEST_C       = 0xFF03;
static const uint16_t GATTS_CHAR_UUID_TEST_D       = 0xFF04;//添加一个uuid 这里改了后面也要改====================================================
static const uint16_t GATTS_CHAR_UUID_TEST_E       = 0xFF05;//只写
static const uint16_t GATTS_CHAR_UUID_TEST_F       = 0xFF06;//带通知
static const uint16_t GATTS_CHAR_UUID_TEST_G       = 0xFF07;//带描述符
static const uint16_t GATTS_CHAR_UUID_TEST_H       = 0xFF08;//只读
static const uint16_t GATTS_CHAR_UUID_TEST_I       = 0xFF09;//只写
static const uint16_t GATTS_CHAR_UUID_TEST_J       = 0xFF0A;//只读
static const uint16_t GATTS_CHAR_UUID_TEST_K       = 0xFF0B;//只写
static const uint16_t GATTS_CHAR_UUID_TEST_L       = 0xFF0C;//只读
static const uint16_t GATTS_CHAR_UUID_TEST_M       = 0xFF0D;//只读
static const uint16_t GATTS_CHAR_UUID_TEST_N       = 0xFF0E;//只写
static const uint16_t GATTS_CHAR_UUID_TEST_O       = 0xFF0F;//只读
static const uint16_t GATTS_CHAR_UUID_TEST_P       = 0xFF10;//只写
static const uint16_t GATTS_CHAR_UUID_TEST_Q       = 0xFF11;//只写
static const uint16_t GATTS_CHAR_UUID_TEST_R       = 0xFF12;//只写




static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t char_prop_read                =  ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t char_prop_write               = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t char_prop_read_write_notify   = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;//注意这边声明的常量==================================================================
static const uint8_t heart_measurement_ccc[2]      = {0x00, 0x00};
static  uint8_t char_value[2]                 = {0x00, 0x00};
static const char test_value[]                 = "test_string";//更改值为字符串================================================================================================
static  uint8_t char_value1                = 0;












static const esp_gatts_attr_db_t gatt_db[HRS_IDX_NB] =
{
    // Service Declaration
    [IDX_SVC]        =       //服务声明                                         //访问权限的读写
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_TEST), (uint8_t *)&GATTS_SERVICE_UUID_TEST}},

    /* Characteristic Declaration */   //特征声明
    [IDX_CHAR_A]     =   //在隔壁文件里
/*   控制模式自动              uuid长度，                      uuid对应的指针          属性表权限表                                */
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
/*     最大长度                   写入值的长度                  写入的值                                                                                                         */
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    /* Characteristic Value */            //特征值
    [IDX_CHAR_VAL_A] =//ESP_GATT_RSP_BY_APP定义一个自动发送事件的==============================================================================================================================
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_A, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},//char_value读取获得的值=不是很重要======================================================================================

    /* Client Characteristic Configuration Descriptor */     //描述特征的
    [IDX_CHAR_CFG_A]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},//描述特征的可以不要



    [IDX_CHAR_B]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_B] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_B, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_B]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},



     [IDX_CHAR_C]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_C] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_C, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_C]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},




       [IDX_CHAR_D]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_D] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_D, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_D]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},




       [IDX_CHAR_E]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_E] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_E, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_E]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},





       [IDX_CHAR_F]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_F] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_F, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_F]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},




       [IDX_CHAR_G]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_G] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_G, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_G]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},


       [IDX_CHAR_H]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_H] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_H, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_H]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

    [IDX_CHAR_I]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_I] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_I, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_I]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

     [IDX_CHAR_J]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_J] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_J, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_J]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},


       [IDX_CHAR_K]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_K] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_K, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_K]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

[IDX_CHAR_L]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_L] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_L, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_L]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},







       [IDX_CHAR_M]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_M] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_M, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_M]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

       [IDX_CHAR_N]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_N] =  
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_N, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

       //描述特征的
    [IDX_CHAR_CFG_N]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

       [IDX_CHAR_O]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_O] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_O, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_O]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

       [IDX_CHAR_P]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_P] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_P, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_P]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

       [IDX_CHAR_Q]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_Q] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_Q, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_Q]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},

       [IDX_CHAR_R]     =  
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_write_notify}},//读写属性

    //特征值
    [IDX_CHAR_VAL_R] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEST_R, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      GATTS_DEMO_CHAR_VAL_LEN_MAX, sizeof(char_value), (uint8_t *)char_value}},

   //描述特征的
    [IDX_CHAR_CFG_R]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(heart_measurement_ccc), (uint8_t *)heart_measurement_ccc}},























};







static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
    #ifdef CONFIG_SET_RAW_ADV_DATA
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #else
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~ADV_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            adv_config_done &= (~SCAN_RSP_CONFIG_FLAG);
            if (adv_config_done == 0){
                esp_ble_gap_start_advertising(&adv_params);
            }
            break;
    #endif
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            /* advertising start complete event to indicate advertising start successfully or failed */
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "advertising start failed");
            }else{
                ESP_LOGI(GATTS_TABLE_TAG, "advertising start successfully");
            }
            break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TABLE_TAG, "Advertising stop failed");
            }
            else {
                ESP_LOGI(GATTS_TABLE_TAG, "Stop adv successfully");
            }
            break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "update connection params status = %d, conn_int = %d, latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
            break;
        default:
            break;
    }
}

void example_prepare_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    ESP_LOGI(GATTS_TABLE_TAG, "prepare write, handle = %d, value len = %d", param->write.handle, param->write.len);
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.offset > PREPARE_BUF_MAX_SIZE) {
        status = ESP_GATT_INVALID_OFFSET;
    } else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE) {
        status = ESP_GATT_INVALID_ATTR_LEN;
    }
    if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL) {
        prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
        prepare_write_env->prepare_len = 0;
        if (prepare_write_env->prepare_buf == NULL) {
            ESP_LOGE(GATTS_TABLE_TAG, "%s, Gatt_server prep no mem", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }

    /*send response when param->write.need_rsp is true */
    if (param->write.need_rsp){
        esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
        if (gatt_rsp != NULL){
            gatt_rsp->attr_value.len = param->write.len;
            gatt_rsp->attr_value.handle = param->write.handle;
            gatt_rsp->attr_value.offset = param->write.offset;
            gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
            memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
            esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
            if (response_err != ESP_OK) {
               ESP_LOGE(GATTS_TABLE_TAG, "Send response error");
            }
            free(gatt_rsp);
        }else{
            ESP_LOGE(GATTS_TABLE_TAG, "%s, malloc failed", __func__);
            status = ESP_GATT_NO_RESOURCES;
        }
    }
    if (status != ESP_GATT_OK){
        return;
    }
    memcpy(prepare_write_env->prepare_buf + param->write.offset,
           param->write.value,
           param->write.len);
    prepare_write_env->prepare_len += param->write.len;

}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param){
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC && prepare_write_env->prepare_buf){
        esp_log_buffer_hex(GATTS_TABLE_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }else{
        ESP_LOGI(GATTS_TABLE_TAG,"ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf) {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}
                                                                      //多个profilegatt——id才有用
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:{
            esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(SAMPLE_DEVICE_NAME);//设置广播名称
            if (set_dev_name_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "set device name failed, error code = %x", set_dev_name_ret);
            }
    #ifdef CONFIG_SET_RAW_ADV_DATA
            esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));//设置广播数据
            if (raw_adv_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
            }
            adv_config_done |= ADV_CONFIG_FLAG;
            //设置扫描回复参数
            esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
            if (raw_scan_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
            }
            adv_config_done |= SCAN_RSP_CONFIG_FLAG;
   
    #endif
            esp_err_t create_attr_ret = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, HRS_IDX_NB, SVC_INST_ID);//注册服务
            if (create_attr_ret){
                ESP_LOGE(GATTS_TABLE_TAG, "create attr table failed, error code = %x", create_attr_ret);
            }

          
            if(param->reg.status == ESP_GATT_OK)
            {
                gl_gatts_if = gatts_if;
                //gl_conn_id = param->connect.conn_id;
            }

        }
         
       	    break;
      case ESP_GATTS_READ_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "read");
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            if(heart_rate_handle_table[IDX_CHAR_VAL_A] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read1);
              memcpy(rsp.attr_value.value, ble_read1, sizeof(ble_read1));
         
               
            }
             if(heart_rate_handle_table[IDX_CHAR_VAL_B] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read2);
              memcpy(rsp.attr_value.value, ble_read2, sizeof(ble_read2));
               
            }

            if(heart_rate_handle_table[IDX_CHAR_VAL_C] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read3);
              memcpy(rsp.attr_value.value, ble_read3, sizeof(ble_read3));

            }

             if(heart_rate_handle_table[IDX_CHAR_VAL_D] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read4);
              memcpy(rsp.attr_value.value, ble_read4, sizeof(ble_read4));

            }

            if(heart_rate_handle_table[IDX_CHAR_VAL_E] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read5);
              memcpy(rsp.attr_value.value, ble_read5, sizeof(ble_read5));

            }

             if(heart_rate_handle_table[IDX_CHAR_VAL_F] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read6);
              memcpy(rsp.attr_value.value, ble_read6, sizeof(ble_read6));

            }

             if(heart_rate_handle_table[IDX_CHAR_VAL_G] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read7);
              memcpy(rsp.attr_value.value, ble_read7, sizeof(ble_read7));

            }

             if(heart_rate_handle_table[IDX_CHAR_VAL_H] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read8);
              memcpy(rsp.attr_value.value, ble_read8, sizeof(ble_read8));
           

            }

             if(heart_rate_handle_table[IDX_CHAR_VAL_I] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read9);
              memcpy(rsp.attr_value.value, ble_read9, sizeof(ble_read9));

            }

             if(heart_rate_handle_table[IDX_CHAR_VAL_J] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read10);
              memcpy(rsp.attr_value.value, ble_read10, sizeof(ble_read10));

            }

             if(heart_rate_handle_table[IDX_CHAR_VAL_K] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read11);
              memcpy(rsp.attr_value.value, ble_read11, sizeof(ble_read11));

            }

             if(heart_rate_handle_table[IDX_CHAR_VAL_L] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read12);
              memcpy(rsp.attr_value.value, ble_read12, sizeof(ble_read12));

            }

             if(heart_rate_handle_table[IDX_CHAR_VAL_M] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read13);
              memcpy(rsp.attr_value.value, ble_read13, sizeof(ble_read13));

            }

            if(heart_rate_handle_table[IDX_CHAR_VAL_N] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read14);
              memcpy(rsp.attr_value.value, ble_read14, sizeof(ble_read14));

            }

            if(heart_rate_handle_table[IDX_CHAR_VAL_O] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read15);
              memcpy(rsp.attr_value.value, ble_read15, sizeof(ble_read15));

            }
            if(heart_rate_handle_table[IDX_CHAR_VAL_P] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read16);
              memcpy(rsp.attr_value.value, ble_read16, sizeof(ble_read16));

            }

            if(heart_rate_handle_table[IDX_CHAR_VAL_Q] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read17);
              memcpy(rsp.attr_value.value, ble_read17, sizeof(ble_read17));

            }

              if(heart_rate_handle_table[IDX_CHAR_VAL_R] == param->read.handle) {//响应的服务
              rsp.attr_value.len = sizeof(ble_read19);
              memcpy(rsp.attr_value.value, ble_read19, sizeof(ble_read19));

            }



  

     




                 esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                        ESP_GATT_OK, &rsp);
            break;  

        case ESP_GATTS_WRITE_EVT://写=====================================================_______——————————————————————————————————————————————
            if (!param->write.is_prep){
                // the data length of gattc write  must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
                ESP_LOGI(GATTS_TABLE_TAG, "GATT_WRITE_EVT, handle = %d, value len = %d, value :%s", param->write.handle, param->write.len, param->write.value);
                esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);//收到的值====================================================================
                
                if (param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_A] ){//1
                    str_save(ble_w1, param->write.value);
                     xSemaphoreTake(ble_read_x, portMAX_DELAY);

                    str_save(ble_read1, param->write.value);
                    write_nvs(ble_read1,BLE_NAMSPACE,BLE_READ1_KEY);
                    xSemaphoreGive(ble_read_x);
                    ESP_LOGI(GATTS_TABLE_TAG, "read1 = %s, value len = %d, w1 :%s", ble_read1, param->write.len, ble_w1);
                }


                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_B] ){//2
                     
                    str_save(ble_w2, param->write.value);

                    xSemaphoreTake(ble_read_x, portMAX_DELAY);
                    str_save(ble_read2, param->write.value);
                    write_nvs(ble_read2,BLE_NAMSPACE,BLE_READ2_KEY);
                    xSemaphoreGive(ble_read_x);

                    ESP_LOGI(GATTS_TABLE_TAG, "read2 = %s, value len = %d, w2 :%s", ble_read2, param->write.len, ble_w2);
                }


                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_C] ){//3
                     
                    str_save(ble_w3, param->write.value);

                    xSemaphoreTake(ble_read_x, portMAX_DELAY);
                    str_save(ble_read3, param->write.value);
                    write_nvs(ble_read3,BLE_NAMSPACE,BLE_READ3_KEY);
                    xSemaphoreGive(ble_read_x);
                    ESP_LOGI(GATTS_TABLE_TAG, "read3 = %s, value len = %d, w3 :%s", ble_read3, param->write.len, ble_w3);
                }


                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_D] ){//4
                     
                    str_save(ble_w4, param->write.value);

                    xSemaphoreTake(ble_read_x, portMAX_DELAY);
                    str_save(ble_read4, param->write.value);
                    write_nvs(ble_read4,BLE_NAMSPACE,BLE_READ4_KEY);
                     xSemaphoreGive(ble_read_x);

                    ESP_LOGI(GATTS_TABLE_TAG, "read4 = %s, value len = %d, w4 :%s", ble_read4, param->write.len, ble_w4);
                }


                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_E] ){//5
                     
                    str_save(ble_w5, param->write.value);
                    xSemaphoreTake(ble_read_x, portMAX_DELAY);
                    str_save(ble_read5, param->write.value);
                    write_nvs(ble_read5,BLE_NAMSPACE,BLE_READ5_KEY);
                    xSemaphoreGive(ble_read_x);

                    ESP_LOGI(GATTS_TABLE_TAG, "read5 = %s, value len = %d, w5 :%s", ble_read5, param->write.len, ble_w5);
                }


                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_F] ){//6
                     
                    str_save(ble_w6, param->write.value);

                    xSemaphoreTake(ble_read_x, portMAX_DELAY);
                    str_save(ble_read6, param->write.value);
                    write_nvs(ble_read6,BLE_NAMSPACE,BLE_READ6_KEY);
                    xSemaphoreGive(ble_read_x);

                    ESP_LOGI(GATTS_TABLE_TAG, "read6 = %s, value len = %d, w6 :%s", ble_read6, param->write.len, ble_w6);
                }


                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_G] ){//当前通道数
                     int A = 0;
                    str_save(ble_w7, param->write.value);
                   
                    A=strlen(ble_w7);
                 
                    if (A == 1)
                    {   
                       xSemaphoreTake(ble_read_x, portMAX_DELAY);

                        str_save(ble_read7, param->write.value);
                        vTaskDelay(800/portTICK_PERIOD_MS);
                        write_nvs(ble_read7,BLE_NAMSPACE,BLE_READ7_KEY);
                        F=1;
                      xSemaphoreGive(ble_read_x);
                    }
                    
                    
                    
                   
                 
                    ESP_LOGI(GATTS_TABLE_TAG, "read7 = %s, value len = %d, w7 :%s", ble_read7, param->write.len, ble_w7);
                }


                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_H] ){//ip
                    
                     
                    str_save(ble_w8, param->write.value);
            
                    str_save(ble_read8, param->write.value);
                   
                    
                    
                    write_nvs(ble_read8,BLE_NAMSPACE,BLE_READ8_KEY);
                    ESP_LOGI(GATTS_TABLE_TAG, "read8 = %s, value len = %d, w8 :%s", ble_read8, param->write.len, ble_w8);
                }

                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_I] ){//配对码
                     
                    str_save(ble_w9, param->write.value);
                    str_save(ble_read9, param->write.value);
                    write_nvs(ble_read9,BLE_NAMSPACE,BLE_READ9_KEY);

                    ESP_LOGI(GATTS_TABLE_TAG, "read9 = %s, value len = %d, w9 :%s", ble_read9, param->write.len, ble_w9);
                }


                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_J] ){//ser
                    
                    str_save(ble_w10, param->write.value);
                    str_save(ble_read10, param->write.value);
                    write_nvs(ble_read10,BLE_NAMSPACE,BLE_READ10_KEY);

                 

                    ESP_LOGI(GATTS_TABLE_TAG, "read10 = %s, value len = %d, w10 :%s", ble_read10, param->write.len, ble_w10);
                    vTaskDelay(3000/portTICK_PERIOD_MS);
                    esp_restart();
                }

                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_K] ){//是否重新配对wifi yes or no
                     
                    str_save(ble_w11, param->write.value);
                    str_save(ble_read11, param->write.value);
                    write_nvs(ble_read11,BLE_NAMSPACE,BLE_READ11_KEY);
              

                    ESP_LOGI(GATTS_TABLE_TAG, "read11 = %s, value len = %d, w11 :%s", ble_read11, param->write.len, ble_w11);
                    vTaskDelay(3000/portTICK_PERIOD_MS);
                    esp_restart();
                }

                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_L] ){//debug
                     
                    str_save(ble_w12, param->write.value);
                 
                       

                    ESP_LOGI(GATTS_TABLE_TAG, "read12 = %s, value len = %d, w12 :%s", ble_read12, param->write.len, ble_w12);
                }

                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_M] ){//换料开关
                     
                    str_save(ble_w13, param->write.value);
                    str_save(ble_read13, param->write.value);
                    write_nvs(ble_read13,BLE_NAMSPACE,BLE_READ13_KEY);
                    ESP_LOGI(GATTS_TABLE_TAG, "read13 = %s, value len = %d, w13 :%s", ble_read13, param->write.len, ble_w13);
                }
                
                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_N] ){//进退料开关
                       xSemaphoreTake(ble_read_x, portMAX_DELAY);
                    str_save(ble_w14, param->write.value);
                    str_save(ble_read14, param->write.value);
                    ESP_LOGI(GATTS_TABLE_TAG, "read14 = %s, value len = %d, w14 :%s", ble_read14, param->write.len, ble_w14);
                       xSemaphoreGive(ble_read_x);
                }

                  if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_O] ){//7
                     
                    str_save(ble_w15, param->write.value);

                    xSemaphoreTake(ble_read_x, portMAX_DELAY);
                    str_save(ble_read15, param->write.value);
                    write_nvs(ble_read15,BLE_NAMSPACE,BLE_READ15_KEY);
                    xSemaphoreGive(ble_read_x);

                    ESP_LOGI(GATTS_TABLE_TAG, "read15 = %s, value len = %d, w15 :%s", ble_read15, param->write.len, ble_w15);
                }

                  if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_P] ){//8
                     
                    str_save(ble_w16, param->write.value);

                    xSemaphoreTake(ble_read_x, portMAX_DELAY);
                    str_save(ble_read16, param->write.value);
                    write_nvs(ble_read16,BLE_NAMSPACE,BLE_READ16_KEY);
                    xSemaphoreGive(ble_read_x);

                    ESP_LOGI(GATTS_TABLE_TAG, "read16 = %s, value len = %d, w16 :%s", ble_read16, param->write.len, ble_w16);
                }

                  if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_Q] ){//退料
                     
                    str_save(ble_w17, param->write.value);

                    xSemaphoreTake(ble_read_x, portMAX_DELAY);
                    str_save(ble_read17, param->write.value);
                    write_nvs(ble_read17,BLE_NAMSPACE,BLE_READ17_KEY);
                    xSemaphoreGive(ble_read_x);

                    ESP_LOGI(GATTS_TABLE_TAG, "read17 = %s, value len = %d, w17 :%s", ble_read17, param->write.len, ble_w17);
                }

                if(param->write.handle == heart_rate_handle_table[IDX_CHAR_VAL_R] ){//debug
                     
                    str_save(ble_w19, param->write.value);
                 
                       

                    ESP_LOGI(GATTS_TABLE_TAG, "read12 = %s, value len = %d, w12 :%s", ble_read19, param->write.len, ble_w19);
                }


                if (heart_rate_handle_table[IDX_CHAR_CFG_A] == param->write.handle && param->write.len == 2){
                    uint16_t descr_value = param->write.value[1]<<8 | param->write.value[0];
                    if (descr_value == 0x0001){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {  
                            notify_data[i] = i % 0xff;
                        }
                        //the size of notify_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                                sizeof(notify_data), notify_data, false);//sizeof(notify_data)字节数是一个int，heart_rate_handle_table[IDX_CHAR_VAL_A],对哪一个特征写入，false代表通知true代表指示======================
                    }else if (descr_value == 0x0002){
                        ESP_LOGI(GATTS_TABLE_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }

                        // if want to change the value in server database, call:
                        // esp_ble_gatts_set_attr_value(heart_rate_handle_table[IDX_CHAR_VAL_A], sizeof(indicate_data), indicate_data);


                        //the size of indicate_data[] need less than MTU size
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, heart_rate_handle_table[IDX_CHAR_VAL_A],
                                            sizeof(indicate_data), indicate_data, true);
                    }
                    else if (descr_value == 0x0000){
                        ESP_LOGI(GATTS_TABLE_TAG, "notify/indicate disable ");
                    }else{
                        ESP_LOGE(GATTS_TABLE_TAG, "unknown descr value");
                        esp_log_buffer_hex(GATTS_TABLE_TAG, param->write.value, param->write.len);
                    }

                }
                /* send response when param->write.need_rsp is true*/
                if (param->write.need_rsp){
                    esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                }
            }else{
                /* handle prepare write */
                example_prepare_write_event_env(gatts_if, &prepare_write_env, param);
            }
      	    break;
        case ESP_GATTS_EXEC_WRITE_EVT:
            // the length of gattc prepare write data must be less than GATTS_DEMO_CHAR_VAL_LEN_MAX.
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
            example_exec_write_event_env(&prepare_write_env, param);
            break;
        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
            break;
        case ESP_GATTS_CONF_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONF_EVT, status = %d, attr_handle %d", param->conf.status, param->conf.handle);
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_CONNECT_EVT:
           {ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_CONNECT_EVT, conn_id = %d", param->connect.conn_id);
            esp_log_buffer_hex(GATTS_TABLE_TAG, param->connect.remote_bda, 6);
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            /* For the iOS system, please refer to Apple official documents about the BLE connection parameters restrictions. */
            conn_params.latency = 0;
            conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
            conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
            conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
            //start sent the update connection parameters to the peer device.
            esp_ble_gap_update_conn_params(&conn_params);}
            gl_con_id = param->connect.conn_id;//保存连接id
            ESP_LOGI(GATTS_TABLE_TAG, "connect successfulid%d", gl_con_id);
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TABLE_TAG, "ESP_GATTS_DISCONNECT_EVT, reason = 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            gl_con_id=0xFFFF;
            break;
        case ESP_GATTS_CREAT_ATTR_TAB_EVT:{//注册att服务成功时的事件
          if (param->add_attr_tab.status != ESP_GATT_OK)    
          {

            ESP_LOGI(GATTS_TABLE_TAG, "create attribute table failed, status ");

          }
          else if (param->add_attr_tab.svc_inst_id == 0)//服务1
          {
            memcpy(heart_rate_handle_table, param->add_attr_tab.handles, sizeof(heart_rate_handle_table));
            esp_err_t ret = esp_ble_gatts_start_service(heart_rate_handle_table[IDX_SVC]);
            if (ret)
            {
                ESP_LOGI(GATTS_TABLE_TAG, "start service failed, error code = %x", ret);
            }
            else
            {
                ESP_LOGI(GATTS_TABLE_TAG, "start service successfully0");
               
            }
          }
        
            
          
        }
        case ESP_GATTS_STOP_EVT:
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        case ESP_GATTS_UNREG_EVT:
        case ESP_GATTS_DELETE_EVT:
        default:
            break;
    }
}




static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{

    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            heart_rate_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TABLE_TAG, "reg app failed, app_id %04x, status %d",
                    param->reg.app_id,
                    param->reg.status);
            return;
        }
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
            if (gatts_if == ESP_GATT_IF_NONE || gatts_if == heart_rate_profile_tab[idx].gatts_if) {
                if (heart_rate_profile_tab[idx].gatts_cb) {
                    heart_rate_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}


void ble_start(){
esp_err_t ret;

    /* Initialize NVS. */
    ret = nvs_flash_init();//===================================================================================================================================================注意不要重复
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();//默认蓝牙配置
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);//使能蓝牙功能低功耗蓝牙
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();//初始化蓝牙
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();//打开蓝牙
    if (ret) {
        ESP_LOGE(GATTS_TABLE_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);//gatts事件回调函数注册
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);//gap事件回调函数注册
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gap register error, error code = %x", ret);
        return;
    }

    ret = esp_ble_gatts_app_register(ESP_APP_ID);//gatts app注册
    if (ret){
        ESP_LOGE(GATTS_TABLE_TAG, "gatts app register error, error code = %x", ret);
        return;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);//设置数据大小一般是20-500，默认是23
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TABLE_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

}


//上报数据
void ble_set_value1(uint8_t value){
 char_value1=value;
 

if (gl_con_id!= 0xFFFF)
{
    ESP_LOGI(GATTS_TABLE_TAG, "send%d indicate%d",gl_gatts_if, gl_con_id);
   //esp_ble_gatts_set_attr_value(heart_rate_handle_table[IDX_CHAR_VAL_A], 2, (const uint8_t*)&char_value);
   esp_ble_gatts_send_indicate(gl_gatts_if, gl_con_id, heart_rate_handle_table[IDX_CHAR_VAL_A], 2, ( uint8_t*)&char_value1, false);


}




}



void ble_nvs_read()
{
    read_nvs(ble_read1,sizeof(ble_read1),BLE_NAMSPACE,BLE_READ1_KEY);
    read_nvs(ble_read2,sizeof(ble_read2),BLE_NAMSPACE,BLE_READ2_KEY);
    read_nvs(ble_read3,sizeof(ble_read3),BLE_NAMSPACE,BLE_READ3_KEY);
    read_nvs(ble_read4,sizeof(ble_read4),BLE_NAMSPACE,BLE_READ4_KEY);
    read_nvs(ble_read5,sizeof(ble_read5),BLE_NAMSPACE,BLE_READ5_KEY);
    read_nvs(ble_read6,sizeof(ble_read6),BLE_NAMSPACE,BLE_READ6_KEY);
    read_nvs(ble_read7,sizeof(ble_read7),BLE_NAMSPACE,BLE_READ7_KEY);   
    read_nvs(ble_read8,sizeof(ble_read8),BLE_NAMSPACE,BLE_READ8_KEY);
	read_nvs(ble_read9,sizeof(ble_read9),BLE_NAMSPACE,BLE_READ9_KEY);
	read_nvs(ble_read10,sizeof(ble_read10),BLE_NAMSPACE,BLE_READ10_KEY);//写入ams信息
    read_nvs(ble_read11,sizeof(ble_read11),BLE_NAMSPACE,BLE_READ11_KEY);//蓝牙写入配网信息
    read_nvs(ble_read15,sizeof(ble_read15),BLE_NAMSPACE,BLE_READ15_KEY);//蓝牙写入配网信息
    read_nvs(ble_read16,sizeof(ble_read16),BLE_NAMSPACE,BLE_READ16_KEY);//蓝牙写入配网信息
    read_nvs(ble_read17,sizeof(ble_read17),BLE_NAMSPACE,BLE_READ17_KEY);//蓝牙写入配网信息
    read_nvs(ble_read18,sizeof(ble_read18),BLE_NAMSPACE,BLE_READ18_KEY);//蓝牙写入配网信息
     read_nvs(ble_read13,sizeof(ble_read13),BLE_NAMSPACE,BLE_READ13_KEY);//蓝牙写入配网信息
}

void ble_nvs_write()
{

write_nvs(ble_read1,BLE_NAMSPACE,BLE_READ1_KEY);
write_nvs(ble_read2,BLE_NAMSPACE,BLE_READ2_KEY);
write_nvs(ble_read3,BLE_NAMSPACE,BLE_READ3_KEY);
write_nvs(ble_read4,BLE_NAMSPACE,BLE_READ4_KEY);
write_nvs(ble_read5,BLE_NAMSPACE,BLE_READ5_KEY);
write_nvs(ble_read6,BLE_NAMSPACE,BLE_READ6_KEY);
write_nvs(ble_read7,BLE_NAMSPACE,BLE_READ7_KEY);
write_nvs(ble_read15,BLE_NAMSPACE,BLE_READ15_KEY); 
write_nvs(ble_read16,BLE_NAMSPACE,BLE_READ16_KEY);
write_nvs(ble_read17,BLE_NAMSPACE,BLE_READ17_KEY);
write_nvs(ble_read18,BLE_NAMSPACE,BLE_READ18_KEY);


}//写入耗材信息
