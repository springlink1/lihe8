#include <string.h>
#include "nvs_flash.h"


void str_save(char A[], auto B){

    memset(A, 0, 20);
    memcpy(A, B, 20);

}

static size_t read_nvs(char* ssid,int maxlen,const char* NAME_SPACE,const char* NVS_KEY)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret_val = ESP_FAIL;
    size_t required_size = 0;
    ESP_ERROR_CHECK(nvs_open(NAME_SPACE, NVS_READWRITE, &nvs_handle));
    ret_val = nvs_get_str(nvs_handle, NVS_KEY, NULL, &required_size);
    if(ret_val == ESP_OK && required_size <= maxlen)
    {
        nvs_get_str(nvs_handle,NVS_KEY,ssid,&required_size);
    }
    else
        required_size = 0;
    nvs_close(nvs_handle);
    return required_size;
}


static esp_err_t write_nvs(char* ssid, const char* NAME_SPACE, const char* NVS_KEY)
{
    nvs_handle_t nvs_handle;
    esp_err_t ret;
    ESP_ERROR_CHECK(nvs_open(NAME_SPACE, NVS_READWRITE, &nvs_handle));
    
    ret = nvs_set_str(nvs_handle, NVS_KEY, ssid);
    nvs_commit(nvs_handle);
    nvs_close(nvs_handle);
    return ret;
}
