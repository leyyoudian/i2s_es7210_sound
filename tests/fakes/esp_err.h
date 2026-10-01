#ifndef TEST_FAKE_ESP_ERR_H
#define TEST_FAKE_ESP_ERR_H

typedef int esp_err_t;

#define ESP_OK 0
#define ESP_FAIL (-1)

static inline const char *esp_err_to_name(esp_err_t err)
{
    return err == ESP_OK ? "ESP_OK" : "ESP_FAIL";
}

#endif
