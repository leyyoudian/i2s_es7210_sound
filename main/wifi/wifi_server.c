/*
 * Wi-Fi Station + HTTP 服务器
 * - 连接到指定 AP (SSID/密码编译时写死)
 * - 启动 HTTP 服务, 提供 dashboard HTML 页面
 * - SSE (Server-Sent Events) 实时推送检测数据到浏览器
 */

#include "wifi_server.h"
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

static const char *TAG = "wifi_srv";

/* Wi-Fi 连接事件组 */
static EventGroupHandle_t s_wifi_eg;
#define WIFI_CONNECTED_BIT  BIT0
#define WIFI_FAIL_BIT      BIT1

#define WIFI_MAX_RETRY     3   /* 最多重试 3 次 */
static int s_wifi_retry_count = 0;

/* HTTP 服务器句柄 */
static httpd_handle_t s_httpd = NULL;

/* SSE 客户端 socket fd (-1 表示空闲) */
#define MAX_SSE_CLIENTS  4
static int s_sse_fds[MAX_SSE_CLIENTS] = {-1, -1, -1, -1};

static int sse_send_all(int fd, const char *data, size_t len)
{
    size_t sent = 0;
    while (sent < len) {
        int ret = httpd_socket_send(s_httpd, fd, data + sent, len - sent, 0);
        if (ret <= 0) {
            return -1;
        }
        sent += (size_t)ret;
    }
    return 0;
}

/* 发送一个符合 HTTP chunked 传输编码的数据块 */
static int sse_send_chunked_frame(int fd, const char *payload, size_t payload_len)
{
    char chunk_hdr[16];
    int hdr_len = snprintf(chunk_hdr, sizeof(chunk_hdr), "%x\r\n", (unsigned)payload_len);
    if (hdr_len <= 0 || hdr_len >= (int)sizeof(chunk_hdr)) {
        errno = EINVAL;
        return -1;
    }

    if (sse_send_all(fd, chunk_hdr, (size_t)hdr_len) < 0) {
        return -1;
    }
    if (sse_send_all(fd, payload, payload_len) < 0) {
        return -1;
    }
    if (sse_send_all(fd, "\r\n", 2) < 0) {
        return -1;
    }
    return 0;
}

static bool sse_is_non_fatal_errno(int err)
{
    return (err == EAGAIN) || (err == EWOULDBLOCK) || (err == ENOMEM);
}

/* 嵌入的 HTML 文件 (通过 CMake EMBED_TXTFILES 链接) */
extern const char web_page_html_start[] asm("_binary_web_page_html_start");
extern const char web_page_html_end[]   asm("_binary_web_page_html_end");

/* ---------------------------------------------------------------- */
/*  Wi-Fi 事件回调                                                    */
/* ---------------------------------------------------------------- */
static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        /* 等待 200ms 让射频稳定后再发起首次连接, 避免偶发 assoc 失败 */
        vTaskDelay(pdMS_TO_TICKS(200));
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        wifi_event_sta_disconnected_t *evt = (wifi_event_sta_disconnected_t *)data;
        s_wifi_retry_count++;
        if (s_wifi_retry_count <= WIFI_MAX_RETRY) {
            ESP_LOGW(TAG, "Wi-Fi 断开 (reason=%d), 第 %d/%d 次重试...",
                     evt->reason, s_wifi_retry_count, WIFI_MAX_RETRY);
            /* 指数退避: 第 n 次重试前等 2^(n-1) 秒 (1, 2, 4, 8, 5) */
            int delay_s = 1 << (s_wifi_retry_count - 1);
            if (delay_s > 5) delay_s = 5;
            vTaskDelay(pdMS_TO_TICKS(delay_s * 1000));
            /* 先断开复位状态机, 再重新连接, 避免 auth 状态残留 */
            esp_wifi_disconnect();
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_wifi_connect();
        } else {
            ESP_LOGE(TAG, "Wi-Fi 连接失败 %d 次, 放弃重试, 继续离线运行", WIFI_MAX_RETRY);
            xEventGroupSetBits(s_wifi_eg, WIFI_FAIL_BIT);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        s_wifi_retry_count = 0;
        ip_event_got_ip_t *evt = (ip_event_got_ip_t *)data;
        ESP_LOGI(TAG, "已连接, IP: " IPSTR, IP2STR(&evt->ip_info.ip));
        xEventGroupSetBits(s_wifi_eg, WIFI_CONNECTED_BIT);
    }
}

static bool wifi_init_sta(void)
{
    /* NVS 初始化 (Wi-Fi 驱动依赖) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    s_wifi_eg = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init_cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));

    wifi_config_t sta_cfg = {};
    /* 复制 SSID 和密码到配置结构 */
    strncpy((char *)sta_cfg.sta.ssid, WIFI_SSID, sizeof(sta_cfg.sta.ssid) - 1);
    strncpy((char *)sta_cfg.sta.password, WIFI_PASSWORD, sizeof(sta_cfg.sta.password) - 1);
    sta_cfg.sta.scan_method = WIFI_FAST_SCAN;
    sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    sta_cfg.sta.threshold.rssi = -127;
    sta_cfg.sta.pmf_cfg = (wifi_pmf_config_t){ .capable = true, .required = false };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
    /* 设置国家码为 CN (中国), 确保扫描信道范围一致 */
    esp_wifi_set_country(&(wifi_country_t){
        .cc = "CN",
        .schan = 1,
        .nchan = 13,
        .policy = WIFI_COUNTRY_POLICY_AUTO
    });
    /* 关闭 WiFi 省电模式, 避免间歇性断连 */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    /* 固定 TX 功率为 19.5dBm (允许范围内最大值), 消除芯片个体差异 */
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(78));   /* 78 = 19.5dBm (0.25dBm/unit) */

    ESP_LOGI(TAG, "正在连接 \"%s\" ...", WIFI_SSID);
    EventBits_t bits = xEventGroupWaitBits(s_wifi_eg,
                        WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                        pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Wi-Fi 连接成功");
        return true;
    } else {
        ESP_LOGW(TAG, "Wi-Fi 连接失败, 系统将以离线模式运行");
        return false;
    }
}

/* ---------------------------------------------------------------- */
/*  HTTP 请求处理                                                     */
/* ---------------------------------------------------------------- */

/* GET / -> 返回 dashboard HTML */
static esp_err_t handler_root(httpd_req_t *req)
{
    size_t len = web_page_html_end - web_page_html_start;
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_send(req, web_page_html_start, len);
    return ESP_OK;
}

/* GET /events -> SSE 事件流 */
static esp_err_t handler_sse(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);

    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Accel-Buffering", "no");

    /* 发送初始握手消息 */
    const char *hello = "data: {\"status\":\"connected\"}\n\n";
    if (httpd_resp_send_chunk(req, hello, strlen(hello)) != ESP_OK) {
        return ESP_FAIL;
    }

    /* 注册此 fd 到 SSE 客户端列表 */
    for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
        if (s_sse_fds[i] == -1) {
            s_sse_fds[i] = fd;
            ESP_LOGI(TAG, "SSE 客户端连接 (fd=%d, slot=%d)", fd, i);
            break;
        }
    }
    return ESP_OK;
}

static void start_http_server(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.lru_purge_enable = true;
    cfg.max_open_sockets = 7;

    if (httpd_start(&s_httpd, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "HTTP 服务器启动失败");
        return;
    }

    httpd_uri_t uri_root = {
        .uri = "/",
        .method = HTTP_GET,
        .handler = handler_root,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_httpd, &uri_root);

    httpd_uri_t uri_sse = {
        .uri = "/events",
        .method = HTTP_GET,
        .handler = handler_sse,
        .user_ctx = NULL
    };
    httpd_register_uri_handler(s_httpd, &uri_sse);

    ESP_LOGI(TAG, "HTTP 服务器已启动 (端口 %d)", cfg.server_port);
}

/* ================================================================ */
/*  公开接口                                                         */
/* ================================================================ */

void wifi_server_init(void)
{
    if (!wifi_init_sta()) {
        ESP_LOGW(TAG, "跳过 HTTP 服务器启动 (离线模式)");
        return;
    }
    start_http_server();
}

static const char *s_app_mode = "doa";

void wifi_server_set_mode(const char *mode)
{
    s_app_mode = mode;
}

void wifi_server_push_data(float angle, float bird_conf,
                           float noise_conf, bool bird_detected)
{
    if (!s_httpd) return;

    char buf[224];
    int len = snprintf(buf, sizeof(buf),
        "data: {\"mode\":\"%s\",\"angle\":%.1f,\"bird\":%.4f,\"noise\":%.4f,\"detected\":%s}\n\n",
        s_app_mode, angle, bird_conf, noise_conf,
        bird_detected ? "true" : "false");

    for (int i = 0; i < MAX_SSE_CLIENTS; i++) {
        if (s_sse_fds[i] < 0) continue;

        if (sse_send_chunked_frame(s_sse_fds[i], buf, (size_t)len) < 0) {
            int err = errno;
            if (sse_is_non_fatal_errno(err)) {
                ESP_LOGW(TAG, "SSE 发送阻塞 (slot=%d, errno=%d), 本帧跳过", i, err);
                continue;
            }

            ESP_LOGW(TAG, "SSE 客户端断开 (slot=%d, errno=%d)", i, err);
            s_sse_fds[i] = -1;
        }
    }
}