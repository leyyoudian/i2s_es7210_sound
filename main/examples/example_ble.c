/*
 * 例程 7: 蓝牙 BLE (NimBLE) —— 广播 + GATT 回显服务
 *
 * ESP32-S3 内置 BLE 5.0 控制器, 与 Wi-Fi 共用天线, 无需任何外接模块!
 * 本例程演示:
 *   1. 广播设备名 "ES7210-DOA"
 *   2. 自定义 GATT 服务 (UUID 0x1234):
 *        特征 0x1235 (读/写/通知): 手机写入任意数据, 板子原样回显
 *        特征 0x1236 (读):        读取设备状态字符串
 *
 * 引脚速查:
 *   蓝牙为芯片内置功能, 不占用任何 GPIO. USB-C 供电即可工作.
 *
 * 使用方法:
 *   1. 手机安装 nRF Connect 或微信小程序 "BLE 调试助手"
 *   2. 扫描到 "ES7210-DOA" 并连接
 *   3. 向特征 0x1235 写入数据 -> 收到相同回显
 *   4. 读取特征 0x1236 -> 返回设备信息字符串
 */

#include <string.h>
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* NimBLE 主机栈 */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "examples.h"

static const char *TAG = "ble_demo";

#define BLE_DEVICE_NAME      "ES7210-DOA"
#define BLE_APP_ADV_INTERVAL 160   /* 100ms, 单位 0.625ms */

/* 自定义 GATT 服务: 0x1234 (16-bit UUID) */
static const ble_uuid16_t s_svc_uuid = BLE_UUID16_INIT(0x1234);
/* 特征1: 回显 (读/写/通知) */
static const ble_uuid16_t s_chr_echo_uuid = BLE_UUID16_INIT(0x1235);
/* 特征2: 设备状态 (只读) */
static const ble_uuid16_t s_chr_status_uuid = BLE_UUID16_INIT(0x1236);

static uint8_t s_echo_buf[64];
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;

static const char s_status_str[] = "Bird-DOA Board v2 | Fs=48kHz | r=40mm | BLE OK";

/* ================================================================ */
/*  GATT 服务                                                        */
/* ================================================================ */

/* 特征 0x1235: 回显 —— 写: 收到后 notify 原样发回; 读: 返回最后写入内容 */
static int gatt_echo_access(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len > sizeof(s_echo_buf)) {
            len = sizeof(s_echo_buf);
        }
        os_mbuf_copydata(ctxt->om, 0, len, s_echo_buf);

        ESP_LOGI(TAG, "[echo] 收到 %d 字节: %.*s", len, (int)len, s_echo_buf);

        /* notify 回显给手机 */
        struct os_mbuf *om = ble_hs_mbuf_from_flat(s_echo_buf, len);
        if (om != NULL) {
            ble_gatts_notify_custom(conn_handle, attr_handle, om);
        }
        return 0;
    }

    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        uint16_t len = strlen((char *)s_echo_buf);
        return os_mbuf_append(ctxt->om, s_echo_buf, len);
    }

    return BLE_ATT_ERR_UNLIKELY;
}

/* 特征 0x1236: 读 (返回设备状态) */
static int gatt_status_read(uint16_t conn_handle, uint16_t attr_handle,
                            struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return os_mbuf_append(ctxt->om, s_status_str, strlen(s_status_str));
}

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &s_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]){
            {
                .uuid = &s_chr_echo_uuid.u,
                .access_cb = gatt_echo_access,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE |
                         BLE_GATT_CHR_F_NOTIFY,
            },
            {
                .uuid = &s_chr_status_uuid.u,
                .access_cb = gatt_status_read,
                .flags = BLE_GATT_CHR_F_READ,
            },
            { 0 }, /* 结束 */
        },
    },
    { 0 }, /* 结束 */
};

/* ================================================================ */
/*  GAP 事件                                                         */
/* ================================================================ */

static void ble_app_advertise(void); /* 前向声明 */

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            s_conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "已连接! conn_handle=%d", s_conn_handle);
        } else {
            s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
            ESP_LOGI(TAG, "连接失败, 重新广播...");
            ble_app_advertise();
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ESP_LOGI(TAG, "断开连接, 重新广播...");
        ble_app_advertise();
        return 0;

    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "广播结束, 重启广播");
        ble_app_advertise();
        return 0;

    default:
        return 0;
    }
}

/* ================================================================ */
/*  主机栈启动                                                       */
/* ================================================================ */

static void ble_host_task(void *param)
{
    nimble_port_run();   /* 运行 NimBLE 主机栈事件循环, 不返回 */
    nimble_port_freertos_deinit();
}

/* 配置广播数据并开始广播 */
static void ble_app_advertise(void)
{
    struct ble_hs_adv_fields fields;
    struct ble_gap_adv_params adv_params;
    uint8_t own_addr_type;
    int rc;

    memset(&fields, 0, sizeof(fields));

    /* 广播标志: 通用发现模式 + 不支持 BR/EDR (纯 BLE) */
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    /* 在广播包中包含完整设备名 */
    fields.name = (uint8_t *)BLE_DEVICE_NAME;
    fields.name_len = strlen(BLE_DEVICE_NAME);
    fields.name_is_complete = 1;

    /* 包含 TX 功率电平 */
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "设置广播字段失败, rc=%d", rc);
        return;
    }

    /* 广播前根据已就绪的地址推断正确的 own_addr_type. */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "推断广播地址类型失败, rc=%d", rc);
        return;
    }

    /* 广播参数: 必须显式给出 conn_mode/disc_mode 和广播间隔.
     * 传 NULL 会导致 conn_mode/disc_mode 均为 NON, ble_gap_adv_start
     * 校验后返回 BLE_HS_EINVAL (rc=3). */
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   /* 可连接 */
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   /* 通用可发现 */
    /* 广播间隔: 单位 0.625ms, 160 -> 100ms. 若为 0 则广播会立即结束
     * 并反复触发 ADV_COMPLETE 死循环. */
    adv_params.itvl_min = BLE_APP_ADV_INTERVAL;
    adv_params.itvl_max = BLE_APP_ADV_INTERVAL;

    /* 开始广播: duration_ms 用 BLE_HS_FOREVER 表示无限广播,
     * 由 GAP 事件(连接/断开)来停止, 而不是靠定时结束. */
    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER,
                           &adv_params, gap_event_handler, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "启动广播失败, rc=%d", rc);
        return;
    }

    ESP_LOGI(TAG, "BLE 广播中, 设备名: %s", BLE_DEVICE_NAME);
}

static void ble_app_on_sync(void)
{
    int rc;

    /* 设置设备名到 GAP 服务 */
    ble_svc_gap_device_name_set(BLE_DEVICE_NAME);

    /* 确保有合法身份地址 (public 优先). 这是广播能成功的前提:
     * 缺少这一步直接调用 ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC)
     * 会返回 BLE_HS_EINVAL (rc=3). */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
        ESP_LOGE(TAG, "确保公共地址失败, rc=%d", rc);
        return;
    }

    /* 配置广播数据并开始广播 */
    ble_app_advertise();
}

static void ble_app_on_reset(int reason)
{
    ESP_LOGE(TAG, "BLE 复位, reason=%d", reason);
}

void example_ble_start(void)
{
    ESP_LOGI(TAG, "=== BLE 例程: 广播 %s + GATT 回显 ===");
    ESP_LOGI(TAG, "引脚: 蓝牙为内置功能, 无需接线 (与 Wi-Fi 共用天线)");

    /* NVS (NimBLE 用于存储绑定信息) */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    /* 初始化 NimBLE 主机栈 (包含蓝牙控制器) */
    ESP_ERROR_CHECK(nimble_port_init());

    /* 注册 GATT 服务 */
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_gatts_count_cfg(gatt_svcs);
    ble_gatts_add_svcs(gatt_svcs);

    /* 同步回调: 主机栈就绪后开始广播 */
    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;

    /* 启动主机栈任务 */
    nimble_port_freertos_init(ble_host_task);

    ESP_LOGI(TAG, "请在手机用 BLE 调试工具连接 %s", BLE_DEVICE_NAME);
    ESP_LOGI(TAG, "向特征 0x1235 写入数据可收到回显; 读 0x1236 可获取设备信息");
    /* 主任务退出, 主机栈在自己的任务中运行 */
    vTaskDelete(NULL);
}