#ifndef WIFI_SERVER_H
#define WIFI_SERVER_H

#include <stdbool.h>

/* Wi-Fi 凭据 (请修改为你的网络) */
#define WIFI_SSID       "CMCC-U93N"
#define WIFI_PASSWORD   "v9r47296"

/*
 * 初始化 Wi-Fi Station 模式并启动 HTTP 服务器.
 * 最多重试连接 3 次, 失败后以离线模式继续运行 (不启动 HTTP 服务).
 */
void wifi_server_init(void);

/*
 * 向所有 SSE 客户端推送检测数据
 *   angle:         方位角 (度)
 *   bird_conf:     鸟声置信度
 *   noise_conf:    噪声置信度
 *   bird_detected: 是否判定为鸟声
 */
void wifi_server_set_mode(const char *mode);
void wifi_server_push_data(float angle, float bird_conf,
                           float noise_conf, bool bird_detected);

#endif /* WIFI_SERVER_H */