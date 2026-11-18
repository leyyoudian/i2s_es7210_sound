#include "doa_pcm_stream.h"
#include "driver/usb_serial_jtag.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "esp_timer.h"
#include "freertos/task.h"

static i2s_chan_handle_t s_rx;
static int s_rate;
static volatile uint32_t *s_overflows;
static int16_t s_pcm[1024 * 4];

static bool write_all(const void *data, size_t size)
{
    const uint8_t *p = data;
    while (size) {
        int n = usb_serial_jtag_write_bytes(p, size, pdMS_TO_TICKS(200));
        if (n <= 0) return false;
        p += n;
        size -= n;
    }
    return true;
}

static void stream_task(void *arg)
{
    (void)arg;
    usb_serial_jtag_driver_config_t cfg = {.tx_buffer_size = 16384, .rx_buffer_size = 256};
    if (!usb_serial_jtag_is_driver_installed()) {
        ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
    }
    bool running = false;
    uint32_t sequence = 0, overflow_base = 0;
    for (;;) {
        char command = 0;
        int received = usb_serial_jtag_read_bytes(&command, 1,
                          running ? 0 : portMAX_DELAY);
        if (received == 1 && command == 'S' && running) {
            ESP_ERROR_CHECK(i2s_channel_disable(s_rx));
            running = false;
        }
        if (received == 1 && command == 'R' && !running) {
            sequence = 0;
            overflow_base = *s_overflows;
            ESP_ERROR_CHECK(i2s_channel_enable(s_rx));
            running = true;
        }
        if (!running) continue;
        size_t count = 0;
        if (i2s_channel_read(s_rx, s_pcm, sizeof(s_pcm), &count,
                             pdMS_TO_TICKS(500)) != ESP_OK || !count) continue;
        /* Little-endian header: 8-byte magic, seq, frames, Fs, DMA overflow,
         * end-of-read device milliseconds, CRC32 of exactly frames*8 bytes. */
        uint32_t header[8] = {
            0x50414f44, 0x21324d43, sequence++, count / 8, (uint32_t)s_rate,
            *s_overflows - overflow_base, (uint32_t)(esp_timer_get_time() / 1000),
            esp_rom_crc32_le(0, (const uint8_t *)s_pcm, count),
        };
        if (!write_all(header, sizeof(header)) || !write_all(s_pcm, count)) {
            ESP_ERROR_CHECK(i2s_channel_disable(s_rx));
            running = false;
        }
    }
}

void doa_pcm_stream_start(i2s_chan_handle_t rx, int sample_rate,
                          volatile uint32_t *overflows)
{
    s_rx = rx;
    s_rate = sample_rate;
    s_overflows = overflows;
    /* No console writes may interleave the binary protocol. Normal DOA,
     * classifier and networking tasks are not started in this build. */
    esp_log_level_set("*", ESP_LOG_NONE);
    configASSERT(xTaskCreatePinnedToCore(stream_task, "pcm_stream", 4096,
                                        NULL, 5, NULL, 1) == pdPASS);
}
