idf_component_register(
    SRCS "main.c"
         "doa/gcc_phat.c"
         "doa/doa_display_gate.c"
         "doa/doa_event.c"
         "doa/doa_tracker.c"
         "doa/direction_mapping.c"
         "bird_ai/bird_decision.c"
         "bird_ai/ei_inference.cpp"
         "wifi/wifi_server.c"
         "led/led_indicator.c"
         "board_test/board_test.c"
         "examples/example_i2c_scan.c"
         "examples/example_adc.c"
         "examples/example_uart.c"
         "examples/example_rgb.c"
         "examples/example_es7210.c"
         "examples/example_usb.c"
         "examples/example_ble.c"
    INCLUDE_DIRS "."
                 "doa"
                 "bird_ai"
                 "wifi"
                 "led"
                 "board_test"
                 "examples"
    PRIV_REQUIRES esp_driver_i2s esp_driver_gpio esp_driver_i2c
                  esp_driver_ledc esp_driver_rmt esp_driver_uart
                  esp_driver_usb_serial_jtag
                  esp_wifi esp_http_server esp_http_client nvs_flash esp_netif
                  esp_event esp_adc edge_impulse led_strip bt
    EMBED_TXTFILES "wifi/web_page.html"
)

# Edge Impulse third-party headers may trigger -Werror diagnostics under newer GCC.
# Keep strict checks for the rest of the component, but allow this translation unit.
# Use COMPILE_OPTIONS (appended last on the command line) with per-diagnostic
# suppression: bare -Wno-error does not override IDF's global -Werror=all in
# GCC 14, but -Wno-error=<class> (like the -Wno-error=unused-* flags below) does.
set_source_files_properties(bird_ai/ei_inference.cpp PROPERTIES
    COMPILE_OPTIONS "-Wno-error=uninitialized;-Wno-error=maybe-uninitialized")
target_compile_options(${COMPONENT_LIB} PRIVATE
    -Wno-error=format
    -Wno-error=unused-but-set-parameter
    -Wno-error=deprecated-declarations
    -Wno-error=missing-field-initializers
    -Wno-error=unused-variable
    -Wno-error=unused-function
    # sqrtf 改用硬件指令 (结果仍是 IEEE 正确舍入, 不改变数值语义)
    -fno-math-errno
)
