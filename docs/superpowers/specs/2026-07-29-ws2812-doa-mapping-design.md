# WS2812 DOA Mapping Design

## Goal

Adapt the existing four-microphone sound direction firmware to the revised PCB: retain ES7210 four-channel TDM capture, use the confirmed physical microphone orientation, and replace four discrete direction LEDs with ten WS2812 pixels driven from GPIO3.

## Audio Mapping

The ES7210 TDM slot order remains MIC1, MIC2, MIC3, MIC4. Physical positions are MIC3 at 0 degrees, MIC4 at 90 degrees, MIC2 at 180 degrees, and MIC1 at 270 degrees. GCC-PHAT therefore compares channel 2 against channel 1 for the X axis and channel 3 against channel 0 for the Y axis.

The new SPM0687LR5H-1 differential analog microphones do not change the ESP32 I2S format. Their gain must be validated on hardware, but the current gain setting remains unchanged in this scoped change.

## LED Mapping

The electrical chain is GPIO3 -> U24 -> ... -> U33. Pixel angles in that order are 22.5, 337.5, 315, 292.5, 247.5, 202.5, 157.5, 135, 112.5, and 67.5 degrees. The nearest pixel is selected using absolute circular distance. A detected source lights one green pixel; no active source clears the strip.

The implementation uses Espressif's `led_strip` component with the RMT backend. The existing `led_init`, `led_all_off`, and `led_update_direction` API remains stable for the audio task.

## Error Handling And Verification

LED initialization failures are logged and leave direction updates as safe no-ops. Pure angle mapping is exposed as a deterministic function and covered by host-side tests for exact positions, wraparound, and gaps. The complete ESP-IDF application is then built for ESP32-S3.
