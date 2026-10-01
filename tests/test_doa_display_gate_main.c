#include <stdio.h>

#include "doa_display_gate.h"

int main(void)
{
    doa_display_gate_t gate;
    float display_angle = -1.0f;
    doa_display_gate_init(&gate);

    /* 首次激活: display_angle 直接取当前角度 */
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 22.5f, &display_angle)) {
        fprintf(stderr, "valid direction did not enable display\n");
        return 1;
    }
    if (display_angle != 22.5f) {
        fprintf(stderr, "initial display angle wrong: %.1f\n", display_angle);
        return 7;
    }

    /* 显示角度死区 5°: 灯位未变时小幅波动不更新 display_angle */
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 24.0f, &display_angle)) return 20;
    if (display_angle != 22.5f) {
        fprintf(stderr, "dead zone leaked small jitter: %.1f\n", display_angle);
        return 21;
    }
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 28.0f, &display_angle)) return 22;
    if (display_angle != 28.0f) {
        fprintf(stderr, "dead zone froze real movement: %.1f\n", display_angle);
        return 23;
    }

    /* 灯位切换滞回 14°: 距 held 灯位中心 13.5° 时即使最近灯已变了也保持原灯位 */
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 9.0f, &display_angle)) return 24;
    if (display_angle != 9.0f) {
        fprintf(stderr, "hysteresis branch lost display update: %.1f\n",
                display_angle);
        return 25;
    }

    /* 超出滞回: 候选确认 2 帧后才切换 (CANDIDATE_CONFIRM_FRAMES=2) */
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 8.0f, &display_angle)) return 26;
    if (display_angle != 9.0f) {
        fprintf(stderr, "single candidate changed display angle: %.1f\n",
                display_angle);
        return 27;
    }
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 8.0f, &display_angle)) return 28;
    if (display_angle != 8.0f) {
        fprintf(stderr, "stable candidate did not switch: %.1f\n",
                display_angle);
        return 29;
    }

    /* 候选来回变化时保持原显示 */
    doa_display_gate_init(&gate);
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 22.5f, &display_angle)) return 8;
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 337.5f, &display_angle)) return 9;
    if (display_angle != 22.5f) {
        fprintf(stderr, "single candidate changed display angle: %.1f\n",
                display_angle);
        return 10;
    }
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 315.0f, &display_angle)) return 11;
    if (display_angle != 22.5f) {
        fprintf(stderr, "changing candidates flashed display angle: %.1f\n",
                display_angle);
        return 12;
    }
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 337.5f, &display_angle)) return 13;
    if (display_angle != 22.5f) {
        fprintf(stderr, "candidate switch early: %.1f\n", display_angle);
        return 14;
    }
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 337.5f, &display_angle)) return 15;
    if (display_angle != 337.5f) {
        fprintf(stderr, "confirmed candidate did not switch: %.1f\n",
                display_angle);
        return 16;
    }

    /* 弱能量拒绝: 连续 STALE_WEAK_REJECT_FRAMES=15 帧后熄灭
     * (能量须低于 WEAK_REJECT_ENERGY=0.0015 才计入拒绝) */
    doa_display_gate_init(&gate);
    if (!doa_display_gate_update(&gate, true, true, true, 0.0100f,
                                 22.5f, &display_angle)) return 30;
    for (int i = 0; i < 14; ++i) {
        if (!doa_display_gate_update(&gate, false, true, false, 0.0012f,
                                     337.5f, &display_angle)) {
            fprintf(stderr, "weak rejected frame %d dropped display early\n", i);
            return 31;
        }
    }
    if (doa_display_gate_update(&gate, false, true, false, 0.0012f,
                                337.5f, &display_angle)) {
        fprintf(stderr, "stale weak rejected frames kept old display\n");
        return 32;
    }

    /* 重新激活 + 高能量拒绝帧保持显示 */
    if (!doa_display_gate_update(&gate, true, true, true, 0.0040f,
                                 22.5f, &display_angle)) {
        fprintf(stderr, "new valid direction did not restore display\n");
        return 33;
    }
    if (!doa_display_gate_update(&gate, false, true, false, 0.0060f,
                                 67.5f, &display_angle)) {
        fprintf(stderr, "high-energy rejected frame should hold display\n");
        return 34;
    }

    /* 声源失活: 立即熄灭 */
    if (doa_display_gate_update(&gate, false, false, false, 0.0008f,
                                67.5f, &display_angle)) {
        fprintf(stderr, "inactive source should disable display\n");
        return 35;
    }

    return 0;
}
