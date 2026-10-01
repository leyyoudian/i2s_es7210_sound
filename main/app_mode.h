#ifndef APP_MODE_H
#define APP_MODE_H

/* ================================================================ */
/*  应用模式 2 选 1                                                   */
/*  修改 APP_MODE 后重新编译烧录即可切换功能:                          */
/*   - APP_MODE_SOUND_LOCALIZATION : 声源定位 (DOA)                    */
/*       麦克风阵列 → GCC-PHAT/SRP → 跟踪 → 16 灯方向环 + 网页罗盘     */
/*       (不跑 Edge Impulse 推理, 省 64KB DRAM 与 CPU0 占用)           */
/*   - APP_MODE_SOUND_CLASSIFY     : 音色识别                          */
/*       麦克风 → 抗混叠抽取 → Edge Impulse INT8 推理                  */
/*       检测到目标时灯环全亮, 网页显示置信度 (无罗盘)                 */
/* ================================================================ */

#define APP_MODE_SOUND_LOCALIZATION   1
#define APP_MODE_SOUND_CLASSIFY       2

#define APP_MODE  APP_MODE_SOUND_LOCALIZATION

#if APP_MODE != APP_MODE_SOUND_LOCALIZATION && \
    APP_MODE != APP_MODE_SOUND_CLASSIFY
#error "APP_MODE 必须为 APP_MODE_SOUND_LOCALIZATION 或 APP_MODE_SOUND_CLASSIFY 之一"
#endif

#endif /* APP_MODE_H */
