#include <math.h>
#include <stdint.h>
#include <stdio.h>

#include "gcc_phat.h"

/* 与固件几何/配置保持一致:
 *  - 阵列半径 40mm (MIC_ARRAY_RADIUS_MM), 与 gcc_phat.c 的 s_mic_pos_m 相同
 *  - 48kHz/1024 (当前固件配置, GCC_FFT_SIZE=1024) 与 16kHz/1024 (旧配置) */
#define N_MAX 2048

static uint32_t rng_state = 1;
static int16_t base_signal[N_MAX + 32];
static int16_t mic_data[4][N_MAX];

static float circular_error(float a, float b)
{
    float d = fabsf(a - b);
    return d > 180.0f ? 360.0f - d : d;
}

static void make_signal(float angle_deg, int n, int fs)
{
    const float positions[4][2] = {
        {0.0f, -0.040f}, {-0.040f, 0.0f},
        {0.040f, 0.0f}, {0.0f, 0.040f},
    };
    const float a = angle_deg * 3.14159265358979323846f / 180.0f;
    const float ux = cosf(a), uy = sinf(a);

    for (int i = 0; i < n + 32; ++i) {
        rng_state = rng_state * 1664525u + 1013904223u;
        base_signal[i] = (int16_t)(((rng_state >> 16) & 0xffffu) - 32768);
    }

    for (int ch = 0; ch < 4; ++ch) {
        int delay = (int)lrintf(-(positions[ch][0] * ux + positions[ch][1] * uy)
                                * (float)fs / 343.0f);
        for (int i = 0; i < n; ++i) {
            mic_data[ch][i] = base_signal[i + 16 - delay];
        }
    }
}

static int run_case_set(int fs, int n)
{
    static const float cases[] = {0.0f, 45.0f, 90.0f, 180.0f, 270.0f, 315.0f};
    const int16_t *mics[4] = {mic_data[0], mic_data[1], mic_data[2], mic_data[3]};

    gcc_phat_init(n);
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        gcc_phat_result_t result = {0};
        /* A continuous source must converge after a direction change; during
         * the transition, ambiguous mixtures may correctly be rejected. */
        for (int frame = 0; frame < 12; ++frame) {
            make_signal(cases[i], n, fs);
            gcc_phat_estimate_4ch(mics, n, fs, &result);
        }
        if (!result.valid) {
            fprintf(stderr, "[fs=%d] %.1f degrees did not converge\n", fs, cases[i]);
            return 1;
        }
        if (circular_error(result.angle_deg, cases[i]) > 10.0f) {
            fprintf(stderr, "[fs=%d] expected %.1f, got %.1f (confidence %.3f)\n",
                    fs, cases[i], result.angle_deg, result.confidence);
            return 1;
        }
        /* 干净信号下对侧镜像峰应远低于主峰 (镜像含糊判定门限 0.60) */
        if (result.mirror_ratio >= 0.60f) {
            fprintf(stderr, "[fs=%d] %.1f degrees mirror_ratio %.3f too high\n",
                    fs, cases[i], result.mirror_ratio);
            return 5;
        }
    }

    make_signal(90.0f, n, fs);
    for (int i = 0; i < n; ++i) mic_data[0][i] = 0;
    gcc_phat_result_t degraded = {0};
    for (int frame = 0; frame < 3; ++frame) {
        if (!gcc_phat_estimate_4ch(mics, n, fs, &degraded) || !degraded.valid) {
            fprintf(stderr, "[fs=%d] three-microphone fallback was rejected\n", fs);
            return 2;
        }
    }
    if (degraded.active_mic_mask != 0x0e || degraded.active_pair_count != 3) {
        fprintf(stderr, "[fs=%d] bad health mask=0x%x pairs=%u\n",
                fs, degraded.active_mic_mask, degraded.active_pair_count);
        return 3;
    }
    if (circular_error(degraded.angle_deg, 90.0f) > 10.0f) {
        fprintf(stderr, "[fs=%d] three-microphone expected 90, got %.1f\n",
                fs, degraded.angle_deg);
        return 4;
    }
    return 0;
}

static int run_fractional_delay_cases(void)
{
    const double frequencies[] = {433,727,1193,1747,2411,3347,4519,5723};
    const double positions[4][2] = {{0,-.04},{-.04,0},{.04,0},{0,.04}};
    const double pi = 3.14159265358979323846;
    const int16_t *mics[4] = {mic_data[0],mic_data[1],mic_data[2],mic_data[3]};
    for (int angle=0;angle<360;angle+=5) {
        double radians = angle*pi/180.;
        for (int ch=0;ch<4;++ch) {
            double delay = -(positions[ch][0]*cos(radians)+positions[ch][1]*sin(radians))/343.;
            for (int i=0;i<1024;++i) {
                double sample=0;
                for (int f=0;f<8;++f) sample+=sin(2*pi*frequencies[f]*(i/48000.-delay)+.37*f*f);
                mic_data[ch][i]=(int16_t)lrint(sample*1200);
            }
        }
        gcc_phat_reset_history();
        gcc_phat_result_t result;
        gcc_phat_estimate_4ch(mics,1024,48000,&result);
        if (!result.valid || circular_error(result.angle_deg,(float)angle)>.5f) {
            fprintf(stderr,"fractional delay: expected %d, got %.3f\n",angle,result.angle_deg);
            return 10;
        }
    }
    return 0;
}

static int run_two_source_cases(void) {
 const double pos[4][2]={{0,-.04},{-.04,0},{.04,0},{0,.04}};
 const double cases[4][2]={{0,90},{20,130},{300,60},{0,180}};
 const int16_t *m[4]={mic_data[0],mic_data[1],mic_data[2],mic_data[3]};
 for(int c=0;c<4;++c) {
  gcc_phat_reset_history();
  gcc_phat_set_score_ema_alpha(.33f);
  gcc_phat_result_t r;
  for(int frame=0;frame<24;++frame) {
   for(int ch=0;ch<4;++ch) for(int i=0;i<1024;++i) {
    double x=0;
    for(int s=0;s<2;++s) {
     double rad=cases[c][s]*3.141592653589793/180.;
     double delay=-(pos[ch][0]*cos(rad)+pos[ch][1]*sin(rad))/343.;
     for(int f=0;f<40;++f) {
      double hz=350+f*137.3+s*19.7;
      x+=sin(6.283185307179586*hz*((frame*1024+i)/48000.-delay)+.31*f*f+1.7*s*f)*400;
     }
    }
    mic_data[ch][i]=(int16_t)lrint(x);
   }
   gcc_phat_estimate_4ch(m,1024,48000,&r);
  }
  if(r.display_peak_count!=2) {
   fprintf(stderr,"two-source peaks missing at %.0f/%.0f\n",cases[c][0],cases[c][1]);return 11;
  }
  for(int source=0;source<2;++source) {
   float best=360;
   for(int j=0;j<r.display_peak_count;++j) {
    float error=circular_error(r.display_peaks[j].angle_deg,(float)cases[c][source]);
    if(error<best)best=error;
    if(r.display_peaks[j].directional_rms<=0) return 12;
   }
   if(best>10) {fprintf(stderr,"wrong secondary direction: %.0f error %.1f\n",cases[c][source],best);return 13;}
  }
 }
 return 0;
}

int main(void)
{
    int rc = run_two_source_cases();
    if (rc) return rc;
    rc = run_fractional_delay_cases();
    if (rc) return rc;
    rc = run_case_set(48000, 1024);
    if (rc != 0) {
        return rc;
    }
    rc = run_case_set(16000, 1024);
    if (rc) return rc;

    const int16_t *mics[4] = {mic_data[0], mic_data[1], mic_data[2], mic_data[3]};
    gcc_phat_result_t result;
    gcc_phat_reset_history();
    gcc_phat_set_score_ema_alpha(.05f);
    for (int i = 0; i < 12; ++i) {
        make_signal(0, 1024, 48000);
        gcc_phat_estimate_4ch(mics, 1024, 48000, &result);
    }
    make_signal(90, 1024, 48000);
    gcc_phat_estimate_4ch(mics, 1024, 48000, &result);
    if (circular_error(result.instant_angle_deg, 90) > 10 ||
        circular_error(result.angle_deg, 0) > 15) {
        fprintf(stderr, "transient inherited old EMA: instant=%.1f ema=%.1f\n",
                result.instant_angle_deg, result.angle_deg);
        return 6;
    }
    gcc_phat_reset_history();
    make_signal(270, 1024, 48000);
    gcc_phat_estimate_4ch(mics, 1024, 48000, &result);
    if (circular_error(result.angle_deg, 270) > 10) return 7;

    /* Noise learning must not absorb a later weak onset. */
    gcc_phat_set_score_ema_alpha(.33f);
    gcc_phat_reset_history();
    gcc_phat_set_noise_learning(true);
    for (int frame = 0; frame < 24; ++frame) {
        for (int ch = 0; ch < 4; ++ch) {
            for (int i = 0; i < 1024; ++i) {
                rng_state = rng_state * 1664525u + 1013904223u;
                mic_data[ch][i] = (int16_t)((int)((rng_state >> 16) & 0xffffu) - 32768) / 100;
            }
        }
        gcc_phat_estimate_4ch(mics, 1024, 48000, &result);
    }
    gcc_phat_set_noise_learning(false);
    gcc_phat_reset_history();
    make_signal(45, 1024, 48000);
    for (int ch = 0; ch < 4; ++ch)
        for (int i = 0; i < 1024; ++i) mic_data[ch][i] /= 20;
    if (!gcc_phat_estimate_4ch(mics, 1024, 48000, &result) ||
        circular_error(result.instant_angle_deg, 45) > 10) {
        fprintf(stderr, "weak coherent onset lost after noise learning\n");
        return 8;
    }
    gcc_phat_set_instant_enabled(false);
    for (int frame = 0; frame < 12; ++frame) {
        make_signal(180, 1024, 48000);
        gcc_phat_estimate_4ch(mics, 1024, 48000, &result);
    }
    if (!result.valid || circular_error(result.angle_deg, 180) > 10) return 9;
    gcc_phat_set_instant_enabled(true);
    return 0;
}
