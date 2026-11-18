#include "led_effects.h"
#include "doa_led_scene.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static unsigned mask(const led_effect_rgb_t *p)
{
    unsigned out = 0;
    for (int i=0;i<DIRECTION_LED_COUNT;++i) if (p[i].r || p[i].g || p[i].b) out |= 1U<<i;
    return out;
}
static unsigned intensity(const led_effect_rgb_t *p)
{
    unsigned out=0;
    for(int i=0;i<DIRECTION_LED_COUNT;++i) out+=p[i].r+p[i].g+p[i].b;
    return out;
}
static int head(const led_effect_rgb_t *p)
{
    int best=0;
    for(int i=1;i<DIRECTION_LED_COUNT;++i)
        if((unsigned)p[i].r+p[i].g+p[i].b>(unsigned)p[best].r+p[best].g+p[best].b) best=i;
    return best;
}
static bool connected_dim_tail(const led_effect_rgb_t *p)
{
    int h=head(p);unsigned main=p[h].r+p[h].g+p[h].b;int tails=0;
    for(int i=0;i<DIRECTION_LED_COUNT;++i) if(i!=h && (p[i].r||p[i].g||p[i].b)) {
        float d=fabsf(direction_led_angle_of_index(i)-direction_led_angle_of_index(h));
        if(d>180)d=360-d;
        if(d>22.6f || (unsigned)p[i].r+p[i].g+p[i].b>main*.15f+3.f) return false;
        tails++;
    }
    return main>0 && tails<=1;
}
#define CHECK(condition, message) do { if (!(condition)) {fprintf(stderr,"%s (line %d)\n",message,__LINE__);return 1;} } while(0)

int main(void)
{
    led_effects_t effect;
    led_effect_rgb_t p[DIRECTION_LED_COUNT];
    led_effect_scene_t scene={.count=1,.sources={{0,.4f}}};
    led_effects_init(&effect);
    led_effects_render(&effect,&scene,0,p);
    CHECK(mask(p)==1,"stationary source must light one exact sector");
    unsigned first=intensity(p);
    led_effects_render(&effect,&scene,LED_EFFECT_BREATH_PERIOD_MS/2,p);
    CHECK(mask(p)==1 && intensity(p)>0,"breath must never extinguish a valid stationary source");
#if LED_EFFECT_MODE == LED_EFFECT_MODE_BREATHE
    CHECK(intensity(p)<first,"breathing phase did not change brightness");
#endif
    CHECK(p[0].b>p[0].r,"weak source should be blue/cyan");
    float weak=led_effect_level_from_rms(.002f,.001f),strong=led_effect_level_from_rms(.03f,.001f);
    CHECK(strong>weak,"RMS strength mapping is not monotonic");
    CHECK(led_effect_level_from_rms(.001f,.002f)==0,"background subtraction must not emit NaN/negative level");
    scene.sources[0].level=1;
    for(uint32_t t=1000;t<1800;t+=20) led_effects_render(&effect,&scene,t,p);
    CHECK(p[0].r>0 && p[0].g>=p[0].b,"loud source should warm toward yellow/green");

    /* Move across 0/360: at most one head plus its immediate trailing neighbor. */
    scene.sources[0].angle_deg=337.5f;
    led_effects_render(&effect,&scene,1800,p);
    CHECK((mask(p)&~3U)==0 && mask(p)!=0,"moving trail took the wrong ring path");
    CHECK(mask(p)==1,"single neighboring candidate should keep the original lamp");
    bool saw_tail=false;
    for(uint32_t t=1820;t<2500;t+=20) {
        led_effects_render(&effect,&scene,t,p);
        CHECK(connected_dim_tail(p),"neighboring handoff lit scattered or over-bright tail pixels");
        if(mask(p)==3) {
            saw_tail=true;
            unsigned head=p[1].r+p[1].g+p[1].b,tail=p[0].r+p[0].g+p[0].b;
            CHECK(head>tail,"trail must be dimmer than the head");
        }
    }
    CHECK(saw_tail,"neighboring move did not create a short adjacent trail");
    CHECK(head(p)==1,"moving source did not settle exactly on target");
    for(uint32_t t=2500;t<2760;t+=20) led_effects_render(&effect,&scene,t,p);
    CHECK(mask(p)==2,"stationary target did not clear its tail");
    scene.sources[0].angle_deg=157.5f;
    led_effects_render(&effect,&scene,2800,p);
    CHECK(mask(p)==2,"one-frame distant glitch must not immediately flash another lamp");
    for(uint32_t t=2820;t<3260;t+=20) {
        led_effects_render(&effect,&scene,t,p);
        CHECK(mask(p)==2 || mask(p)==(1U<<9),"large handoff left a remote trail or intermediate pixels");
    }
    CHECK(mask(p)==(1U<<9),"persistent large change failed to settle");

    /* Reproduce repeated adjacent A-B-A measurement jitter. Brightness may
     * breathe, but the displayed direction must not switch for ten seconds. */
    led_effects_init(&effect);
    scene=(led_effect_scene_t){.count=1,.sources={{0,.5f}}};
    led_effects_render(&effect,&scene,0,p);
    for(uint32_t t=20;t<10000;t+=20) {
        scene.sources[0].angle_deg=(t/100U)%2U ? 22.5f : 0;
        led_effects_render(&effect,&scene,t,p);
        CHECK(mask(p)==1,"adjacent jitter made the lamp chase A-B-A measurements");
    }
    /* A real neighboring move persists and eventually wins; it is not locked
     * forever by the anti-jitter policy. */
    scene.sources[0].angle_deg=22.5f;
    for(uint32_t t=10000;t<10800;t+=20) led_effects_render(&effect,&scene,t,p);
    CHECK(mask(p)==(1U<<15),"persistent neighboring movement was frozen");

    /* Full clockwise/counterclockwise circles at three speeds. The brightest
     * pixel must visit ALL 16 sectors in order, including 359->0, without
     * waiting hundreds of milliseconds at each target. Input is 50 Hz. */
    for(int sign=-1;sign<=1;sign+=2) for(int turn_ms=2000;turn_ms<=8000;turn_ms*=2) {
        led_effects_init(&effect);
        scene=(led_effect_scene_t){.count=1,.sources={{0,.8f}}};
        led_effects_render(&effect,&scene,0,p);
        unsigned visited=1;int previous=0;
        for(uint32_t t=20;t<=(uint32_t)turn_ms+400;t+=20) {
            float angle=sign*360.f*fminf((float)t/turn_ms,1.f);
            angle=fmodf(angle+720.f,360.f);
            scene.sources[0].angle_deg=angle;
            led_effects_render(&effect,&scene,t,p);
            CHECK(connected_dim_tail(p),"360 degree motion created detached/multiple/bright tails");
            int h=head(p);visited|=1U<<h;
            if(h!=previous) {
                int expected=(previous+(sign>0 ? 15 : 1))%16;
                CHECK(h==expected,"full-circle motion skipped or reversed a sector");
                previous=h;
            }
            float error=fabsf(angle-direction_led_angle_of_index(h));if(error>180)error=360-error;
            CHECK(error<20.f,"brightest head is lagging more than one sector behind the source");
        }
        CHECK(visited==0xffff,"rotating source did not visit the whole ring");
        CHECK(head(p)==0 && mask(p)==1,"one full turn failed to settle back at zero");
    }

    /* Two actual directions can carry different levels simultaneously. */
    led_effects_init(&effect);
    scene=(led_effect_scene_t){.count=2,.sources={{0,1.f},{180,.1f}}};
    led_effects_render(&effect,&scene,0,p);
    CHECK(mask(p)==((1U<<0)|(1U<<8)),"two sources not visible together");
    CHECK((unsigned)p[0].r+p[0].g+p[0].b>(unsigned)p[8].r+p[8].g+p[8].b,"per-direction strength was lost");
    led_effect_source_t swap=scene.sources[0];scene.sources[0]=scene.sources[1];scene.sources[1]=swap;
    led_effects_render(&effect,&scene,20,p);
    CHECK(mask(p)==((1U<<0)|(1U<<8)),"rank swap moved source tracks");
    scene.count=0;
    for(uint32_t t=40;t<3000;t+=20) led_effects_render(&effect,&scene,t,p);
    CHECK(mask(p)==0,"all sources must eventually fade completely off");
    scene.count=1;scene.sources[0].angle_deg=NAN;
    led_effects_render(&effect,&scene,3020,p);
    CHECK(mask(p)==0,"non-finite input was rendered");

    /* Multipeak confirmation uses duration AND observation count. */
    doa_led_scene_t tracker;
    doa_led_scene_init(&tracker);
    gcc_phat_result_t spatial={.display_peak_count=2};
    spatial.display_peaks[0]=(gcc_phat_peak_t){.angle_deg=0,.directional_rms=.01f};
    spatial.display_peaks[1]=(gcc_phat_peak_t){.angle_deg=100,.directional_rms=.005f};
    for(uint32_t t=0;t<500;t+=20) {
        doa_led_scene_update(&tracker,&spatial,true,true,180,.01f,.001f,t,&scene);
        CHECK(scene.count==1,"secondary source appeared before persistent confirmation");
    }
    doa_led_scene_update(&tracker,&spatial,true,true,180,.01f,.001f,500,&scene);
    CHECK(scene.count==2,"persistent independent peaks did not produce two sources");
    spatial.display_peak_count=0;
    doa_led_scene_update(&tracker,&spatial,true,true,180,.01f,.001f,540,&scene);
    CHECK(scene.count==2,"brief missing peak should hold stable pair");
    doa_led_scene_update(&tracker,&spatial,true,true,180,.01f,.001f,660,&scene);
    CHECK(scene.count==1,"vanished secondary remained indefinitely");
    doa_led_scene_update(&tracker,&spatial,false,false,180,0,.001f,680,&scene);
    CHECK(scene.count==0,"silence left a phantom source");
    doa_led_scene_init(&tracker);
    for(uint32_t t=0;t<2000;t+=20) {
        spatial.display_peak_count=t%100==0 ? 2 : 0;
        doa_led_scene_update(&tracker,&spatial,true,true,180,.01f,.001f,t,&scene);
        CHECK(scene.count==1,"intermittent secondary flashes were accumulated across missing frames");
    }

    /* Returning from a genuine pair to one source must not leave the removed
     * distant source fading while the remaining head is rendered. */
    led_effects_init(&effect);
    scene=(led_effect_scene_t){.count=2,.sources={{0,.7f},{180,.7f}}};
    led_effects_render(&effect,&scene,0,p);
    CHECK(mask(p)==((1U<<0)|(1U<<8)),"initial confirmed pair missing");
    scene.count=1;scene.sources[0]=(led_effect_source_t){180,.7f};
    led_effects_render(&effect,&scene,20,p);
    CHECK(mask(p)==(1U<<8),"removed source left a scattered ghost pixel");

    /* One-frame far glitches at arbitrary sectors cannot flash briefly even
     * when the display has already been resident for many seconds. */
    led_effects_init(&effect);
    scene=(led_effect_scene_t){.count=1,.sources={{0,.7f}}};
    led_effects_render(&effect,&scene,0,p);
    for(uint32_t t=20;t<10000;t+=20) {
        scene.sources[0].angle_deg=t%400==0 ? 180 : 0;
        led_effects_render(&effect,&scene,t,p);
        CHECK(mask(p)==1,"single-frame distant outlier caused a flash");
    }
    return 0;
}
