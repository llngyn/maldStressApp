#ifndef HR_CALC_H
#define HR_CALC_H

#include <zephyr/kernel.h>
#include <math.h>

/* --- TUNING PARAMETERS --- */
#define MIN_BEAT_MS        300  /* 200 BPM max */
#define MA_SIZE            4    /* Signal smoothing window */
#define BPM_BUFFER_SIZE    5    /* Output BPM averaging window */
#define MAX_BPM_JUMP       30   /* Reject beats that jump more than this from average */

/* HRV Stress Parameters */
#define HRV_ALPHA          0.99f /* Slow baseline update (approx 100 beats to shift baseline) */
#define STRESS_DROP_RATIO  0.70f /* Stress = Current HRV is < 70% of Baseline */
#define MIN_RMSSD_VALID    5.0f  /* Ignore noise below 5ms */

typedef struct {
    float dc_filter_ir;
    float peak_threshold;
    int64_t last_beat_time;
    
    /* HRV Buffers */
    int32_t rr_buffer[10];
    uint8_t rr_index;
    
    /* BPM Smoothing */
    int32_t bpm_buffer[BPM_BUFFER_SIZE];
    uint8_t bpm_idx;
    int32_t current_bpm; /* The smoothed output */

    /* Stress Metrics (using HRV) */
    float   current_rmssd;
    float   baseline_rmssd; /* Long-term average HRV */
    bool    stress_from_hrv; /* TRUE if HRV drops significantly */
    
    bool    beat_detected;
    
    /* Signal Processing Buffer */
    uint32_t ma_buf[MA_SIZE];
    uint8_t ma_idx;
} hr_context_t;

static inline void hr_init(hr_context_t *ctx) {
    // for bpm monitoring
    ctx->dc_filter_ir = 0;
    ctx->peak_threshold = 200.0f;
    ctx->last_beat_time = 0;
    ctx->current_bpm = 0;
    ctx->bpm_idx = 0;
    ctx->ma_idx = 0;

    // for hrv monitoring
    ctx->current_rmssd = 0;
    ctx->baseline_rmssd = -1.0f; /* Flag to indicate uninitialized */
    ctx->stress_from_hrv = false;
    
    for(int i=0; i<MA_SIZE; i++) ctx->ma_buf[i] = 0;
    /* Pre-fill BPM buffer with a safe default (e.g., 70) to prevent startup jitters */
    for(int i=0; i<BPM_BUFFER_SIZE; i++) ctx->bpm_buffer[i] = 70;
}

static inline void hr_process(hr_context_t *ctx, uint32_t ir_raw) {
    ctx->beat_detected = false;
    ctx->stress_from_hrv = false; /* Reset status every sample (pulse) */
    
    /* 1. Signal Smoothing (Moving Average) */
    ctx->ma_buf[ctx->ma_idx] = ir_raw;
    ctx->ma_idx = (ctx->ma_idx + 1) % MA_SIZE;
    uint32_t smooth_ir = 0;
    for(int i=0; i<MA_SIZE; i++) smooth_ir += ctx->ma_buf[i];
    smooth_ir /= MA_SIZE;

    /* 2. DC Removal */
    if (ctx->dc_filter_ir == 0) ctx->dc_filter_ir = smooth_ir;
    ctx->dc_filter_ir = 0.95f * ctx->dc_filter_ir + 0.05f * smooth_ir;
    float ac_signal = ctx->dc_filter_ir - smooth_ir;

    /* 3. Beat Logic */
    ctx->peak_threshold *= 0.999f;
    if (ctx->peak_threshold < 50.0f) ctx->peak_threshold = 50.0f;

    int64_t now = k_uptime_get();
    
    if (ac_signal > ctx->peak_threshold && (now - ctx->last_beat_time) > MIN_BEAT_MS) {
        
        int32_t rr_interval = (int32_t)(now - ctx->last_beat_time);
        int32_t instant_bpm = 60000 / rr_interval;
        
        ctx->last_beat_time = now;
        
        /* Update Threshold */
        ctx->peak_threshold = ac_signal * 0.5f;
        if (ctx->peak_threshold < 100.0f) ctx->peak_threshold = 100.0f;

        /* --- 4. BPM FILTERING --- */
        /* only accept realistic HR values */
        if (instant_bpm > 40 && instant_bpm < 200) {
            
            /* Outlier Rejection (Sudden Jumps) */
            bool accept_beat = true;
            if (ctx->current_bpm > 0) {
                int diff = abs(instant_bpm - ctx->current_bpm);
                if (diff > MAX_BPM_JUMP) {
                    /* If it jumps > 30 BPM instantly, ignore it (likely noise or missed beat) */
                    accept_beat = false;
                }
            }

            if (accept_beat) {
                /* Add to smoothing buffer */
                ctx->bpm_buffer[ctx->bpm_idx] = instant_bpm;
                ctx->bpm_idx = (ctx->bpm_idx + 1) % BPM_BUFFER_SIZE;

                /* Calculate Average */
                int32_t sum = 0;
                for(int i=0; i<BPM_BUFFER_SIZE; i++) sum += ctx->bpm_buffer[i];
                ctx->current_bpm = sum / BPM_BUFFER_SIZE;
                
                ctx->beat_detected = true;

                /* HRV Logic (Only update HRV on valid beats) */
                ctx->rr_buffer[ctx->rr_index++ % 10] = rr_interval;
                float sum_sq_diff = 0;
                int count = 0;
                for (int i = 0; i < 9; i++) {
                    if (ctx->rr_buffer[i] > 0 && ctx->rr_buffer[i+1] > 0) {
                        int32_t diff = ctx->rr_buffer[i+1] - ctx->rr_buffer[i];
                        sum_sq_diff += (float)(diff * diff);
                        count++;
                    }
                }
                if (count > 0){
                    ctx->current_rmssd = sqrtf(sum_sq_diff / count);
                    // init HRV baseline
                    if (ctx->baseline_rmssd < 0) {
                        ctx->baseline_rmssd = ctx->current_rmssd;
                    } else {    // update baseline overtime
                        ctx->baseline_rmssd = (HRV_ALPHA * ctx->baseline_rmssd) + 
                                    ((1.0f - HRV_ALPHA) * ctx->current_rmssd);
                    }
                    // CHECK FOR STRESS EVENT -> i.e. drop in HRV
                    if (ctx->current_rmssd > MIN_RMSSD_VALID && 
                        ctx->current_rmssd < (ctx->baseline_rmssd * STRESS_DROP_RATIO)) {
                        ctx->stress_from_hrv = true;
                    }
                } 
            }
        }
    }
}

#endif