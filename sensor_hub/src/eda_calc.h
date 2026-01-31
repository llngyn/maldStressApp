#ifndef EDA_CALC_H
#define EDA_CALC_H

#include <zephyr/kernel.h>
#include <math.h>

/* --- TUNING PARAMETERS --- */
/* Filter Alpha: Determines how fast the baseline updates.
 * 0.99 = Slow baseline (good for spotting spikes).
 * 0.50 = Fast baseline (adapts quickly, might hide spikes).
 */
#define EDA_FILTER_ALPHA    0.995f

/* Stress Threshold: How many ADC counts above baseline constitutes "Stress"?
 * Start with 50. Decrease to 20 if wrist signal is weak.
 */
#define EDA_THRESHOLD       30.0f

/* Cooldown: Time (ms) to wait before triggering another event */
#define EDA_COOLDOWN_MS     2000

typedef struct {
    float baseline;         /* Tonic Level (Average) */
    float phasic;           /* Phasic Level (Current - Average) */
    bool  stress_detected;
    int64_t last_event_time;
    bool  is_calibrated;    /* Ignore first few seconds */
    int   startup_samples;
} eda_context_t;

static inline void eda_init(eda_context_t *ctx) {
    ctx->baseline = 0;
    ctx->phasic = 0;
    ctx->stress_detected = false;
    ctx->last_event_time = 0;
    ctx->is_calibrated = false;
    ctx->startup_samples = 0;
}

static inline void eda_process(eda_context_t *ctx, uint16_t raw_val) {
    ctx->stress_detected = false;
    int64_t now = k_uptime_get();

    /* 1. Initialization Phase (First 2 seconds @ 25Hz = 50 samples) */
    if (!ctx->is_calibrated) {
        /* Just set baseline to current value to avoid initial spike */
        if (ctx->baseline == 0) ctx->baseline = (float)raw_val;
        
        /* Fast converge filter during startup */
        ctx->baseline = 0.90f * ctx->baseline + 0.10f * (float)raw_val;
        
        ctx->startup_samples++;
        if (ctx->startup_samples > 50) ctx->is_calibrated = true;
        return;
    }

    /* 2. Update Baseline (Slow Moving Average) */
    /* Logic: NewBaseline = (Old * 0.995) + (Current * 0.005) */
    ctx->baseline = (EDA_FILTER_ALPHA * ctx->baseline) + 
                    ((1.0f - EDA_FILTER_ALPHA) * (float)raw_val);

    /* 3. Calculate Phasic Component (The Spike) */
    /* Since Stress = Higher Value, Phasic = Raw - Baseline */
    ctx->phasic = (float)raw_val - ctx->baseline;

    /* 4. Detect Stress Event */
    if (ctx->phasic > EDA_THRESHOLD && 
       (now - ctx->last_event_time) > EDA_COOLDOWN_MS) {
        
        ctx->stress_detected = true;
        ctx->last_event_time = now;
    }
}

#endif