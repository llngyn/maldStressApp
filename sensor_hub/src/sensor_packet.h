#ifndef SENSOR_PACKET_H
#define SENSOR_PACKET_H

#include <stdint.h>

/* --- BLE DATA STRUCTURE --- */
/* Packed struct to ensure consistent byte alignment for transmission over BLE.
 * Total Size: 18 Bytes
 */
typedef struct __attribute__((packed)) {
    uint32_t timestamp;    /* 4 Bytes: Time in ms */
    uint16_t hrv;          /* 2 Bytes: RMSSD in ms */
    uint16_t eda_raw;      /* 2 Bytes: Raw ADC Value */
    uint8_t  heart_rate;   /* 1 Byte:  BPM */
    int16_t  temperature;  /* 2 Bytes: Temp in C * 100 (e.g., 3650 = 36.50C) */
    int16_t  accel_x;     /* 2 Bytes: Delta X (m/s^2 * 100) */
    int16_t  accel_y;     /* 2 Bytes: Delta Y (m/s^2 * 100) */
    int16_t  accel_z;     /* 2 Bytes: Delta Z (m/s^2 * 100) */
    uint8_t  status_flags; /* 1 Byte:  Bitmask for status booleans */
} sensor_packet_t;

/* Status Flag Bit Definitions */
#define STATUS_BIT_MOTION      (1 << 0)
#define STATUS_BIT_INDOOR      (1 << 1)
#define STATUS_BIT_STRESS_EDA  (1 << 2)
#define STATUS_BIT_STRESS_HRV  (1 << 3)

#endif /* SENSOR_PACKET_H */