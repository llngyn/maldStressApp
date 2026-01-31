#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
//#include <zephyr/drivers/led.h> // for PMIC LEDs
#include <zephyr/logging/log.h>
#include <zephyr/drivers/spi.h>
#include <math.h>
#include <stdio.h>

#include <zephyr/drivers/sensor/adxl345.h>
#include <zephyr/drivers/sensor/veml7700.h>
#include <zephyr/drivers/sensor/max30101.h>

#include "hr_calc.h"
#include "eda_calc.h"
#include "sensor_packet.h"

#define MOTION_THRESHOLD 1.0 
#define PPG_ADDR 0x57
#define PPG_DATA_DUMP 0

LOG_MODULE_REGISTER(SensorHub, LOG_LEVEL_INF);

// get device labels
const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c22));              // i2c bus for sensors
const struct device *acc_dev = DEVICE_DT_GET(DT_NODELABEL(adxl345));            // accelerometer
const struct device *light_dev = DEVICE_DT_GET(DT_NODELABEL(veml7700));         // ambient light sensor
const struct device *ppg_dev = DEVICE_DT_GET(DT_NODELABEL(max30102));           // ppg sensor
// for pmic -> uncomment for custom pcb
// const struct device *pmic_leds = DEVICE_DT_GET(DT_NODELABEL(npm1304_leds));     // pmic LEDs
// const struct device *charger = DEVICE_DT_GET(DT_NODELABEL(npm1304_charger));    // pmic battery charger

// init EDA sensor SPI device label (looks a little diff)
const struct spi_dt_spec eda_dev = SPI_DT_SPEC_GET(DT_NODELABEL(mcp3201), SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0);

// define HR and EDA context structs
hr_context_t hr_ctx;
eda_context_t eda_ctx;

// *****HELPER FUNCTIONS PROTOTYPES*****
void scan_i2c_bus(const struct device *dev);
int configure_acc(void);
int configure_light(void);
int configure_ppg(void);
// PMIC functions, uncomment later
// int32_t read_battery(void); 
// void update_pmic_status(void);

int main(void)
{
        // power up delay for device
        k_msleep(1000);
        LOG_INF("*** DEVICE SENSING STARTING ***");

        // check if I2C bus is ready
        if (!device_is_ready(i2c_bus)){
                LOG_ERR("I2C bus not ready");
                return 0;
        } else {
                // scan for devices on I2C bus
                scan_i2c_bus(i2c_bus);
        }

        // EDA SENSOR CONFIG
        // check if SPI Bus/EDA sensor device is ready
        if (!spi_is_ready_dt(&eda_dev)) {
                LOG_ERR("SPI bus not ready.");
                return 0;
        }
        LOG_INF("SPI Bus Ready. Reading GSR Sensor...");
        // init eda context struct
        eda_init(&eda_ctx);
        // eda/spi data buffer vars
        uint8_t eda_rx_buffer[2] = {0, 0};
        // config spi buffer and definitions
        const struct spi_buf rx_buf = {.buf = eda_rx_buffer, .len = sizeof(eda_rx_buffer)};
        const struct spi_buf_set rx_set = {.buffers = &rx_buf, .count = 1};
        // eda data values
        uint16_t raw_val;
        uint16_t eda_adc_val=-1;
        double eda_volt_val;

        // ADXL345 CONFIGURATION
        int config_err = configure_acc();
        if (config_err) return 0;
        // accelerometer data & state
        struct sensor_value accel[3];   // accelerometer data
        double curr_x=0, curr_y=0, curr_z=0;
        double prev_x = 0, prev_y = 0, prev_z = 0;
        double delta_x=0, delta_y=0, delta_z=0;
        double magnitude;
        bool first_accel_sample = true;

        // VEML7700 CONFIGURATION
        config_err = configure_light();
        if (config_err) return 0;
        // ambient light sensor data var
        struct sensor_value lux;

        // MAX30102 CONFIGURATION
        // init hr_context var and configure device
        hr_init(&hr_ctx);
        config_err = configure_ppg();
        if (config_err) return 0;
        // ppg fifo data variables
        uint8_t ptrs[2] = {0};
        uint8_t reg_wr = MAX30101_REG_FIFO_WR;
        uint8_t reg_rd = MAX30101_REG_FIFO_RD;
        uint8_t reg_data = MAX30101_REG_FIFO_DATA;
        int num_samples;
        uint8_t fifo_data[6]; // 3 bytes for RED, 3 bytes for IR LED
        uint32_t ir_val=-1;
        float current_temp_c = 0.0f;

        // // set battery status LEDs
        // if (!device_is_ready(pmic_leds)) {
        //         LOG_ERR("PMIC LEDs not ready");
        //         return 0;
        // }
        // update_pmic_status();

        // init sensor status vars
        sensor_packet_t current_packet = {0}; // BLE data packet
        int loop = 0; // keep track of cycles to sample devices at dedicated samp freq
        #if PPG_DATA_DUMP 
        int csv_sample_counter = 0;     /* Counts samples to handle decimation */
        #endif
        bool indoors = true; // use for light sensor status
        bool in_motion = false;
        int32_t curr_bpm=-1;

        // read sensor data in main loop
        while(1) {
                loop++; // increment loop var
        
                // Motion Detection
                // fetch x, y, z data from accelerometer
                if (sensor_sample_fetch(acc_dev) == 0) {
                        sensor_channel_get(acc_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
                        
                        // fetch current x, y, z data readings
                        curr_x = sensor_value_to_double(&accel[0]);
                        curr_y = sensor_value_to_double(&accel[1]);
                        curr_z = sensor_value_to_double(&accel[2]);

                        // check for motion
                        if (first_accel_sample) {
                                prev_x = curr_x;
                                prev_y = curr_y;
                                prev_z = curr_z;
                                first_accel_sample = false;
                        } else {
                                // calculate change from prev sample
                                delta_x = curr_x - prev_x;
                                delta_y = curr_y - prev_y;
                                delta_z = curr_z - prev_z;

                                // update prev samples
                                prev_x = curr_x;
                                prev_y = curr_y;
                                prev_z = curr_z;

                                // calculate magnitude of change
                                magnitude = sqrt(pow(delta_x, 2) + pow(delta_y, 2) + pow(delta_z, 2));

                                // check if motion exceeds threshold
                                if (magnitude > MOTION_THRESHOLD) {
                                        in_motion = true;
                                        // LOG_INF("Movement Detected with Magnitude: %.2f m/s^2", magnitude);
                                } else {
                                        in_motion = false;
                                        // uncomment if want to see still w no movement detected
                                        // LOG_INF("Still. Magnitude: %.2f", magnitude);
                                }
                        }
                        // uncomment to print out full sensor data
                        // LOG_INF("Accel: X=%.2f, Y=%.2f, Z=%.2f",
                        //         sensor_value_to_double(&accel[0]),
                        //         sensor_value_to_double(&accel[1]),
                        //         sensor_value_to_double(&accel[2]));
                } else {
                        LOG_ERR("Failed to fetch accel data");
                }

                // sampling at 1 Hz (25 Hz/25=1Hz)
                if (loop%25==0){ 
                        // INDOOR/OUTDOOR DETECTION
                        // fetch sample from veml7700 device
                        if (sensor_sample_fetch(light_dev) < 0) {
                                LOG_ERR("VEML7700 sample fetch failed");
                                continue;
                        }
                        // read the light channel
                        if (sensor_channel_get(light_dev, SENSOR_CHAN_LIGHT, &lux) < 0) {
                                LOG_ERR("Cannot read veml7700 light channel");
                                continue;
                        }
                        // // uncomment to print full light sensor data
                        // LOG_INF("Ambient Light: %d.%06d Lux", lux.val1, lux.val2);
                        // differentiate between indoor vs. outdoor
                        if (lux.val1 > 2000) {  // > 2000 lux == outdoor
                                indoors = false;
                                // LOG_INF("Status: Outdoors");
                        } else if (lux.val1 < 500) {        // < 500 lux == indoor
                                indoors = true;
                                // LOG_INF("Status: Indoors"); 
                        }

                        // read temperature from MAX30102
                        uint8_t temp_req[2] = {MAX30101_REG_TEMP_CFG, 0x01}; // trigger request for temp reading 
                        if (i2c_write(i2c_bus, temp_req, 2, PPG_ADDR) == 0) {
                                // since temp conversion take 30 ms, will read prev. temp value
                                uint8_t temp_int = 0;
                                uint8_t temp_frac = 0;
                                uint8_t reg_temp = MAX30101_REG_TINT;
                                
                                i2c_write_read(i2c_bus, PPG_ADDR, &reg_temp, 1, &temp_int, 1);
                                reg_temp = MAX30101_REG_TFRAC;
                                i2c_write_read(i2c_bus, PPG_ADDR, &reg_temp, 1, &temp_frac, 1);
                                
                                /* Combine Integer and Fraction (0.0625 degrees per LSB) */
                                current_temp_c = (float)temp_int + ((float)temp_frac * 0.0625f);
                        }
                }

                // READ PPG DATA
                // read FIFO for data
                i2c_write_read(i2c_bus, PPG_ADDR, &reg_wr, 1, &ptrs[0], 1); 
                i2c_write_read(i2c_bus, PPG_ADDR, &reg_rd, 1, &ptrs[1], 1); 
                // calculate # of samples available
                num_samples = ptrs[0] - ptrs[1];
                if (num_samples < 0) num_samples += 32;
                if (num_samples > 0) {
                        // read data from each sample
                        for (int i = 0; i < num_samples; i++) {
                                if (i2c_write_read(i2c_bus, PPG_ADDR, &reg_data, 1, fifo_data, 6) == 0) {
                                        // combine IR data into one val (bytes 3, 4, 5)
                                        ir_val = ((uint32_t)fifo_data[3] << 16) | 
                                                        ((uint32_t)fifo_data[4] << 8)  | 
                                                        (uint32_t)fifo_data[5];
                                        ir_val &= 0x3FFFF; // mask to 18 bits
                                        // process data with hr algorithm
                                        hr_process(&hr_ctx, ir_val); // LOG EVERY 100ms
                                        // log HR if heart beat detected
                                        if (hr_ctx.beat_detected) {
                                                curr_bpm = hr_ctx.current_bpm;
                                                /* * Decimation Logic:
                                                * Sensor runs at 400Hz. User wants 100ms intervals (10Hz).
                                                * We need to print every 40th sample (400 / 10 = 40).
                                                */
                                                #if !PPG_DATA_DUMP
                                                        // log stress event (using HRV data)
                                                        if (hr_ctx.stress_from_hrv){
                                                                LOG_INF("Stress Event from PPG Sensor Detected! HRV: %.2f ms", (double)hr_ctx.current_rmssd);
                                                        }
                                                        // uncomment to print out HR
                                                        // LOG_INF("BPM: %d | HRV: %.2f ms", hr_ctx.current_bpm, (double)hr_ctx.current_rmssd);
                                        
                                                #endif
                                        }
                                        // uncomment for raw data stream -> may be useful for plotting??
                                        /* LOG_INF("IR: %d", ir_val); */
                                }
                        }
                }

                // MEASURE EDA DATA
                // since MCP3201 doesn't need a command byte, we just read 2 bytes
                int err = spi_read_dt(&eda_dev, &rx_set);
                if (err) {
                        LOG_ERR("SPI Read Failed (Err: %d)", err);
                } else {
                        /* decode MCP3201 data (12-bit)
                        * The MCP3201 returns 2 bytes.
                        * Byte 0: [0 0 0 Null B11 B10 B9 B8]
                        * Byte 1: [B7 B6 B5 B4 B3 B2 B1 B0]
                        * * Logic: 
                        * 1. Mask Byte 0 to remove top 3 bits + Null bit (0x1F)
                        * 2. Shift Byte 0 left by 7 (or 8 depending on exact timing alignment)
                        */
                        // combine to 16-bit integer for easier processing
                        raw_val = ((uint16_t)eda_rx_buffer[0] << 8) | eda_rx_buffer[1];
                        
                        // format raw data to print:
                        /* MCP3201 formatting often requires shifting right by 1 bit 
                        * to remove trailing zeros depending on clock edge. 
                        * Standard formula: (Buffer[0] & 0x1F) << 7 | (Buffer[1] >> 1)
                        */
                        eda_adc_val = ((eda_rx_buffer[0] & 0x1F) << 7) | (eda_rx_buffer[1] >> 1);
                        eda_volt_val = (double)(eda_adc_val * 3.3 / 4096.0);

                        // process eda data
                        eda_process(&eda_ctx, eda_adc_val);
                        #if !PPG_DATA_DUMP
                        // detect stress event
                        if (eda_ctx.stress_detected) {
                                LOG_INF("Stress Event from EDA Sensor Detected! Raw: %d (Baseline: %.0f)", 
                                        eda_adc_val, (double)eda_ctx.baseline);
                        }
                        #endif

                        // uncomment to print at raw eda data
                        // LOG_INF("GSR Raw: %d (V: %.2f)", eda_adc_val, (double)(eda_adc_val * 3.3 / 4096.0));
                }

                // update BLE packet with most recent data
                current_packet.timestamp = k_uptime_get_32();   // update timestamp
                // update data values for ppg and eda
                current_packet.heart_rate = (uint8_t)hr_ctx.current_bpm;
                current_packet.hrv = (uint16_t)hr_ctx.current_rmssd;
                current_packet.eda_raw = eda_adc_val;
                current_packet.temperature = (int16_t)(current_temp_c * 100); // temp scaled x100
                // NOTE: accelerometer data values are scaled by 100 -> 1.25ms^2 becomes 125
                current_packet.accel_x = (int16_t)(curr_x * 100);
                current_packet.accel_y = (int16_t)(curr_y * 100);
                current_packet.accel_z = (int16_t)(curr_z * 100);
                // update status flags for status variables
                current_packet.status_flags = 0;
                if (in_motion)       current_packet.status_flags |= STATUS_BIT_MOTION;
                if (indoors)      current_packet.status_flags |= STATUS_BIT_INDOOR;
                if (eda_ctx.stress_detected) current_packet.status_flags |= STATUS_BIT_STRESS_EDA;
                if (hr_ctx.stress_from_hrv)  current_packet.status_flags |= STATUS_BIT_STRESS_HRV;
                // *** place bluetooth call her later ***

                #if !PPG_DATA_DUMP
                // print sensor status data every 4 sec
                if (loop==75){
                        LOG_INF("\n**PRINTING DEVICE READINGS**");
                        LOG_INF("Current HR: %d bpm", curr_bpm);
                        LOG_INF("Temperature: %.2f C", (double)current_temp_c);
                        if (in_motion){
                                LOG_INF("Motion Detected!");
                        } else {
                                LOG_INF("Still -> No Motion Detected.");
                        }
                        if (indoors){
                                LOG_INF("Currently Indoors.");
                        } else {
                                LOG_INF("Currently Outdoors.");
                        }
                        loop=0;
                        LOG_INF("\nCollecting New Device Data...");
                }
                #else
                csv_sample_counter++;
                if (csv_sample_counter >= 5) {
                        /* Format: Timestamp(ms), Raw_Value, BPM */
                        printk("%lld,%d,%d,%f,%d,%f,%f,%f,%f,%d,%d,%d,%d\n", k_uptime_get(),ir_val,hr_ctx.current_bpm,(double)hr_ctx.current_rmssd,
                                eda_adc_val,(double)current_temp_c,curr_x,curr_y,curr_z,in_motion,indoors,eda_ctx.stress_detected,hr_ctx.stress_from_hrv);
                        csv_sample_counter = 0;
                }
                #endif

                k_msleep(40); // sleep for 40 ms (25 Hz sampling rate)
        }
        return 0;
}


// *****HELPER FUNCTIONS*****
// i2c scanner function
void scan_i2c_bus(const struct device *dev)
{
    LOG_INF("Starting I2C Scan on %s...", dev->name);
    // uint8_t error = 0u;
    bool found = false;

    for (uint8_t i = 1; i <= 127; i++) {
        struct i2c_msg msgs[1];
        uint8_t dst;

        /* Send a zero-length write to probe the address */
        msgs[0].buf = &dst;
        msgs[0].len = 0U;
        msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;

        if (i2c_transfer(dev, &msgs[0], 1, i) == 0) {
            LOG_INF("--> Found device at 0x%02x", i);
            found = true;
        }
    }

    if (!found) {
        LOG_ERR("No devices found on I2C bus! Check SDA/SCL wiring.");
    } else {
        LOG_INF("I2C Scan Complete.");
    }
}

// configue accelerometer helper
int configure_acc(void){
        // check if adxl345 device is ready
        if (!device_is_ready(acc_dev)) {
                LOG_ERR("Accelerometer sensor device not ready. Check sensor wiring and reset device.");
                return -1;
        } else {
                // configure device for +/- 2g range
                struct sensor_value range;
                sensor_value_from_double(&range, 2.0); // +/- 2g
                sensor_attr_set(acc_dev, SENSOR_CHAN_ACCEL_XYZ, 
                                SENSOR_ATTR_FULL_SCALE, &range);
                // configure device for +/- 25 Hz sampling rate
                struct sensor_value odr;
                sensor_value_from_double(&odr, 25.0); // 25 Hz
                sensor_attr_set(acc_dev, SENSOR_CHAN_ACCEL_XYZ, 
                                SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
                LOG_INF("ADXL345 Device Ready");
                return 0;
        }
}

// configue ambient light sensor helper
int configure_light(void) {
        // check if veml7700 device is ready
        if (!device_is_ready(light_dev)) {
                LOG_ERR("Light sensor device not ready. Check sensor wiring and reset device.");
                return -1;
        } else {
                // set gain value to 1/8 gain
                struct sensor_value gain_val;
                gain_val.val1 = VEML7700_ALS_GAIN_1_8;
                gain_val.val2 = 0; // fraction to 0
                int ret = sensor_attr_set(light_dev, SENSOR_CHAN_LIGHT, 
                                        (enum sensor_attribute)SENSOR_ATTR_VEML7700_GAIN,
                                        &gain_val);
                if (ret != 0) {
                        LOG_ERR("Failed to set VEMl7700 gain");
                        return -1;
                }
                // set integration time to 100ms
                struct sensor_value it_val;
                it_val.val1 = VEML7700_ALS_IT_100;
                it_val.val2 = 0;
                ret = sensor_attr_set(light_dev, SENSOR_CHAN_LIGHT, 
                                (enum sensor_attribute)SENSOR_ATTR_VEML7700_ITIME, 
                                &it_val);
                if (ret != 0) {
                        LOG_ERR("Failed to set VEML7700 integration time");
                        return -1;
                }
                LOG_INF("VEML7700 Device Ready");
                return 0;
        }
}

// configure PPG sensor helper
int configure_ppg(void) {
        // check if max30102 device is ready
        if (!device_is_ready(ppg_dev)){
                LOG_ERR("PPG sensor not ready. Check sensor wiring and reset device.");
                return -1;
        } else {
                uint8_t config[2];
                // reset ppg sensor
                config[0] = MAX30101_REG_MODE_CFG; config[1] = 0x40;
                i2c_write(i2c_bus, config, 2, PPG_ADDR);
                k_msleep(100);
        
                // configure fifo and enable rollover
                config[0] = MAX30101_REG_FIFO_CFG; config[1] = 0x10;
                i2c_write(i2c_bus, config, 2, PPG_ADDR);
        
                // configure spO2 mode (SpO2 = Red + IR) */
                config[0] = MAX30101_REG_MODE_CFG; config[1] = 0x03;
                i2c_write(i2c_bus, config, 2, PPG_ADDR);
        
                /* Sp02 configuration
                * Range: 4096nA (01)
                * Rate:  400 Hz  (011) <-- High speed for HRV
                * Pulse: 411 us  (11)
                * Binary: 0010 1111 -> 0x2F
                */
                config[0] = MAX30101_REG_SPO2_CFG; config[1] = 0x2F;
                i2c_write(i2c_bus, config, 2, PPG_ADDR);
        
                // configue LED current at ~7mA (0x24)
                config[0] = MAX30101_REG_LED1_PA; config[1] = 0x24; /* RED */
                i2c_write(i2c_bus, config, 2, PPG_ADDR);
                config[0] = MAX30101_REG_LED2_PA; config[1] = 0x24; /* IR */
                i2c_write(i2c_bus, config, 2, PPG_ADDR);
        
                // set FIFO pointers to 0
                config[0] = MAX30101_REG_FIFO_WR; config[1] = 0x00;
                i2c_write(i2c_bus, config, 2, PPG_ADDR);
                config[0] = MAX30101_REG_FIFO_RD; config[1] = 0x00;
                i2c_write(i2c_bus, config, 2, PPG_ADDR);

                LOG_INF("MAX30102 Device Ready");
                return 0;
        }  
}

// // reads battery status and return percentage
// int32_t read_battery(void) {
//         struct sensor_value volt, soc, current;
//         // fetch sample data from chargers
//         if (sensor_sample_fetch(charger) < 0) {
//             LOG_ERR("Failed to fetch battery data");
//             return 0;
//         }
//         // read battery voltage level
//         sensor_channel_get(charger, SENSOR_CHAN_GAUGE_VOLTAGE, &volt);
//         // battery percentage
//         sensor_channel_get(charger, SENSOR_CHAN_GAUGE_STATE_OF_CHARGE, &soc);
//         // get average charging current value
//         sensor_channel_get(charger, SENSOR_CHAN_GAUGE_AVG_CURRENT, &current);
    
//         // Uncomment to print out battery status
//         // LOG_INF("Battery: %d%% (%d.%02d V) | Current: %d.%02d mA", 
//         //         soc.val1, 
//         //         volt.val1, volt.val2/10000, 
//         //         current.val1, current.val2/10000);

//         return soc.val1; // return battery percentage (what we will use for status LED)
// }

// // update battery status leds
// void update_pmic_status(void) {
//         if (device_is_ready(pmic_leds)) {
//             /* 1. Turn ON Green LED (System Active) */
//             led_on(pmic_leds, 1); /* Index 1 = Green in your overlay */
    
//             /* 2. Logic for Red LED (Low Battery) */
//             if (read_battery() < 10) led_on(pmic_leds, 2); else led_off(pmic_leds, 2);
//         }
// }
