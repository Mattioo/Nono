#ifndef MSP_OSD_H
#define MSP_OSD_H

#include <stdint.h>

#define MSP_CMD_API_VERSION       1
#define MSP_CMD_FC_VARIANT        2
#define MSP_CMD_FC_VERSION        3
#define MSP_CMD_NAME              10
#define MSP_CMD_OSD_CONFIG        84
#define MSP_CMD_FILTER_CONFIG     92
#define MSP_CMD_PID_ADVANCED      94
#define MSP_CMD_STATUS            101
#define MSP_CMD_RC                105
#define MSP_CMD_ANALOG            110
#define MSP_CMD_RC_TUNING         111
#define MSP_CMD_PID               112
#define MSP_CMD_BATTERY_STATE     130
#define MSP_CMD_STATUS_EX         150
#define MSP_CMD_DISPLAYPORT       182
#define MSP_CMD_SET_OSD_CANVAS    188

#define MAX_NAME_LENGTH 16u

// MSP_OSD_CONFIG replay
struct msp_osd_config_t {
    uint8_t  osdflags;
    uint8_t  video_system;
    uint8_t  units;
    uint8_t  rssi_alarm;
    uint16_t cap_alarm;
    uint8_t  old_timer_alarm;
    uint8_t  osd_item_count;
    uint16_t alt_alarm;
    uint16_t osd_rssi_value_pos;
    uint16_t osd_main_batt_voltage_pos;
    uint16_t osd_crosshairs_pos;
    uint16_t osd_artificial_horizon_pos;
    uint16_t osd_horizon_sidebars_pos;
    uint16_t osd_item_timer_1_pos;
    uint16_t osd_item_timer_2_pos;
    uint16_t osd_flymode_pos;
    uint16_t osd_craft_name_pos;
    uint16_t osd_throttle_pos_pos;
    uint16_t osd_vtx_channel_pos;
    uint16_t osd_current_draw_pos;
    uint16_t osd_mah_drawn_pos;
    uint16_t osd_gps_speed_pos;
    uint16_t osd_gps_sats_pos;
    uint16_t osd_altitude_pos;
    uint16_t osd_roll_pids_pos;
    uint16_t osd_pitch_pids_pos;
    uint16_t osd_yaw_pids_pos;
    uint16_t osd_power_pos;
    uint16_t osd_pidrate_profile_pos;
    uint16_t osd_warnings_pos;
    uint16_t osd_avg_cell_voltage_pos;
    uint16_t osd_gps_lon_pos;
    uint16_t osd_gps_lat_pos;
    uint16_t osd_debug_pos;
    uint16_t osd_pitch_angle_pos;
    uint16_t osd_roll_angle_pos;
    uint16_t osd_main_batt_usage_pos;
    uint16_t osd_disarmed_pos;
    uint16_t osd_home_dir_pos;
    uint16_t osd_home_dist_pos;
    uint16_t osd_numerical_heading_pos;
    uint16_t osd_numerical_vario_pos;
    uint16_t osd_compass_bar_pos;
    uint16_t osd_esc_tmp_pos;
    uint16_t osd_esc_rpm_pos;
    uint16_t osd_remaining_time_estimate_pos;
    uint16_t osd_rtc_datetime_pos;
    uint16_t osd_adjustment_range_pos;
    uint16_t osd_core_temperature_pos;
    uint16_t osd_anti_gravity_pos;
    uint16_t osd_g_force_pos;
    uint16_t osd_motor_diag_pos;
    uint16_t osd_log_status_pos;
    uint16_t osd_flip_arrow_pos;
    uint16_t osd_link_quality_pos;
    uint16_t osd_flight_dist_pos;
    uint16_t osd_stick_overlay_left_pos;
    uint16_t osd_stick_overlay_right_pos;
    uint16_t osd_display_name_pos;
    uint16_t osd_esc_rpm_freq_pos;
    uint16_t osd_rate_profile_name_pos;
    uint16_t osd_pid_profile_name_pos;
    uint16_t osd_profile_name_pos;
    uint16_t osd_rssi_dbm_value_pos;
    uint16_t osd_rc_channels_pos;
    uint8_t  osd_stat_count;
    uint8_t  osd_stat_rtc_date_time;
    uint8_t  osd_stat_timer_1;
    uint8_t  osd_stat_timer_2;
    uint8_t  osd_stat_max_speed;
    uint8_t  osd_stat_max_distance;
    uint8_t  osd_stat_min_battery;
    uint8_t  osd_stat_end_battery;
    uint8_t  osd_stat_battery;
    uint8_t  osd_stat_min_rssi;
    uint8_t  osd_stat_max_current;
    uint8_t  osd_stat_used_mah;
    uint8_t  osd_stat_max_altitude;
    uint8_t  osd_stat_blackbox;
    uint8_t  osd_stat_blackbox_number;
    uint8_t  osd_stat_max_g_force;
    uint8_t  osd_stat_max_esc_temp;
    uint8_t  osd_stat_max_esc_rpm;
    uint8_t  osd_stat_min_link_quality;
    uint8_t  osd_stat_flight_distance;
    uint8_t  osd_stat_max_fft;
    uint8_t  osd_stat_total_flights;
    uint8_t  osd_stat_total_time;
    uint8_t  osd_stat_total_dist;
    uint8_t  osd_stat_min_rssi_dbm;
    uint16_t osd_timer_count;
    uint16_t osd_timer_1;
    uint16_t osd_timer_2;
    uint16_t enabledwarnings;
    uint8_t  osd_warning_count;
    uint32_t enabledwarnings_1_41_plus;
    uint8_t  osd_profile_count;
    uint8_t  osdprofileindex;
    uint8_t  overlay_radio_mode;
} __attribute__ ((packed));

// MSP_NAME replay
struct msp_name_t {
    char craft_name[MAX_NAME_LENGTH + 1];
} __attribute__ ((packed));

// MSP_FILTER_CONFIG replay
struct msp_filter_config_t {
    uint8_t  gyro_lowpass_hz_u8;        // Gyro_lowpass_hz
    uint16_t dterm_lowpass_hz;          // Delta Filter in hz
    uint16_t yaw_lowpass_hz;            // Additional yaw filter when yaw axis too noisy
    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t dterm_notch_hz;            // Biquad dterm notch hz
    uint16_t dterm_notch_cutoff;        // Biquad dterm notch low cutoff
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
    uint8_t  dterm_filter_type;         // Filter selection for dterm
    uint8_t  gyro_hardware_lpf;         // Gyro DLPF setting
    uint8_t  gyro_32khz_hardware_lpf;   // Gyro 32khz DLPF setting
    uint16_t gyro_lowpass_hz_u16;       // Gyro_lowpass_hz
    uint16_t gyro_lowpass2_hz;
    uint8_t  gyro_lowpass_type;
    uint8_t  gyro_lowpass2_type;
    uint16_t dterm_lowpass2_hz;         // Extra PT1 Filter on D in hz
} __attribute__ ((packed));

// MSP_PID_ADVANCED replay
struct msp_pid_advanced_t {
    uint16_t _1;                     // 0
    uint16_t _2;                     // 0
    uint16_t _3;                     // 0
    uint8_t  _4;                     // 0
    uint8_t  vbatPidCompensation;    // Scale PIDsum to battery voltage
    uint8_t  feedForwardTransition;  // Feed forward weight transition
    uint8_t  _5;                     // 0
    uint8_t  _6;                     // 0
    uint8_t  _7;                     // 0
    uint8_t  _8;                     // 0
    uint16_t rateAccelLimit;         // accel limiter roll/pitch deg/sec/ms
    uint16_t yawRateAccelLimit;      // yaw accel limiter for deg/sec/ms
    uint8_t  levelAngleLimit;        // Max angle in degrees in level mode
    uint8_t  _9;                     // 0
    uint16_t itermThrottleThreshold; // max allowed throttle delta before iterm accelerated in ms
    uint16_t itermAcceleratorGain;   // Iterm Accelerator Gain when itermThrottlethreshold is hit
    uint16_t _10;                    // 0
    uint8_t  iterm_rotation;         // rotates iterm to translate world errors to local coordinate system
    uint8_t  _11;                    // 0
    uint8_t  _12;                    // 0
    uint8_t  _13;                    // 0
    uint8_t  _14;                    // 0
    uint8_t  _15;                    // 0
    uint8_t  _16;                    // 0
    uint16_t pidRoll;
    uint16_t pidPitch;
    uint16_t pidYaw;
    uint8_t  antiGravityMode;        // type of anti gravity method
} __attribute__ ((packed));


// MSP_BATTERY_STATE replay
enum batteryState_e {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT,
    BATTERY_INIT
};

struct msp_battery_state_t {
    uint8_t  batteryCellCount; // 0 - 255
    uint16_t batteryCapacity;
    uint8_t  batteryVoltage;   // 0 - 255
    uint16_t mAhDrawn;         // 0 - 0xFFFF
    uint16_t amperage;         // -0x8000 - 0x7FFF
    uint8_t  batteryState;     // batteryState_e
} __attribute__ ((packed));

#endif // MSP_OSD_H