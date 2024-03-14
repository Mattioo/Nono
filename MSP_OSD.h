#pragma once

#define MSP_CMD_OSD_CONFIG 84
#define MSP_CMD_FILTER_CONFIG 92
#define MSP_CMD_PID_ADVANCED 94
#define MSP_CMD_STATUS_EX 150
#define MSP_CMD_RTC 247

#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_CMD_OSD_CONFIG
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct msp_cmd_osd_config_t {
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_NAME
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define MAX_NAME_LENGTH 16u

struct msp_cmd_name_t {
    char craft_name[MAX_NAME_LENGTH + 1];
} __attribute__ ((packed));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_CMD_FILTER_CONFIG
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define GYRO_LPF1_DYN_MIN_HZ_DEFAULT 250
#define GYRO_LPF2_HZ_DEFAULT 500

enum lowpassFilterType_e {
    FILTER_PT1 = 0,
    FILTER_BIQUAD,
    FILTER_PT2,
    FILTER_PT3,
};

enum gyroHardwareLpf_e {
    GYRO_HARDWARE_LPF_NORMAL,
    GYRO_HARDWARE_LPF_OPTION_1,
    GYRO_HARDWARE_LPF_OPTION_2,
    GYRO_HARDWARE_LPF_COUNT
};

struct msp_cmd_filter_config_t {
    uint8_t  gyro_lpf1_static_hz_u8;
    uint16_t dterm_lpf1_static_hz;
    uint16_t yaw_lowpass_hz;
    uint16_t gyro_soft_notch_hz_1;
    uint16_t gyro_soft_notch_cutoff_1;
    uint16_t dterm_notch_hz;
    uint16_t dterm_notch_cutoff;
    uint16_t gyro_soft_notch_hz_2;
    uint16_t gyro_soft_notch_cutoff_2;
    uint8_t  dterm_lpf1_type;
    uint8_t  gyro_hardware_lpf;
    uint8_t  unused01;
    uint16_t gyro_lpf1_static_hz_u16;
    uint16_t gyro_lpf2_static_hz;
    uint8_t  gyro_lpf1_type;
    uint8_t  gyro_lpf2_type;
    uint16_t dterm_lpf2_static_hz;
    uint8_t  dterm_lpf2_type;
    uint16_t gyro_lpf1_dyn_min_hz;
    uint16_t gyro_lpf1_dyn_max_hz;
    uint16_t dterm_lpf1_dyn_min_hz;
    uint16_t dterm_lpf1_dyn_max_hz;
    uint8_t  unused02;
    uint8_t  unused03;
    uint16_t dyn_notch_q;
    uint16_t dyn_notch_min_hz;
    uint8_t  rpm_filter_harmonics;
    uint8_t  rpm_filter_min_hz;
    uint16_t dyn_notch_max_hz;
    uint8_t  dterm_lpf1_dyn_expo;
    uint8_t  dyn_notch_count;
} __attribute__ ((packed));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_CMD_PID_ADVANCED
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum tpaMode_e {
    TPA_MODE_PD,
    TPA_MODE_D
};

enum auto_profile_cell_count_e {
    AUTO_PROFILE_CELL_COUNT_STAY = 0,
    AUTO_PROFILE_CELL_COUNT_CHANGE = -1
};

struct msp_cmd_pid_advanced_t {
  uint16_t unused01;
  uint16_t unused02;
  uint16_t unused03;
  uint8_t  unused04;
  uint8_t  unused05;
  uint8_t  feedforward_transition;
  uint8_t  unused06;
  uint8_t  unused07;
  uint8_t  unused08;
  uint8_t  unused09;
  uint16_t rateAccelLimit;
  uint16_t yawRateAccelLimit;
  uint8_t  angle_limit;
  uint8_t  unused10;
  uint16_t unused11;
  uint16_t anti_gravity_gain;
  uint16_t unused12;
  uint8_t  iterm_rotation;
  uint8_t  unused13;
  uint8_t  iterm_relax;
  uint8_t  iterm_relax_type;
  uint8_t  abs_control_gain;
  uint8_t  throttle_boost;
  uint8_t  acro_trainer_angle_limit;
  uint16_t pid_roll_f;
  uint16_t pid_pitch_f;
  uint16_t pid_yaw_f;
  uint8_t  unused14;
  uint8_t  pid_roll;
  uint8_t  pid_pitch;
  uint8_t  pid_yaw;
  uint8_t  d_min_gain;
  uint8_t  d_min_advance;
  uint8_t  use_integrated_yaw;
  uint8_t  integrated_yaw_relax;
  uint8_t  iterm_relax_cutoff;
  uint8_t  motor_output_limit;
  uint8_t  auto_profile_cell_count;
  uint8_t  dyn_idle_min_rpm;
  uint8_t  feedforward_averaging;
  uint8_t  feedforward_smooth_factor;
  uint8_t  feedforward_boost;
  uint8_t  feedforward_max_rate_limit;
  uint8_t  feedforward_jitter_factor;
  uint8_t  vbat_sag_compensation;
  uint8_t  thrustLinearization;
  uint8_t  tpa_mode;
  uint8_t  tpa_rate;
  uint16_t tpa_breakpoint;
} __attribute__ ((packed));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_RC_TUNING
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define CONTROL_RATE_CONFIG_RATE_LIMIT_MAX  1998

enum throttleLimitType_e {
    THROTTLE_LIMIT_TYPE_OFF = 0,
    THROTTLE_LIMIT_TYPE_SCALE,
    THROTTLE_LIMIT_TYPE_CLIP,
    THROTTLE_LIMIT_TYPE_COUNT
};

enum ratesType_e {
    RATES_TYPE_BETAFLIGHT = 0,
    RATES_TYPE_RACEFLIGHT,
    RATES_TYPE_KISS,
    RATES_TYPE_ACTUAL,
    RATES_TYPE_QUICK,
    RATES_TYPE_COUNT
};

struct msp_cmd_rc_tuning_t {
  uint8_t  rcRate_roll;
  uint8_t  rcExpo_roll;
  uint8_t  rates[3];
  uint8_t  unused01;
  uint8_t  thrMid8;
  uint8_t  thrExpo8;
  uint16_t unused02;
  uint8_t  rcExpo_yaw;
  uint8_t  rcRate_yaw;
  uint8_t  rcRate_pitch;
  uint8_t  rcExpo_pitch;
  uint8_t  throttle_limit_type;
  uint8_t  throttle_limit_percent;
  uint16_t rate_limit_roll;
  uint16_t rate_limit_pitch;
  uint16_t rate_limit_yaw;
  uint8_t  rates_type;
} __attribute__ ((packed));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_PID
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct msp_cmd_pid_t {
  uint8_t roll[3];     // 0=P, 1=I, 2=D
  uint8_t pitch[3];    // 0=P, 1=I, 2=D
  uint8_t yaw[3];      // 0=P, 1=I, 2=D
  uint8_t level[3];    // 0=P, 1=I, 2=D
  uint8_t mag[3];      // 0=P, 1=I, 2=D
} __attribute__ ((packed));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_CELLS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum batteryState_e {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT,
    BATTERY_INIT
};

struct msp_cmd_battery_state_t {
    uint8_t  battery_cell_count;
    uint16_t battery_capacity;
    uint8_t  battery_voltage;
    uint16_t mAh_drawn;
    uint16_t amperage;
    uint8_t  battery_state;
} __attribute__ ((packed));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_CMD_RTC
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct msp_cmd_rtc_t {
  uint16_t year;
  uint8_t  month;
  uint8_t  day;
  uint8_t  hours;
  uint8_t  minutes;
  uint8_t  seconds;
  uint16_t millis;
} __attribute__ ((packed));