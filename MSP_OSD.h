#pragma once
#define MSP_CMD_OSD_CONFIG 84
#define MSP_CMD_STATUS_EX 150
#define MAX_NAME_LENGTH 16u
#include <stdint.h>

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_CMD_OSD_CONFIG
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_CMD_NAME
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct msp_name_t {
    char craft_name[MAX_NAME_LENGTH + 1];
} __attribute__ ((packed));

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// MSP_CMD_BATTERY_STATE
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

enum batteryState_e {
    BATTERY_OK = 0,
    BATTERY_WARNING,
    BATTERY_CRITICAL,
    BATTERY_NOT_PRESENT,
    BATTERY_INIT
};

struct msp_battery_state_t {
    uint8_t  battery_cell_count;  // 0 - 255
    uint16_t battery_capacity;
    uint8_t  battery_voltage;     // 0 - 255
    uint16_t mAh_drawn;           // 0 - 0xFFFF
    uint16_t amperage;            // -0x8000 - 0x7FFF
    uint8_t  battery_state;       // batteryState_e
} __attribute__ ((packed));