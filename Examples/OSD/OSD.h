#pragma once
#include <Arduino.h>
#include <ReefwingMSP.h>

#define MSP_OSD_CONFIG  84

struct msp_osd_config_t {
    uint8_t  osdflags;
    uint8_t  video_system;
    uint8_t  units = 1;
    uint8_t  rssi_alarm;
    uint16_t cap_alarm;
    uint8_t  old_timer_alarm;
    uint8_t  osd_item_count = 56;
    uint16_t alt_alarm;
    uint16_t osd_rssi_value_pos = 234;
    uint16_t osd_main_batt_voltage_pos = 234;
    uint16_t osd_crosshairs_pos = 234;
    uint16_t osd_artificial_horizon_pos = 234;
    uint16_t osd_horizon_sidebars_pos= 234;
    uint16_t osd_item_timer_1_pos;
    uint16_t osd_item_timer_2_pos;
    uint16_t osd_flymode_pos = 234;
    uint16_t osd_craft_name_pos = 2048;
    uint16_t osd_throttle_pos_pos = 234;
    uint16_t osd_vtx_channel_pos = 234;
    uint16_t osd_current_draw_pos = 234;
    uint16_t osd_mah_drawn_pos = 234;
    uint16_t osd_gps_speed_pos = 234;
    uint16_t osd_gps_sats_pos = 234;
    uint16_t osd_altitude_pos = 234;
    uint16_t osd_roll_pids_pos = 234;
    uint16_t osd_pitch_pids_pos = 234;
    uint16_t osd_yaw_pids_pos = 234;
    uint16_t osd_power_pos = 234;
    uint16_t osd_pidrate_profile_pos = 234;
    uint16_t osd_warnings_pos = 234;
    uint16_t osd_avg_cell_voltage_pos = 234;
    uint16_t osd_gps_lon_pos = 234;
    uint16_t osd_gps_lat_pos = 234;
    uint16_t osd_debug_pos = 234;
    uint16_t osd_pitch_angle_pos = 234;
    uint16_t osd_roll_angle_pos = 234;
    uint16_t osd_main_batt_usage_pos = 2531;
    uint16_t osd_disarmed_pos = 2284;
    uint16_t osd_home_dir_pos = 234;
    uint16_t osd_home_dist_pos = 2537;
    uint16_t osd_numerical_heading_pos = 234;
    uint16_t osd_numerical_vario_pos = 234;
    uint16_t osd_compass_bar_pos = 234;
    uint16_t osd_esc_tmp_pos = 234;
    uint16_t osd_esc_rpm_pos = 234;
    uint16_t osd_remaining_time_estimate_pos = 234;
    uint16_t osd_rtc_datetime_pos = 234;
    uint16_t osd_adjustment_range_pos = 234;
    uint16_t osd_core_temperature_pos = 234;
    uint16_t osd_anti_gravity_pos = 234;
    uint16_t osd_g_force_pos = 234;
    uint16_t osd_motor_diag_pos = 234;
    uint16_t osd_log_status_pos = 234;
    uint16_t osd_flip_arrow_pos = 234;
    uint16_t osd_link_quality_pos = 234;
    uint16_t osd_flight_dist_pos = 234;
    uint16_t osd_stick_overlay_left_pos = 234;
    uint16_t osd_stick_overlay_right_pos = 234;
    uint16_t osd_display_name_pos = 234;
    uint16_t osd_esc_rpm_freq_pos = 234;
    uint16_t osd_rate_profile_name_pos = 234;
    uint16_t osd_pid_profile_name_pos = 234;
    uint16_t osd_profile_name_pos = 234;
    uint16_t osd_rssi_dbm_value_pos = 234;
    uint16_t osd_rc_channels_pos = 234;
    uint8_t  osd_stat_count = 24;
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
    uint16_t osd_timer_count = 2;
    uint16_t osd_timer_1;
    uint16_t osd_timer_2;
    uint16_t enabledwarnings;
    uint8_t  osd_warning_count = 16;
    uint32_t enabledwarnings_1_41_plus;
    uint8_t  osd_profile_count = 1;
    uint8_t  osdprofileindex = 1;
    uint8_t  overlay_radio_mode = 0;
} __attribute__ ((packed));

struct msp_name_t {
  char displayName[16];
} __attribute__ ((packed));

struct OSD_state_t {
    bool arm = false;
    uint8_t voltage = 0;
    uint8_t gps_state = 0;
    uint16_t sensor_distance = 0;
    uint16_t batteryCapacity;
    uint8_t batteryCellCount;
    float vMin;
    float vMax;
} __attribute__ ((packed));

class OSD {
  public:
    OSD(HardwareSerial& serial);
    void set_logger(HardwareSerial* serial = nullptr);
    void init(uint16_t wait_to_start_ms, uint16_t batteryCapacity, uint8_t batteryCellCount, float vMin, float vMax);
    void set_state(bool arm, uint8_t voltage, uint16_t sensor_distance);
    void loop();

  private:
    ReefwingMSP msp;

    msp_api_version_t api;
    msp_api_version_t api_version;
    msp_fc_version_t fc_version;
    msp_fc_variant_t fc_variant;
    msp_name_t name;
    msp_osd_config_t config;

    msp_packet_t packet;

    unsigned long previousMillis;
    const long interval;

    OSD_state_t state;
    
    HardwareSerial* uart;
    HardwareSerial* logger;
    void log(String val, bool line = true);
};