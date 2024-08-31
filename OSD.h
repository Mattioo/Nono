#pragma once
#include <ReefwingMSP.h>
#include "INA219.h"

#define MSP_API_VERSION 1
#define MSP_FC_VARIANT 2
#define MSP_FC_VERSION 3
#define MSP_NAME 10
#define MSP_OSD_CONFIG 84
#define MSP_FILTER_CONFIG 92
#define MSP_PID_ADVANCED 94
#define MSP_STATUS 101
#define MSP_RC 105
#define MSP_COMP_GPS 107
#define MSP_ANALOG 110
#define MSP_RC_TUNING 111
#define MSP_PID 112
#define MSP_BATTERY_STATE 130
#define MSP_STATUS_EX 150

struct msp_battery_state_struct {
    uint8_t batteryCellCount;
    uint16_t batteryCapacity;
    uint8_t legacyBatteryVoltage;
    uint16_t mAhDrawn;
    uint16_t amperage;
    uint8_t batteryState;
    uint16_t batteryVoltage;
} __attribute__ ((packed));

struct msp_osd_config_struct {
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

class OSD {
  public:
    OSD(HardwareSerial& serial, String name);
    void set_logger(HardwareSerial* serial = nullptr);
    void init(uint16_t wait_to_start_ms);
    void loop();

    void set_arm(bool arm);
    void set_battery_state(INA219State battery_state);
    void set_sensor_distance(uint16_t distance);

  private:
    HardwareSerial* uart;
    
    uint8_t response_buff[255];
    size_t response_length;

    unsigned long previousMillis;
    const long interval;
 
    INA219State battery_state;
    msp_osd_config_struct osd_config;

    uint8_t gps_state;
    bool arm;
    String name;
    uint16_t sensor_distance;

    template <typename T>
    void object_to_byte_array(const T &obj, uint8_t *byteArray, size_t &arrayLength);
    uint8_t calculate_checksum(uint8_t command, uint8_t* data, uint8_t dataLength);

    void handle_request(uint8_t command);
    void send_message(uint8_t command, uint8_t* data, uint8_t dataLength);

    HardwareSerial* logger;
    void log(String val, bool line = true);
};