#include "OSD.h"
#include "Protocol.h"

OSD::OSD(HardwareSerial& serial, uint8_t voltage, uint16_t batteryCapacity, uint8_t cellCount) : uart(&serial) {
    OSD::set_fc_version();
    OSD::set_analog(voltage);
    OSD::set_battery_state(voltage, batteryCapacity, cellCount);
    
    OSD::set_osd_config();
    OSD::set_osd_config_positions();
    OSD::set_arm(false);
}

void OSD::SetLogger(HardwareSerial* serial) {
    logger = serial;
}

void OSD::Init() {
    uart->begin(115200); while (!(*uart));
    msp.begin(*uart);
}

void OSD::Loop() {
  if (msp.recv(&packet.recvMessageID, packet.payload, packet.maxSize, &packet.recvSize)) {
    switch(packet.recvMessageID) {
      case MSP_FC_VARIANT:
        msp.response(MSP_FC_VARIANT, &fc_version, sizeof(fc_version));
        OSD::send_config();
        break;
      case MSP_FC_VERSION:
        msp.response(MSP_FC_VERSION, &fc_version, sizeof(fc_version));
        OSD::send_config();
        break;
      case MSP_NAME:
        msp.response(MSP_NAME, &name, sizeof(name));
        OSD::send_config();
        break;
      case MSP_STATUS:
        msp.response(MSP_STATUS, &status, sizeof(status));
        OSD::send_config();
        break;
      case MSP_ANALOG:
        msp.response(MSP_ANALOG, &analog, sizeof(analog));
        OSD::send_config();
        break;
      case MSP_CELLS:
        msp.response(MSP_CELLS, &battery_state, sizeof(battery_state));
        OSD::send_config();
        break;
      case MSP_CMD_STATUS_EX:
        msp.response(MSP_CMD_STATUS_EX, &status_ex, sizeof(status_ex));
        OSD::send_config();
        break;
      default:
        msp.error(packet.recvMessageID, NULL, 0);
        log(String(packet.recvMessageID));
        break;
    }
  }
}

void OSD::set_fc_variant() {
  memset(fc_variant.flightControlIdentifier, '\0', sizeof(fc_variant.flightControlIdentifier));
  strncpy(fc_variant.flightControlIdentifier, REEFWING_IDENTIFIER, sizeof(fc_variant.flightControlIdentifier) - 1);
}

void OSD::set_fc_version() {
  fc_version.versionMajor = 4;
  fc_version.versionMinor = 3;
  fc_version.versionPatchLevel = 0;
}

void OSD::set_name(String craftName) {
  memset(name.craft_name, '\0', sizeof(name.craft_name));
  strncpy(name.craft_name, craftName.c_str(), sizeof(name.craft_name) - 1);
}

void OSD::set_status(uint32_t flightModeFlags) {
  status.cycleTime = 0x0080;
  status.i2cErrorCounter = 0;
  status.sensor = 0x23;
  status.flightModeFlags = flightModeFlags;
  status.configProfileIndex = 1;
}

void OSD::set_status_ex(uint32_t flightModeFlags) {
  status_ex.cycleTime = 0x0080;
  status_ex.i2cErrorCounter = 0;
  status_ex.sensor = 0x23;
  status_ex.flightModeFlags = flightModeFlags;
  status_ex.configProfileIndex = 1;
  status_ex.averageSystemLoadPercent = 10;
  status_ex.armingFlags = 0x0303;
  status_ex.accCalibrationAxisFlags = 0;
}

void OSD::set_arm(bool state) {
  uint32_t flightModeFlags = state ? 0x00000003 : 0x00000002;
  OSD::set_status(flightModeFlags);
  OSD::set_status_ex(flightModeFlags);
}

void OSD::set_analog(uint8_t voltage, uint16_t rssi, int16_t amperage, uint16_t mAhDrawn) {
  analog.vbat = voltage;
  analog.rssi = rssi;
  analog.amperage = amperage;
  analog.mAhDrawn = mAhDrawn;
}

void OSD::set_battery_state(uint8_t voltage, uint16_t batteryCapacity, uint8_t cellCount, uint8_t batteryState, int16_t amperage, uint16_t mAhDrawn) {
  battery_state.amperage = amperage;
  battery_state.battery_voltage = (uint16_t)(voltage * 10);
  battery_state.mAh_drawn = mAhDrawn;
  battery_state.battery_cell_count = cellCount;
  battery_state.battery_capacity = batteryCapacity;
  battery_state.battery_state = batteryState;
}

void OSD::set_battery_voltage(uint8_t voltage, uint8_t batteryState) {
  analog.vbat = voltage;
  battery_state.battery_voltage = (uint16_t)(voltage * 10);
  battery_state.battery_state = batteryState;
}

void OSD::set_osd_config() {
  osd_config.units = 1;
  osd_config.osd_item_count = 56;
  osd_config.osd_stat_count = 24;
  osd_config.osd_timer_count = 2;
  osd_config.osd_warning_count = 16;
  osd_config.osd_profile_count = 1;
  osd_config.osdprofileindex = 1;
  osd_config.overlay_radio_mode = 0;
}

void OSD::set_osd_config_positions() {
  osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
  osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
  osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
  osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
  osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
  osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
  osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
  osd_config.osd_flymode_pos = osd_flymode_pos;
  osd_config.osd_craft_name_pos = osd_craft_name_pos;
  osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
  osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
  osd_config.osd_current_draw_pos = osd_current_draw_pos;
  osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
  osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
  osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
  osd_config.osd_altitude_pos = osd_altitude_pos;
  osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
  osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
  osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
  osd_config.osd_power_pos = osd_power_pos;
  osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
  osd_config.osd_warnings_pos = osd_warnings_pos;
  osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
  osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
  osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
  osd_config.osd_debug_pos = osd_debug_pos;
  osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
  osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
  osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
  osd_config.osd_disarmed_pos = osd_disarmed_pos;
  osd_config.osd_home_dir_pos = osd_home_dir_pos;
  osd_config.osd_home_dist_pos = osd_home_dist_pos;
  osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
  osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
  osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
  osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
  osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
  osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
  osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
  osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
  osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
  osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
  osd_config.osd_g_force_pos = osd_g_force_pos;
  osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
  osd_config.osd_log_status_pos = osd_log_status_pos;
  osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
  osd_config.osd_link_quality_pos = osd_link_quality_pos;
  osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
  osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
  osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
  osd_config.osd_display_name_pos = osd_display_name_pos;
  osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
  osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
  osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
  osd_config.osd_profile_name_pos = osd_profile_name_pos;
  osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
  osd_config.osd_rc_channels_pos = osd_rc_channels_pos;
}

void OSD::send_config() {
  msp.send(MSP_CMD_OSD_CONFIG, &osd_config, sizeof(osd_config));
}

void OSD::log(String val, bool line) {
  if (logger != nullptr) { if (line) logger->println(val); else logger->print(val); }
}