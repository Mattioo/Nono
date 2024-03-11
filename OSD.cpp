#include "OSD.h"

OSD::OSD(HardwareSerial& serial, unsigned long interval) : uart(&serial), _interval(interval) {
    OSD::set_api_version();
    OSD::set_fc_version();
    OSD::set_fc_variant();   
    OSD::set_osd_config();
    OSD::set_osd_config_positions();
}

void OSD::SetLogger(HardwareSerial* serial) {
    logger = serial;
}

void OSD::Init() {
    uart->begin(115200); while (!(*uart));
    msp.begin(*uart);
}

void OSD::Loop() {
  unsigned long currentMillis = millis();
  if (currentMillis - _previousMillis >= _interval) {
    send_API_Version();
    send_FC_Variant();
    send_FC_Version();
    send_Name();
    send_Filter_Config();
    send_PID_Advanced();
    send_Status();
    send_RC();
    send_Analog();
    send_RC_Tuning();
    send_PID();
    send_Battery_State();
    send_Status_Ex();
    send_Config();

    _previousMillis = currentMillis;
  }
}

void OSD::Set_Name(String craftName) {
  memset(name.craft_name, 0, sizeof(name.craft_name));
  size_t len = craftName.length() < MAX_NAME_LENGTH ? craftName.length() : MAX_NAME_LENGTH;
  memcpy(name.craft_name, craftName.c_str(), len);
}

void OSD::Set_Status(uint32_t flightModeFlags) {
  status.cycleTime = 0x0080;
  status.i2cErrorCounter = 0;
  status.sensor = 0x23;
  status.flightModeFlags = flightModeFlags;
  status.configProfileIndex = 1;
}

void OSD::Set_Status_Ex(uint32_t flightModeFlags) {
  status_ex.cycleTime = 0x0080;
  status_ex.i2cErrorCounter = 0;
  status_ex.sensor = 0x23;
  status_ex.flightModeFlags = flightModeFlags;
  status_ex.configProfileIndex = 1;
  status_ex.averageSystemLoadPercent = 10;
  status_ex.armingFlags = 0x0303;
  status_ex.accCalibrationAxisFlags = 0;
}

void OSD::Set_Analog(uint8_t voltage, uint16_t rssi, int16_t amperage, uint16_t mAhDrawn) {
  analog.vbat = voltage;
  analog.rssi = rssi;
  analog.amperage = amperage;
  analog.mAhDrawn = mAhDrawn;
}

void OSD::Set_Battery_State(uint8_t voltage, uint16_t batteryCapacity, uint8_t batteryState, int16_t amperage, uint16_t mAhDrawn) {
  battery_state.amperage = amperage;
  battery_state.batteryVoltage = (uint16_t)(voltage * 10);
  battery_state.mAhDrawn = mAhDrawn;
  battery_state.batteryCellCount = get_cell_count(voltage);
  battery_state.batteryCapacity = batteryCapacity;
  battery_state.batteryState = batteryState;
}

void OSD::set_api_version() {
  api_version.protocolVersion = 1;
  api_version.APIMajor = 3;
  api_version.APIMinor = 0;
}

void OSD::set_fc_version() {
  fc_version.versionMajor = 3;
  fc_version.versionMinor = 5;
  fc_version.versionPatchLevel = 0;
}

void OSD::set_fc_variant() {
  memset(fc_variant.flightControlIdentifier, 0, sizeof(fc_variant.flightControlIdentifier));
  String variant = "BFTL";
  memcpy(fc_variant.flightControlIdentifier, variant.c_str(), variant.length());
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

void OSD::send_API_Version() {
  msp.send(MSP_CMD_API_VERSION, &api_version, sizeof(api_version));
}

void OSD::send_FC_Version() {
  msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));
}

void OSD::send_FC_Variant() {
  msp.send(MSP_FC_VARIANT, &fc_variant, sizeof(fc_variant));
}

void OSD::send_Name() {
  msp.send(MSP_NAME, &name, sizeof(name));
}

void OSD::send_Status() {
  msp.send(MSP_STATUS, &status, sizeof(status));
}

void OSD::send_RC() {
  msp.send(MSP_CMD_RC, &rc, sizeof(rc));
}

void OSD::send_Status_Ex() {
  msp.send(MSP_CMD_STATUS_EX, &status_ex, sizeof(status_ex));
}

void OSD::send_Analog() {
  msp.send(MSP_ANALOG, &analog, sizeof(analog));
}

void OSD::send_RC_Tuning() {
  msp.send(MSP_CMD_RC_TUNING, &rc_tuning, sizeof(rc_tuning));
}

void OSD::send_PID() {
  msp.send(MSP_CMD_PID, &pid, sizeof(pid));
}

void OSD::send_Battery_State() {
  msp.send(MSP_CMD_BATTERY_STATE, &battery_state, sizeof(battery_state));
}

void OSD::send_Config() {
  msp.send(MSP_CMD_OSD_CONFIG, &osd_config, sizeof(osd_config));
}

void OSD::send_Filter_Config() {
  msp.send(MSP_CMD_FILTER_CONFIG, &filter_config, sizeof(filter_config));
}

void OSD::send_PID_Advanced() {
  msp.send(MSP_CMD_PID_ADVANCED, &pid_advanced, sizeof(pid_advanced));
}

uint8_t OSD::get_cell_count(uint8_t voltage) {
  if (voltage == 0)
    return 0;
  else if (voltage < 43)
    return 1;
  else if (voltage < 85)
    return 2;
  else if (voltage < 127)
    return 3;
  else if (voltage < 169)
    return 4;
  else if (voltage < 211)
    return 5;
  else if (voltage < 255)
    return 6;
}