#include "OSD.h"

OSD::OSD(HardwareSerial& serial) : uart(&serial) {
    OSD::setVariant();
    OSD::setVersion();
    OSD::setConfig();
    OSD::setPositions();
}

void OSD::SetLogger(HardwareSerial* serial) {
    logger = serial;
}

void OSD::Init() {
    uart->begin(115200); while (!(*uart));
    msp.begin(*uart);
}

void OSD::SendVariant() {
  msp.send(MSP_FC_VARIANT, &fc_variant, sizeof(fc_variant));
}

void OSD::SendVersion() {
  msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));
}

void OSD::SetName(String craftName) {
  for (int i = 0; i < sizeof(name.craft_name); i++) {
    name.craft_name[i] = 0;
  }
  const char* displayName = craftName.c_str();
  memcpy(name.craft_name, displayName, sizeof(displayName));
}

void OSD::SendName() {
  msp.send(MSP_NAME, &name, sizeof(name));
}

void OSD::SetStatusDJI(uint32_t flightModeFlags) {
  status_DJI.cycleTime = 0x0080;
  status_DJI.i2cErrorCounter = 0;
  status_DJI.sensor = 0x23;
  status_DJI.configProfileIndex = 0;
  status_DJI.averageSystemLoadPercent = 7;
  status_DJI.accCalibrationAxisFlags = 0;
  status_DJI.DJI_ARMING_DISABLE_FLAGS_COUNT = 20;
  status_DJI.djiPackArmingDisabledFlags = (1 << 24);
  status_DJI.flightModeFlags = flightModeFlags; // 0x00000002 -> DISARM, 0x00000003 -> ARM
}

void OSD::SendStatusDJI() {
  status_DJI.armingFlags = 0x0303;
  msp.send(MSP_STATUS_EX, &status_DJI, sizeof(status_DJI));
  status_DJI.armingFlags = 0x0000;
  msp.send(MSP_STATUS, &status_DJI, sizeof(status_DJI));
}

void OSD::SetAnalog(uint8_t voltage, uint16_t rssi, int16_t amperage, uint16_t mAhDrawn) {
  analog.vbat = voltage;
  analog.rssi = rssi;
  analog.amperage = amperage;
  analog.mAhDrawn = mAhDrawn;
}

void OSD::SendAnalog() {
  msp.send(MSP_ANALOG, &analog, sizeof(analog));
}

void OSD::SetBatteryState(uint8_t voltage, uint16_t batteryCapacity, uint8_t batteryState, int16_t amperage, uint16_t mAhDrawn) {
  battery_state.amperage = amperage;
  battery_state.batteryVoltage = (uint16_t)(voltage * 10);
  battery_state.mAhDrawn = mAhDrawn;
  battery_state.batteryCellCount = getCellCount(voltage);
  battery_state.batteryCapacity = batteryCapacity;
  battery_state.batteryState = batteryState;
  battery_state.legacyBatteryVoltage = voltage;
}

void OSD::SendBatteryState() {
  msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));
}

void OSD::SetRawGPS(int32_t gps_lat, int32_t gps_lon, uint8_t numSat, int32_t gps_alt, int16_t groundspeed) {
  raw_gps.lat = gps_lat;
  raw_gps.lon = gps_lon;
  raw_gps.numSat = numSat;
  raw_gps.alt = gps_alt;
  raw_gps.groundSpeed = groundspeed;
}

void OSD::SendRawGPS() {
  msp.send(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps));
}

void OSD::SetCompGPS(uint32_t distanceToHome, int16_t directionToHome, int16_t heading) {
  comp_gps.distanceToHome = (int16_t)distanceToHome;
  comp_gps.directionToHome = directionToHome - heading;
}

void OSD::SendCompGPS() {
  msp.send(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps));
}

void OSD::SetAttitude(int16_t roll_angle, int16_t pitch_angle) {
  attitude.pitch = pitch_angle * 10;
  attitude.roll = roll_angle * 10;
}

void OSD::SendAttitude() {
  msp.send(MSP_ATTITUDE, &attitude, sizeof(attitude));
}

void OSD::SetAltitude(int32_t relative_alt, int16_t climb_rate) {
  altitude.estimatedActualPosition = relative_alt;
  altitude.estimatedActualVelocity = (int16_t)(climb_rate);
}

void OSD::SendAltitude() {
  msp.send(MSP_ALTITUDE, &altitude, sizeof(altitude));
}

void OSD::SendConfig() {
  msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}

void OSD::setVariant() {
  for (int i = 0; i < sizeof(fc_variant.flightControlIdentifier); i++) {
    fc_variant.flightControlIdentifier[i] = 0;
  }
  const char fcVariant[5] = "BTFL";
  memcpy(fc_variant.flightControlIdentifier, fcVariant, sizeof(fcVariant));
}

void OSD::setVersion() {
  fc_version.versionMajor = 4;
  fc_version.versionMinor = 1;
  fc_version.versionPatchLevel = 1;
}

void OSD::setConfig() {
  msp_osd_config.units = 1;
  msp_osd_config.osd_item_count = 56;
  msp_osd_config.osd_stat_count = 24;
  msp_osd_config.osd_timer_count = 2;
  msp_osd_config.osd_warning_count = 16;
  msp_osd_config.osd_profile_count = 1;
  msp_osd_config.osdprofileindex = 1;
  msp_osd_config.overlay_radio_mode = 0;
}

void OSD::setPositions() {
  msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
  msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
  msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
  msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
  msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
  msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
  msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
  msp_osd_config.osd_flymode_pos = osd_flymode_pos;
  msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
  msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
  msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
  msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
  msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
  msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
  msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
  msp_osd_config.osd_altitude_pos = osd_altitude_pos;
  msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
  msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
  msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
  msp_osd_config.osd_power_pos = osd_power_pos;
  msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
  msp_osd_config.osd_warnings_pos = osd_warnings_pos;
  msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
  msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
  msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
  msp_osd_config.osd_debug_pos = osd_debug_pos;
  msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
  msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
  msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
  msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
  msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
  msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
  msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
  msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
  msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
  msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
  msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
  msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
  msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
  msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
  msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
  msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
  msp_osd_config.osd_g_force_pos = osd_g_force_pos;
  msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
  msp_osd_config.osd_log_status_pos = osd_log_status_pos;
  msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
  msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
  msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
  msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
  msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
  msp_osd_config.osd_display_name_pos = osd_display_name_pos;
  msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
  msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
  msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
  msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
  msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
  msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;
}

uint8_t OSD::getCellCount(uint8_t voltage) {
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