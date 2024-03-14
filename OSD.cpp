#include "WString.h"
#include "OSD.h"
#include "Protocol.h"

OSD::OSD(HardwareSerial& serial, uint16_t delay) : uart(&serial), msp_ident_sent_time(millis() + delay), msp_ident_sent(false), msp_cmd_osd_config_sent(false) {
}

void OSD::set_logger(HardwareSerial* serial) {
    logger = serial;
}

void OSD::init(String name, uint16_t batteryCapacity, uint8_t cellCount) {
    uart->begin(115200); while (!(*uart));
    msp.begin(*uart);

    OSD::set_api_version();
    OSD::set_ident();
    OSD::set_fc_variant();
    OSD::set_fc_version();
    OSD::set_name(name);
    OSD::set_filter_config();
    OSD::set_pid_advanced();
    OSD::set_arm(false);
    OSD::set_rc(nullptr, 0);
    OSD::set_analog(0);
    OSD::set_rc_tuning();
    OSD::set_pid();
    OSD::set_battery_state(0, batteryCapacity, cellCount);
    OSD::set_osd_config();
    OSD::set_rtc(2024, 1, 1, 0, 0, 0, 1);

    log("[OSD] START");
}

void OSD::loop() {
  if (millis() >= OSD::msp_ident_sent_time)
  {
    if (!OSD::msp_ident_sent) {
      msp.send(MSP_IDENT, &ident, sizeof(ident));   
      OSD::msp_ident_sent = true;

      log("[OSD] MSP_IDENT SEND");
    }
    if (OSD::msp_ident_sent && msp.recv(&packet.recvMessageID, packet.payload, packet.maxSize, &packet.recvSize))
    {
      switch(packet.recvMessageID) {
        case MSP_API_VERSION:
          msp.response(MSP_API_VERSION, &api_version, sizeof(api_version));
          break;
        case MSP_IDENT:
          msp.response(MSP_IDENT, &ident, sizeof(ident));
          break;
        case MSP_FC_VARIANT:
          msp.response(MSP_FC_VARIANT, &fc_version, sizeof(fc_version));
          break;
        case MSP_FC_VERSION:
          msp.response(MSP_FC_VERSION, &fc_version, sizeof(fc_version));
          break;
        case MSP_NAME:
          msp.response(MSP_NAME, &name, sizeof(name));
          if (!OSD::msp_cmd_osd_config_sent)
          {
            msp.send(MSP_CMD_OSD_CONFIG, &osd_config, sizeof(osd_config));
            OSD::msp_cmd_osd_config_sent = true;

            log("[OSD] MSP_CMD_OSD_CONFIG SEND");
          }
          break;
        case MSP_CMD_OSD_CONFIG:
          msp.response(MSP_CMD_OSD_CONFIG, &osd_config, sizeof(osd_config));
          break;
        case MSP_CMD_FILTER_CONFIG:
          msp.response(MSP_CMD_FILTER_CONFIG, &filter_config, sizeof(filter_config));
          break;
        case MSP_CMD_PID_ADVANCED:
          msp.response(MSP_CMD_PID_ADVANCED, &pid_advanced, sizeof(pid_advanced));
          break;
        case MSP_STATUS:
          msp.response(MSP_STATUS, &status, sizeof(status));
          break;
        case MSP_RC:
          msp.response(MSP_RC, &rc, sizeof(rc));
          break;
        case MSP_RAW_GPS:
          msp.response(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps));
          break;
        case MSP_COMP_GPS:
          msp.response(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps));
          break;
        case MSP_ATTITUDE:
          msp.response(MSP_ATTITUDE, &attitude, sizeof(attitude));
          break;
        case MSP_ALTITUDE:
          msp.response(MSP_ALTITUDE, &altitude, sizeof(altitude));
          break;
        case MSP_ANALOG:
          msp.response(MSP_ANALOG, &analog, sizeof(analog));
          break;
        case MSP_RC_TUNING:
          msp.response(MSP_RC_TUNING, &rc_tuning, sizeof(rc_tuning));
          break;
        case MSP_PID:
          msp.response(MSP_PID, &pid, sizeof(pid));
          break;
        case MSP_CELLS:
          msp.response(MSP_CELLS, &battery_state, sizeof(battery_state));
          break;
        case MSP_DEBUG:
          msp.response(MSP_DEBUG, &debug, sizeof(debug));
          break;
        case MSP_CMD_STATUS_EX:
          msp.response(MSP_CMD_STATUS_EX, &status_ex, sizeof(status_ex));
          break;
        case MSP_CMD_RTC:
          msp.response(MSP_CMD_RTC, &rtc, sizeof(rtc));
          break;
        default:
          msp.error(packet.recvMessageID, NULL, 0);
          break;
      }
      log("[OSD] MSP RESPONSE " + String(packet.recvMessageID));
    }
  }
}

void OSD::set_api_version() {
  api_version.protocolVersion = MSP_PROTOCOL_VERSION;
  api_version.APIMajor = API_VERSION_MAJOR;
  api_version.APIMajor = API_VERSION_MINOR;
}

void OSD::set_ident() {
  ident.multiWiiVersion = MSP_PROTOCOL_VERSION;
  ident.multiType = QUADX;
  ident.mspVersion = MSP_FEATURE_VBAT | MSP_FEATURE_TELEMETRY | MSP_FEATURE_RX_MSP | MSP_FEATURE_OSD;
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

void OSD::set_rc(uint16_t array[], int length) {
  memset(rc.channelValue, 0, MSP_MAX_SUPPORTED_CHANNELS); int eToCopy = std::min(length, MSP_MAX_SUPPORTED_CHANNELS);
  memcpy(rc.channelValue, array, sizeof(uint16_t) * eToCopy);
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

void OSD::set_rc_tuning() {
  rc_tuning.rcRate_roll = 7;
  rc_tuning.rcExpo_roll = 0;
  memset(rc_tuning.rates, 67, sizeof(rc_tuning.rates));
  rc_tuning.unused01 = 0;
  rc_tuning.thrMid8 = 50;
  rc_tuning.thrExpo8 = 0;
  rc_tuning.unused02 = 0;
  rc_tuning.rcExpo_yaw = 0;
  rc_tuning.rcRate_yaw = 7;
  rc_tuning.rcRate_pitch = 7;
  rc_tuning.rcExpo_pitch = 0;
  rc_tuning.throttle_limit_type = THROTTLE_LIMIT_TYPE_OFF;
  rc_tuning.throttle_limit_percent = 100;
  rc_tuning.rate_limit_roll = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX;
  rc_tuning.rate_limit_pitch = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX;
  rc_tuning.rate_limit_yaw = CONTROL_RATE_CONFIG_RATE_LIMIT_MAX;
  rc_tuning.rates_type = RATES_TYPE_ACTUAL;
}

void OSD::set_pid() {
  memset(pid.roll, 100, sizeof(pid.roll));
  memset(pid.pitch, 100, sizeof(pid.pitch));
  memset(pid.yaw, 100, sizeof(pid.yaw));
  memset(pid.level, 100, sizeof(pid.level));
  memset(pid.mag, 100, sizeof(pid.mag));
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

void OSD::set_filter_config() {
  filter_config.gyro_lpf1_static_hz_u8 = GYRO_LPF1_DYN_MIN_HZ_DEFAULT;
  filter_config.dterm_lpf1_static_hz = 100;
  filter_config.yaw_lowpass_hz = 0;
  filter_config.gyro_soft_notch_hz_1 = 0;
  filter_config.gyro_soft_notch_cutoff_1 = 0;
  filter_config.dterm_notch_hz = 260;
  filter_config.dterm_notch_cutoff = 160;
  filter_config.gyro_soft_notch_hz_2 = 0;
  filter_config.gyro_soft_notch_cutoff_2 = 0;
  filter_config.dterm_lpf1_type = FILTER_BIQUAD;
  filter_config.gyro_hardware_lpf = GYRO_HARDWARE_LPF_NORMAL;
  filter_config.unused01 = 0;
  filter_config.gyro_lpf1_static_hz_u16 = GYRO_LPF1_DYN_MIN_HZ_DEFAULT;
  filter_config.gyro_lpf2_static_hz = GYRO_LPF2_HZ_DEFAULT;
  filter_config.gyro_lpf1_type = FILTER_PT1;
  filter_config.gyro_lpf2_type = FILTER_PT1;
  filter_config.dterm_lpf2_static_hz = 0;
  filter_config.dterm_lpf2_type = FILTER_PT1;
  filter_config.gyro_lpf1_dyn_min_hz = 0;
  filter_config.gyro_lpf1_dyn_max_hz = 0;
  filter_config.dterm_lpf1_dyn_min_hz = 0;
  filter_config.dterm_lpf1_dyn_max_hz = 0;
  filter_config.unused02 = 0;
  filter_config.unused03 = 0;
  filter_config.dyn_notch_q = 0;
  filter_config.dyn_notch_min_hz = 0;
  filter_config.rpm_filter_harmonics = 0;
  filter_config.rpm_filter_min_hz = 0;
  filter_config.dyn_notch_max_hz = 0;
  filter_config.dterm_lpf1_dyn_expo = 0;
  filter_config.dyn_notch_count = 0;
}

void OSD::set_pid_advanced() {
  pid_advanced.unused01 = 0;
  pid_advanced.unused02 = 0;
  pid_advanced.unused03 = 0;
  pid_advanced.unused04 = 0;
  pid_advanced.unused05 = 0;
  pid_advanced.feedforward_transition = 0;
  pid_advanced.unused06 = 0;
  pid_advanced.unused07 = 0;
  pid_advanced.unused08 = 0;
  pid_advanced.unused09 = 0;
  pid_advanced.rateAccelLimit = 0;
  pid_advanced.yawRateAccelLimit = 100;
  pid_advanced.angle_limit = 60;
  pid_advanced.unused10 = 0;
  pid_advanced.unused11 = 0;
  pid_advanced.anti_gravity_gain = 10;
  pid_advanced.unused12 = 0;
  pid_advanced.iterm_rotation = false;
  pid_advanced.unused13 = 0;
  pid_advanced.iterm_relax = 0;
  pid_advanced.iterm_relax_type = 0;
  pid_advanced.abs_control_gain = 0;
  pid_advanced.throttle_boost = 0;
  pid_advanced.acro_trainer_angle_limit = 0;
  pid_advanced.pid_roll_f = 0;
  pid_advanced.pid_pitch_f = 0;
  pid_advanced.pid_yaw_f = 0;
  pid_advanced.unused14 = 0;
  pid_advanced.pid_roll = 0;
  pid_advanced.pid_pitch = 0;
  pid_advanced.pid_yaw = 0;
  pid_advanced.d_min_gain = 0;
  pid_advanced.d_min_advance = 0;
  pid_advanced.use_integrated_yaw = 0;
  pid_advanced.integrated_yaw_relax = 0;
  pid_advanced.iterm_relax_cutoff = 0;
  pid_advanced.motor_output_limit = 100;
  pid_advanced.auto_profile_cell_count = AUTO_PROFILE_CELL_COUNT_STAY;
  pid_advanced.dyn_idle_min_rpm = 0;
  pid_advanced.feedforward_averaging = 0;
  pid_advanced.feedforward_smooth_factor = 0;
  pid_advanced.feedforward_boost = 0;
  pid_advanced.feedforward_max_rate_limit = 0;
  pid_advanced.feedforward_jitter_factor = 0;
  pid_advanced.vbat_sag_compensation = 0;
  pid_advanced.thrustLinearization = 0;
  pid_advanced.tpa_mode = TPA_MODE_D;
  pid_advanced.tpa_rate = 65;
  pid_advanced.tpa_breakpoint = 1350;
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

  osd_config.osd_altitude_pos = osd_altitude_pos;
  osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
  osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
  osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
  osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
  osd_config.osd_craft_name_pos = osd_craft_name_pos;
  osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
  osd_config.osd_home_dir_pos = osd_home_dir_pos;
  osd_config.osd_home_dist_pos = osd_home_dist_pos;
  osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
  osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
  osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
  osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
  osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
  osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
  osd_config.osd_display_name_pos = osd_display_name_pos;
  osd_config.osd_flymode_pos = osd_flymode_pos;
  osd_config.osd_current_draw_pos = osd_current_draw_pos;
  osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
  osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
  osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
  osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
  osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
  osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
  osd_config.osd_power_pos = osd_power_pos;
  osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
  osd_config.osd_warnings_pos = osd_warnings_pos;
  osd_config.osd_debug_pos = osd_debug_pos;
  osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
  osd_config.osd_disarmed_pos = osd_disarmed_pos;
  osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
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
  osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
  osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
  osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
  osd_config.osd_profile_name_pos = osd_profile_name_pos;
  osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
  osd_config.osd_rc_channels_pos = osd_rc_channels_pos;
}

void OSD::set_rtc(uint16_t year, uint8_t month, uint8_t day, uint8_t hours, uint8_t minutes, uint8_t seconds, uint16_t millis) {
  rtc.year = year;
  rtc.month = month;
  rtc.day = day;
  rtc.hours = hours;
  rtc.minutes = minutes;
  rtc.seconds = seconds;
  rtc.millis = millis;
}

void OSD::log(String val, bool line) {
  if (logger != nullptr) { if (line) logger->println(val); else logger->print(val); }
}