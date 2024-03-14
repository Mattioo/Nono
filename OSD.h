#pragma once

#include <ReefwingMSP.h>
#include <algorithm>
#include <vector>
#include "Protocol.h"
#include "MSP_OSD.h"
#include "OSD_POS.h"
#include "BTS7960.h"

class OSD {
public:
  OSD(HardwareSerial& serial, uint16_t delay);
  void set_logger(HardwareSerial* serial = nullptr);
  void init(String name, uint16_t batteryCapacity, uint8_t cellCount);
  void loop();

  void set_battery_voltage(uint8_t voltage, uint8_t batteryState = BATTERY_OK);
  void set_rtc(uint16_t year, uint8_t month, uint8_t day, uint8_t hours, uint8_t minutes, uint8_t seconds, uint16_t millis);
  void set_rc(uint16_t array[], int length);
  void set_arm(bool state);

private:
  HardwareSerial* uart;
  HardwareSerial* logger;

  ReefwingMSP msp;
  msp_packet_t packet;

  msp_api_version_t api_version;
  msp_ident_t ident;
  msp_fc_variant_t fc_variant;
  msp_fc_version_t fc_version;
  msp_cmd_name_t name;
  msp_cmd_osd_config_t osd_config;
  msp_cmd_filter_config_t filter_config;
  msp_cmd_pid_advanced_t pid_advanced;
  msp_status_t status;
  msp_rc_t rc;
  msp_raw_gps_t raw_gps;
  msp_comp_gps_t comp_gps;
  msp_attitude_t attitude;
  msp_altitude_t altitude;
  msp_analog_t analog;
  msp_cmd_rc_tuning_t rc_tuning;
  msp_cmd_pid_t pid;
  msp_cmd_battery_state_t battery_state;
  msp_debug_t debug;
  msp_status_ex_t status_ex;
  msp_cmd_rtc_t rtc;

  unsigned long msp_ident_sent_time;
  bool msp_ident_sent;
  bool msp_cmd_osd_config_sent;
  
  void set_api_version();
  void set_ident();
  void set_fc_variant();
  void set_fc_version();
  void set_name(String craftName);
  void set_osd_config();
  void set_filter_config();
  void set_pid_advanced();
  void set_status(uint32_t flightModeFlags);
  void set_analog(uint8_t voltage, uint16_t rssi = 0, int16_t amperage = 0, uint16_t mAhDrawn = 0);
  void set_rc_tuning();
  void set_pid();
  void set_battery_state(uint8_t voltage, uint16_t batteryCapacity, uint8_t cellCount, uint8_t batteryState = BATTERY_OK, int16_t amperage = 0, uint16_t mAhDrawn = 0);
  void set_status_ex(uint32_t flightModeFlags);

  void log(String val, bool line = true);
};