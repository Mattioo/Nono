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
  OSD(HardwareSerial& serial, String name, unsigned long delay, uint16_t batteryCapacity, uint8_t cellCount);
  void set_logger(HardwareSerial* serial = nullptr);
  void init();
  void loop();

  void set_battery_voltage(uint8_t voltage, uint8_t batteryState = BATTERY_OK);
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
  msp_name_t name;
  msp_osd_config_t osd_config;
  msp_filter_config_t filter_config;
  msp_pid_advanced_t pid_advanced;
  msp_status_t status;
  msp_rc_t rc;
  msp_analog_t analog;
  msp_cmd_rc_tuning_t rc_tuning;
  msp_cmd_pid_t pid;
  msp_battery_state_t battery_state;
  msp_status_ex_t status_ex;

  bool config_osd_sent;
  bool ident_req_sent;
  bool initialized;
  
  void set_api_version();
  void set_ident();
  void set_fc_variant();
  void set_fc_version();
  void set_name(String craftName);
  void set_osd_config();
  void set_osd_config_positions();
  void set_filter_config();
  void set_pid_advanced();
  void set_status(uint32_t flightModeFlags);
  void set_analog(uint8_t voltage, uint16_t rssi = 0, int16_t amperage = 0, uint16_t mAhDrawn = 0);
  void set_rc_tuning();
  void set_pid();
  void set_battery_state(uint8_t voltage, uint16_t batteryCapacity, uint8_t cellCount, uint8_t batteryState = BATTERY_OK, int16_t amperage = 0, uint16_t mAhDrawn = 0);
  void set_status_ex(uint32_t flightModeFlags);

  unsigned long config_osd_sent_time;

  uint8_t get_cell_count(uint8_t voltage);
  void log(String val, bool line = true);
};