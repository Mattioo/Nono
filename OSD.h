#pragma once

#include <ReefwingMSP.h>
#include <vector>
#include "Protocol.h"
#include "MSP_OSD.h"
#include "OSD_POS.h"
#include "BTS7960.h"

class OSD {
public:
  OSD(HardwareSerial& serial, uint8_t voltage, uint16_t batteryCapacity, uint8_t cellCount);
  void SetLogger(HardwareSerial* serial = nullptr);
  void Init();
  void Loop();

  void set_name(String craftName);
  void set_battery_voltage(uint8_t voltage, uint8_t batteryState = BATTERY_OK);
  void set_arm(bool state);

  
  void Set_Attitude(const BTS7960State& state);
  void Set_RC(std::vector<uint16_t>& channels);

private:
  HardwareSerial* uart;
  HardwareSerial* logger;

  ReefwingMSP msp;
  msp_packet_t packet;

  msp_fc_variant_t fc_variant;
  msp_fc_version_t fc_version;
  msp_name_t name;
  msp_status_t status;
  msp_analog_t analog;
  msp_battery_state_t battery_state;
  msp_status_ex_t status_ex;
  msp_osd_config_t osd_config;
  
  void set_fc_variant();
  void set_fc_version();
  void set_status(uint32_t flightModeFlags);
  void set_analog(uint8_t voltage, uint16_t rssi = 0, int16_t amperage = 0, uint16_t mAhDrawn = 0);
  void set_battery_state(uint8_t voltage, uint16_t batteryCapacity, uint8_t cellCount, uint8_t batteryState = BATTERY_OK, int16_t amperage = 0, uint16_t mAhDrawn = 0);
  void set_status_ex(uint32_t flightModeFlags);
  void set_osd_config();
  void set_osd_config_positions();

  void send_config();

  uint8_t get_cell_count(uint8_t voltage);
  void log(String val, bool line = true);
};