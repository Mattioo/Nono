#ifndef OSD_h
#define OSD_h

#include <ReefwingMSP.h>
#include "Protocol.h"
#include "MSP_OSD.h"
#include "OSD_POS.h"

class OSD {
public:
  OSD(HardwareSerial& serial, uint8_t voltage, uint16_t batteryCapacity, unsigned long interval = 1000);
  void SetLogger(HardwareSerial* serial = nullptr);
  void Init();
  void Loop();

  void Set_Name(String craftName);
  void Set_Arm(bool state);
  void Set_Battery_Voltage(uint8_t voltage, uint8_t batteryState = BATTERY_OK);

private:
  HardwareSerial* uart;
  HardwareSerial* logger;

  ReefwingMSP msp;

  msp_api_version_t api_version;
  msp_fc_version_t fc_version;
  msp_fc_variant_t fc_variant;
  msp_name_t name;
  msp_osd_config_t osd_config;
  msp_filter_config_t filter_config;
  msp_pid_advanced_t pid_advanced;
  msp_status_t status;
  msp_rc_t rc;
  msp_analog_t analog;
  msp_rc_tuning_t rc_tuning;
  msp_pid_t pid;
  msp_battery_state_t battery_state;
  msp_status_ex_t status_ex;

  void set_api_version();
  void set_fc_version();
  void set_fc_variant();
  void set_status(uint32_t flightModeFlags);
  void set_status_ex(uint32_t flightModeFlags);
  void set_analog(uint8_t voltage, uint16_t rssi = 0, int16_t amperage = 0, uint16_t mAhDrawn = 0);
  void set_battery_state(uint8_t voltage, uint16_t batteryCapacity, uint8_t batteryState = 0, int16_t amperage = 0, uint16_t mAhDrawn = 0);
  void set_osd_config();
  void set_osd_config_positions();

  void send_API_Version();
  void send_FC_Version();
  void send_FC_Variant();
  void send_Name();
  void send_Status();
  void send_Status_Ex();
  void send_Analog();
  void send_Battery_State();
  void send_Filter_Config();
  void send_PID_Advanced();
  void send_RC();
  void send_RC_Tuning();
  void send_PID();
  void send_Config();

  uint8_t get_cell_count(uint8_t voltage);

  unsigned long _previousMillis;
  unsigned long _interval;
};

#endif // OSD_H