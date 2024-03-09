#ifndef OSD_h
#define OSD_h

#include <ReefwingMSP.h>
#include "MSP_OSD.h"
#include "OSD_POS.h"

class OSD {
public:
  OSD(HardwareSerial& serial);
  void SetLogger(HardwareSerial* serial = nullptr);
  void Init();

  void SendVariant();
  void SendVersion();

  void SetName(String craftName);
  void SendName();

  void SetStatusDJI(uint32_t flightModeFlags);
  void SendStatusDJI();

  void SetAnalog(uint8_t voltage, uint16_t rssi = 0, int16_t amperage = 0, uint16_t mAhDrawn = 0);
  void SendAnalog();

  void SetBatteryState(uint8_t voltage, uint16_t batteryCapacity, uint8_t batteryState = 0, int16_t amperage = 0, uint16_t mAhDrawn = 0);
  void SendBatteryState();

  void SetRawGPS(int32_t gps_lat = 0, int32_t gps_lon = 0, uint8_t numSat = 0, int32_t gps_alt = 0, int16_t groundspeed = 0);
  void SendRawGPS();

  void SetCompGPS(uint32_t distanceToHome = 0, int16_t directionToHome = 0, int16_t heading = 0);
  void SendCompGPS();

  void SetAttitude(int16_t roll_angle = 0, int16_t pitch_angle = 0);
  void SendAttitude();

  void SetAltitude(int32_t relative_alt = 0, int16_t climb_rate = 0);
  void SendAltitude();

  void SendConfig();

private:
  HardwareSerial* uart;
  HardwareSerial* logger;
  ReefwingMSP msp;

  msp_fc_variant_t fc_variant;
  msp_fc_version_t fc_version = { 0 };
  msp_name_t name;
  msp_status_DJI_t status_DJI = { 0 };
  msp_analog_t analog = { 0 };
  msp_battery_state_t battery_state = { 0 }; 
  msp_raw_gps_t raw_gps = { 0 };
  msp_comp_gps_t comp_gps = { 0 };
  msp_attitude_t attitude = { 0 };
  msp_altitude_t altitude = { 0 };
  msp_osd_config_t msp_osd_config = { 0 };

  void setVariant();
  void setVersion();
  void setConfig();
  void setPositions();

  uint8_t getCellCount(uint8_t voltage);
};

#endif // OSD_H