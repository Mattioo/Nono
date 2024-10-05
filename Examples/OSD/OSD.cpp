#include "OSD.h"

OSD::OSD(HardwareSerial& serial) : logger(nullptr), previousMillis(0), interval(100) {
  this->uart = &serial;
}

void OSD::set_logger(HardwareSerial* serial) {
    logger = serial;
}

void OSD::init(uint16_t wait_to_start_ms, uint16_t batteryCapacity, uint8_t batteryCellCount, float vMin, float vMax) {
    uart->begin(115200); while (!(*uart));
    msp.begin(*uart);

    delay(wait_to_start_ms);

    strcpy(fc_variant.flightControlIdentifier, REEFWING_IDENTIFIER);
    strcpy(name.displayName, "Nono");

    api_version.protocolVersion = 0;
    api_version.APIMajor = 1;
    api_version.APIMinor = 46;

    fc_version.versionMajor = 4;
    fc_version.versionMinor = 5;
    fc_version.versionPatchLevel = 0;
  
    state.batteryCapacity = batteryCapacity;
    state.batteryCellCount = batteryCellCount;
    state.vMin = batteryCellCount * vMin;
    state.vMax = batteryCellCount * vMax;

    log("[OSD] START");
}

void OSD::loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    uint16_t batteryVoltage = state.voltage * 10;
    uint16_t mAhDrawn = (uint16_t)constrain(state.batteryCapacity * (1.0 - ((state.voltage / 10.0) - state.vMin) / (state.vMax - state.vMin)), 0, state.batteryCapacity);

    uint8_t battery_data[] =
    {
        state.batteryCellCount,
        (uint8_t)(state.batteryCapacity & 0xFF), (uint8_t)(state.batteryCapacity >> 8),
        state.voltage,
        (uint8_t)(mAhDrawn & 0xFF), (uint8_t)(mAhDrawn >> 8),
        0x00, 0x00,
        0x00,
        (uint8_t)(batteryVoltage & 0xFF), (uint8_t)(batteryVoltage >> 8)
    };

    uint8_t status_data[] =
    {
        0x80, 0x00,
        0x00, 0x00,
        0x23, 0x00,
        (state.arm ? 0x03 : 0x02), 0x00, 0x00, 0x00,
        0x01
    };

    uint8_t sensor_distance_data[] =
    {
      (uint8_t)(state.sensor_distance & 0xFF), (uint8_t)(state.sensor_distance >> 8),
      0x00, 0x00,
      (++state.gps_state & 1)
    };

    msp.send(MSP_API_VERSION, &api_version, sizeof(api_version));
    msp.send(MSP_FC_VARIANT, &fc_variant, sizeof(fc_variant));
    msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));
    msp.send(MSP_NAME, &name, sizeof(name));
    msp.send(MSP_STATUS, status_data, sizeof(status_data));
    msp.send(MSP_CELLS, battery_data, sizeof(battery_data));
    msp.send(MSP_COMP_GPS, sensor_distance_data, sizeof(sensor_distance_data));
    msp.send(MSP_OSD_CONFIG, &config, sizeof(config));
  }
}

void OSD::set_state(bool arm, uint8_t voltage, uint16_t sensor_distance) {
  state.arm = arm;
  state.voltage = voltage;
  state.sensor_distance = sensor_distance;
  log("[OSD] ARM: " + String(arm) + " Voltage: " + String(voltage) + " Distance: " + String(sensor_distance));
}

void OSD::log(String val, bool line) {
  if (logger != nullptr) { if (line) logger->println(val); else logger->print(val); }
}