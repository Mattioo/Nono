#include "OSD.h"

OSD::OSD(HardwareSerial& serial, String name) : logger(nullptr), arm(false), gps_state(0), previousMillis(0), interval(100) {
  this->uart = &serial;
  this->name = name;
}

void OSD::set_logger(HardwareSerial* serial) {
    logger = serial;
}

void OSD::init(uint16_t wait_to_start_ms) {
    uart->begin(115200); while (!(*uart));
    delay(wait_to_start_ms);

    log("[OSD] START");
}

void OSD::loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    handle_request(MSP_API_VERSION);
    handle_request(MSP_FC_VARIANT);
    handle_request(MSP_FC_VERSION);
    handle_request(MSP_NAME);
    handle_request(MSP_STATUS);
    handle_request(MSP_STATUS_EX);
    handle_request(MSP_BATTERY_STATE);
    handle_request(MSP_COMP_GPS);
    handle_request(MSP_OSD_CONFIG);
  }
}

void OSD::handle_request(uint8_t command) {
  switch (command) {
    case MSP_API_VERSION: {
      uint8_t responseData[] = {MSP_PROTOCOL_VERSION, API_VERSION_MAJOR, API_VERSION_MINOR};
      send_message(MSP_API_VERSION, responseData, sizeof(responseData));
      break;
    }
    case MSP_FC_VARIANT: {
      uint8_t responseData[] = {'B', 'F', 'L', 'T'};
      send_message(MSP_FC_VARIANT, responseData, sizeof(responseData));
      break;
    }
    case MSP_FC_VERSION: {
      uint8_t responseData[] = {4, 2, 0};
      send_message(MSP_FC_VERSION, responseData, sizeof(responseData));
      break;
    }
    case MSP_NAME: {
      uint8_t responseData[] = {'\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0', '\0'};
      size_t responseSize = sizeof(responseData);
      strncpy((char*)responseData, name.c_str(), responseSize - 1);
      send_message(MSP_NAME, responseData, sizeof(responseData));
      break;
    }
    case MSP_STATUS: {
      uint8_t responseData[] =
      {
          0x80, 0x00,
          0x00, 0x00,
          0x23, 0x00,
          (arm ? 0x03 : 0x02), 0x00, 0x00, 0x00,
          0x01
      };
      send_message(MSP_STATUS, responseData, sizeof(responseData));
      break;
    }
    case MSP_STATUS_EX: {
      uint8_t responseData[] =
      {
          0x80, 0x00,
          0x00, 0x00,
          0x23, 0x00,
          (arm ? 0x03 : 0x02), 0x00, 0x00, 0x00,
          0x01,
          0x0A,
          0x03, 0x03,
          0x00, 0x00
      };
      send_message(MSP_STATUS_EX, responseData, sizeof(responseData));
      break;
    }
    case MSP_BATTERY_STATE: {
      uint8_t legacyBatteryVoltage = static_cast<uint8_t>(battery_state.BusVoltage);
      uint16_t amperage = static_cast<uint16_t>(battery_state.Current_mA);
      uint8_t batteryState = static_cast<uint8_t>(
        ((battery_state.BusVoltagePerCell - battery_state.Min_voltage_per_cell) / (battery_state.Max_voltage_per_cell - battery_state.Min_voltage_per_cell)) * 100
      );
      uint16_t drawn_mAH = static_cast<uint16_t>(battery_state.Drawn_mAH);
      uint8_t responseData[] =
      {
          battery_state.Cells,
          (uint8_t)(battery_state.Capacity & 0xFF), (uint8_t)(battery_state.Capacity >> 8),
          legacyBatteryVoltage,
          (uint8_t)(drawn_mAH & 0xFF), (uint8_t)(drawn_mAH >> 8),
          (uint8_t)(amperage & 0xFF), (uint8_t)(amperage >> 8),
          batteryState,
          legacyBatteryVoltage * 10
      };
      send_message(MSP_BATTERY_STATE, responseData, sizeof(responseData));
      break;
    }
    case MSP_COMP_GPS: {
      uint8_t responseData[] =
      {
        (uint8_t)(sensor_distance & 0xFF), (uint8_t)(sensor_distance >> 8),
        0x00, 0x00,
        (++gps_state & 1)
      };
      send_message(MSP_COMP_GPS, responseData, sizeof(responseData));
      break;
    }
    case MSP_OSD_CONFIG: {
      object_to_byte_array(osd_config, response_buff, response_length);
      send_message(MSP_OSD_CONFIG, response_buff, response_length);
      break;
    }
  }
  log("[OSD] MSP COMMAND: " + String(command));
}

template <typename T>
void OSD::object_to_byte_array(const T &obj, uint8_t *byteArray, size_t &arrayLength) {
  const uint8_t *ptr = reinterpret_cast<const uint8_t*>(&obj);
  arrayLength = sizeof(T);
  memcpy(byteArray, ptr, arrayLength);
}

uint8_t OSD::calculate_checksum(uint8_t command, uint8_t* data, uint8_t dataLength) {
  uint8_t checksum = 0;
  checksum ^= dataLength;
  checksum ^= command;
  for (uint8_t i = 0; i < dataLength; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

void OSD::send_message(uint8_t command, uint8_t* data, uint8_t dataLength) {

  const char header[] = "$M<";
  uint8_t checksum = calculate_checksum(command, data, dataLength);

  uart->write(header, 3);
  uart->write(dataLength);
  uart->write(command);

  for (uint8_t i = 0; i < dataLength; i++) {
    uart->write(data[i]);
  }

  uart->write(checksum);
}

void OSD::set_arm(bool state) {
  arm = state;
}

void OSD::set_battery_state(INA219State state) {
  battery_state = state;
}

void OSD::set_sensor_distance(uint16_t distance) {
    sensor_distance = distance;
}

void OSD::log(String val, bool line) {
  if (logger != nullptr) { if (line) logger->println(val); else logger->print(val); }
}