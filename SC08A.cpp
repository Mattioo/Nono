#include "SC08A.h"

// SC08A Manual: https://drive.google.com/file/d/0BzFWfMiqqjyqem1oQnlXUk4yaXc/view?resourcekey=0-jyKQAxRWIkpJ6Szne90Y1g

SC08A::SC08A(HardwareSerial& serial) {
    this->uart = &serial;
    channels = {1, 2, 3, 4, 5, 6, 7, 8};
}

SC08A::~SC08A() {
    channels.clear();
    activate_choosen_servo_channels();
}

void SC08A::set_logger(HardwareSerial* serial) {
    logger = serial;
}

void SC08A::set_channels(const std::vector<unsigned char>& channelsToActivate) {
    channels = channelsToActivate;
}

void SC08A::set_input_range(int min, int max) {
  if (min < max) {
    signalRange.Minimum = min;
    signalRange.Maximum = max;
  }
}

void SC08A::init() {
    uart->begin(9600);
    activate_choosen_servo_channels();
}

void SC08A::set(const std::vector<unsigned char>& channelsToSet, int position, unsigned char velocity) { 
    int position_before_scale = position;

    if (signalRange.Minimum != 0 || signalRange.Maximum != 0) {
      position = static_cast<int>(
          ((static_cast<double>(position) - signalRange.Minimum) / (signalRange.Maximum - signalRange.Minimum)) * (positions.Maximum - positions.Minimum) + positions.Minimum
      );
    }

    log("[SC08A] Channels: " + convert_vector_to_string(channelsToSet) + " Position: " + position_before_scale + "(" + position + ") " + "Velocity: " + velocity);    

    if (std::all_of(channelsToSet.begin(), channelsToSet.end(), [&](unsigned char channel) { return contains(channels, channel); }) && position >= positions.Minimum && position <= positions.Maximum) {
        for (size_t i = 0; i < channelsToSet.size(); i++) {
        /* 2 BYTES COMBINED  INTO 13-BIT DATA: (HOME) 0 - 8000 (360*) */
        
        // BITS 12-6 (0b0XXXXXXX)
        unsigned char high_byte = ((position >> 6) & 0x7F);
        // BITS 5-0 (0b00XXXXXX)
        unsigned char low_byte = (position & 0x3F);

        uart->write(0b11100000 | channelsToSet[i]);
        uart->write(high_byte);
        uart->write(low_byte);
        uart->write(velocity);
      }
    }
}

void SC08A::set_inverted(const std::vector<unsigned char>& channelsToSet, int position, unsigned char velocity) {
  SC08A::set(channelsToSet, CRSF_CHANNEL_VALUE_MIN + CRSF_CHANNEL_VALUE_MAX - position, velocity);
}

void SC08A::home() {
    log("[SC08A] Home ", false);
    if (signalRange.Minimum != 0 || signalRange.Maximum != 0)
      set(channels, (signalRange.Minimum + (signalRange.Maximum - signalRange.Minimum) / 2));
    else
      set(channels, positions.Home);
}

bool SC08A::is_possible_movement_Y(int Camera_Y) {
  return Camera_Y >= CRSF_CHANNEL_VALUE_MID && Camera_Y <= (CRSF_CHANNEL_VALUE_MID + (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID)/2);
}

bool SC08A::is_possible_movement_X(int Camera_X) {
  return Camera_X >= (CRSF_CHANNEL_VALUE_MIN + (CRSF_CHANNEL_VALUE_MID - CRSF_CHANNEL_VALUE_MIN)/2) && Camera_X <= (CRSF_CHANNEL_VALUE_MID + (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MID)/2);
}

void SC08A::activate_choosen_servo_channels() {

    unsigned char onByte = 0;

    uart->write(0b11000000); // ALL CHANNELS
    uart->write(onByte); // LAST BIT = 0 => OFF

    onByte = 1;
    
    for (size_t i = 0; i < channels.size(); i++) {
      uart->write(0b11000000 | channels[i]); // LAST 5 BITS = CHANNEL
      uart->write(onByte); // LAST BIT = 1 => ON
    }
}

bool SC08A::contains(const std::vector<unsigned char>& vec, unsigned char value) {
    return std::find(vec.begin(), vec.end(), value) != vec.end();
}

String SC08A::convert_vector_to_string(const std::vector<unsigned char>& vec) {
  String result = "";
  for (size_t i = 0; i < vec.size(); ++i) { result += String(vec[i]); if (i < vec.size() - 1) { result += ","; } }
  return result;
}

void SC08A::log(String val, bool line) {
  if (logger != nullptr) { if (line) logger->println(val); else logger->print(val); }
}