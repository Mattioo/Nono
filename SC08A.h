#pragma once

#include <Arduino.h>
#include <algorithm>
#include <vector>
#include "CrsfSerial.h"

struct Positions {
    Positions() : Minimum(0), Maximum(8000), Home(4000) {}

    int Minimum;
    int Maximum;
    int Home;
};

struct SignalRange {
    SignalRange() : Minimum(0), Maximum(0) {}

    int Minimum;
    int Maximum;
};

class SC08A {
public:
    SC08A(HardwareSerial& serial);
    ~SC08A();
    void set_logger(HardwareSerial* serial = nullptr);
    void set_channels(const std::vector<unsigned char>& newChannels);    
    void set_input_range(int min, int max);
    void init();

    void set(const std::vector<unsigned char>& channels, int position, unsigned char velocity = 100);
    void set_inverted(const std::vector<unsigned char>& channels, int position, unsigned char velocity = 100);
    void home();
    bool is_possible_movement_Y(int Camera_Y);
    bool is_possible_movement_X(int Camera_X);

private:
    HardwareSerial* uart;
    std::vector<unsigned char> channels;
    Positions positions;
    SignalRange signalRange;

    void activate_choosen_servo_channels();
    bool contains(const std::vector<unsigned char>& vec, unsigned char value);
    String convert_vector_to_string(const std::vector<unsigned char>& vec);

    HardwareSerial* logger;
    void log(String val, bool line = true);
};