#ifndef SC08A_h
#define SC08A_h

#include <Arduino.h>
#include <algorithm>
#include <vector>

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
    SC08A(HardwareSerial& serial = Serial1);
    ~SC08A();
    void SetChannels(const std::vector<unsigned char>& newChannels);
    void SetLogger(HardwareSerial& logger);
    void SetInputRange(int min, int max);
    void Init();
    void Set(const std::vector<unsigned char>& channels, int position, unsigned char velocity = 100);
    void Home();
private:
    HardwareSerial* uart;
    HardwareSerial* logger;
    std::vector<unsigned char> channels;
    Positions positions;
    SignalRange signalRange;
    void activate_choosen_servo_channels();
    bool contains(const std::vector<unsigned char>& vec, unsigned char value);
    String convertVectorToString(const std::vector<unsigned char>& vec);
    void log(String val, bool line = true);
};

#endif