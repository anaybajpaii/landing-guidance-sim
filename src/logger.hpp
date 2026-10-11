#pragma once
#include <fstream>
#include <string>
#include "lander.hpp"

class DataLogger {
public:
    explicit DataLogger(const std::string& filename);

    bool isOpen() const;
    void log(const LanderState& s, double mass);

private:
    std::ofstream file_;
};