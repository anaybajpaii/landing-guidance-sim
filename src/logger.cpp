#include "logger.hpp"

namespace {
constexpr double kPi = 3.14159265358979323846;
}

DataLogger::DataLogger(const std::string& filename) : file_(filename) {
    if (file_.is_open()) {
        file_ << "t,x,y,vx,vy,fuel,mass,thrust,tilt_deg\n";
    }
}

bool DataLogger::isOpen() const {
    return file_.is_open();
}

void DataLogger::log(const LanderState& s, double mass) {
    file_ << s.t << ',' << s.x << ',' << s.y << ','
          << s.vx << ',' << s.vy << ',' << s.fuel << ','
          << mass << ',' << s.thrust << ',' << s.tilt * 180.0 / kPi << '\n';
}