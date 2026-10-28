#pragma once

#include "navigation/kinematics_controller.hpp"
#include <string>
#include <atomic>

namespace nidar {
namespace hal {

enum FlightMode {
    MODE_DISARMED   = 0,
    MODE_ARMED      = 1,
    MODE_TAKEOFF    = 2,
    MODE_OFFBOARD   = 3,
    MODE_LANDING    = 4,
    MODE_ABORTED    = 5
};

class FlightControllerInterface {
public:
    FlightControllerInterface(const std::string& serial_device = "/dev/ttyS0", uint32_t baudrate = 921600);
    ~FlightControllerInterface();

    bool initialize();
    bool arm();
    bool disarm();
    bool takeoff(float altitude_m = 1.0f);
    bool land();
    bool sendVelocity(const navigation::TwistCommand& cmd);
    void emergencyKillSwitch();

    FlightMode getMode() const { return current_mode_.load(); }
    float getBatteryVoltage() const { return battery_voltage_.load(); }
    bool isArmed() const { return current_mode_.load() != MODE_DISARMED; }

private:
    std::string serial_dev_;
    uint32_t baudrate_;
    std::atomic<FlightMode> current_mode_{MODE_DISARMED};
    std::atomic<float> battery_voltage_{15.8f}; // Simulated 4S LiPo (nominal ~16.0V)
    std::atomic<bool> hardware_connected_{false};
};

} // namespace hal
} // namespace nidar
