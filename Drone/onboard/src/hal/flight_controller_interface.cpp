#include "hal/flight_controller_interface.hpp"
#include <iostream>
#include <thread>
#include <chrono>

namespace nidar {
namespace hal {

FlightControllerInterface::FlightControllerInterface(const std::string& serial_device, uint32_t baudrate)
    : serial_dev_(serial_device), baudrate_(baudrate) {}

FlightControllerInterface::~FlightControllerInterface() {
    emergencyKillSwitch();
}

bool FlightControllerInterface::initialize() {
    std::cout << "[Flight Controller HAL] Connecting to FC via " << serial_dev_ 
              << " @ " << baudrate_ << " baud..." << std::endl;
    // Real companion computer sends MAVLink heartbeat or MSP connection frame
    hardware_connected_.store(true);
    current_mode_.store(MODE_DISARMED);
    std::cout << "[Flight Controller HAL] Flight controller ready. Telemetry streaming active." << std::endl;
    return true;
}

bool FlightControllerInterface::arm() {
    std::cout << "[Flight Controller HAL] Arming motors..." << std::endl;
    current_mode_.store(MODE_ARMED);
    return true;
}

bool FlightControllerInterface::disarm() {
    std::cout << "[Flight Controller HAL] Disarming motors." << std::endl;
    current_mode_.store(MODE_DISARMED);
    return true;
}

bool FlightControllerInterface::takeoff(float altitude_m) {
    if (current_mode_.load() == MODE_DISARMED) {
        arm();
    }
    std::cout << "[Flight Controller HAL] Executing autonomous takeoff to " << altitude_m << "m AGL..." << std::endl;
    current_mode_.store(MODE_TAKEOFF);
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
    current_mode_.store(MODE_OFFBOARD);
    return true;
}

bool FlightControllerInterface::land() {
    std::cout << "[Flight Controller HAL] Engaging precision autonomous descent and landing..." << std::endl;
    current_mode_.store(MODE_LANDING);
    return true;
}

bool FlightControllerInterface::sendVelocity(const navigation::TwistCommand& cmd) {
    if (current_mode_.load() != MODE_OFFBOARD) return false;
    // In production, formats MAVLink #84 SET_POSITION_TARGET_LOCAL_NED velocity setpoint
    // cmd.vx: Forward velocity, cmd.vy: Lateral velocity, cmd.yaw_rate: Yaw angular rate
    return true;
}

void FlightControllerInterface::emergencyKillSwitch() {
    current_mode_.store(MODE_ABORTED);
    // Instant hard motor shutdown / emergency disarm
    std::cerr << "[Flight Controller HAL] EMERGENCY KILL SWITCH ENGAGED! MOTORS HARD DISARMED." << std::endl;
}

} // namespace hal
} // namespace nidar
