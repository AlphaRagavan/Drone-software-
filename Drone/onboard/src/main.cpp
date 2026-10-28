#include "autonomy/mission_state_machine.hpp"
#include <iostream>
#include <csignal>
#include <chrono>
#include <thread>
#include <string>

static nidar::autonomy::MissionStateMachine* g_state_machine = nullptr;

void signalHandler(int signum) {
    std::cout << "\n[MAIN] Caught signal " << signum << ". Triggering emergency safe stop..." << std::endl;
    if (g_state_machine) {
        g_state_machine->triggerEmergencyAbort("OS Signal Termination (SIGINT/SIGTERM)");
    }
}

int main(int argc, char* argv[]) {
    std::string lidar_port = "/dev/ttyUSB0";
    std::string model_path = "models/yolov8n.onnx";
    std::string fc_port = "/dev/ttyS0";

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--lidar" && i + 1 < argc) lidar_port = argv[++i];
        else if (arg == "--model" && i + 1 < argc) model_path = argv[++i];
        else if (arg == "--fc" && i + 1 < argc) fc_port = argv[++i];
        else if (arg == "--help") {
            std::cout << "NIDAR Autonomous AirMouse Companion Computer System\n"
                      << "Usage: ./nidar_onboard [options]\n"
                      << "  --lidar <port>   RPLIDAR serial port (default: /dev/ttyUSB0)\n"
                      << "  --model <path>   YOLOv8n ONNX model path (default: models/yolov8n.onnx)\n"
                      << "  --fc <port>      Flight controller serial port (default: /dev/ttyS0)\n";
            return 0;
        }
    }

    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    nidar::autonomy::MissionStateMachine state_machine;
    g_state_machine = &state_machine;

    if (!state_machine.initialize(lidar_port, model_path, fc_port)) {
        std::cerr << "[MAIN] Fatal error during state machine initialization." << std::endl;
        return 1;
    }

    std::cout << "[MAIN] Commencing 20 Hz Real-Time Autonomy Loop..." << std::endl;

    constexpr auto LOOP_PERIOD = std::chrono::milliseconds(50); // 20 Hz
    auto next_tick = std::chrono::steady_clock::now();

    while (state_machine.isRunning()) {
        state_machine.step();

        next_tick += LOOP_PERIOD;
        std::this_thread::sleep_until(next_tick);
    }

    std::cout << "[MAIN] Mission loop terminated. Final State: " 
              << state_machine.getStateName() << std::endl;
    return 0;
}
