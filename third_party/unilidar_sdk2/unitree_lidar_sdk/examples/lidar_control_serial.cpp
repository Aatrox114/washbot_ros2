#include <iostream>
#include <string>
#include <unistd.h>
#include "unitree_lidar_sdk.h"

using namespace unilidar_sdk2;

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Usage: lidar_control_serial [stop|start|reset] [serial_port] [baudrate]" << std::endl;
        std::cout << "Example:" << std::endl;
        std::cout << "  lidar_control_serial stop" << std::endl;
        std::cout << "  lidar_control_serial start /dev/ttyACM0 4000000" << std::endl;
        std::cout << "  lidar_control_serial reset /dev/ttyACM0 4000000" << std::endl;
        return 1;
    }

    std::string cmd = argv[1];

    std::string port = "/dev/ttyACM0";
    uint32_t baudrate = 4000000;

    if (argc >= 3) {
        port = argv[2];
    }

    if (argc >= 4) {
        baudrate = static_cast<uint32_t>(std::stoul(argv[3]));
    }

    std::cout << "Serial port: " << port << std::endl;
    std::cout << "Baudrate   : " << baudrate << std::endl;

    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    int ret = lreader->initializeSerial(port, baudrate);
    if (ret != 0) {
        std::cerr << "Unilidar serial initialization failed!" << std::endl;
        std::cerr << "Please check:" << std::endl;
        std::cerr << "  1. Lidar is already in serial mode, work_mode = 8" << std::endl;
        std::cerr << "  2. Serial device exists, e.g. /dev/ttyACM0" << std::endl;
        std::cerr << "  3. Permission is OK, e.g. sudo chmod 666 /dev/ttyACM0" << std::endl;
        std::cerr << "  4. Baudrate is correct, usually 4000000" << std::endl;
        return -1;
    }

    std::cout << "Unilidar serial initialization succeed." << std::endl;

    // 串口模式下建议明确设置一次 work_mode = 8
    uint32_t work_mode = 8;
    std::cout << "Set lidar work mode to serial mode: " << work_mode << std::endl;
    lreader->setLidarWorkMode(work_mode);
    sleep(1);

    if (cmd == "stop") {
        std::cout << "Stopping lidar rotation..." << std::endl;
        lreader->stopLidarRotation();
    } else if (cmd == "start") {
        std::cout << "Starting lidar rotation..." << std::endl;
        lreader->startLidarRotation();
    } else if (cmd == "reset") {
        std::cout << "Resetting lidar..." << std::endl;
        lreader->resetLidar();
    } else {
        std::cerr << "Unknown command: " << cmd << std::endl;
        std::cerr << "Usage: lidar_control_serial [stop|start|reset] [serial_port] [baudrate]" << std::endl;
        lreader->closeSerial();
        return 1;
    }

    sleep(1);
    lreader->closeSerial();

    std::cout << "Done." << std::endl;
    return 0;
}
