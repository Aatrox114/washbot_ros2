#include <iostream>
#include <string>
#include <unistd.h>
#include "unitree_lidar_sdk.h"

using namespace unilidar_sdk2;

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Usage: lidar_control_udp [stop|start|reset]" << std::endl;
        return 1;
    }

    std::string cmd = argv[1];

    std::string lidar_ip = "192.168.1.62";
    std::string local_ip = "192.168.1.2";
    unsigned short lidar_port = 6101;
    unsigned short local_port = 6201;

    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    int ret = lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip);
    if (ret != 0) {
        std::cerr << "Unilidar initialization failed!" << std::endl;
        return -1;
    }

    std::cout << "Unilidar initialization succeed." << std::endl;

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
        std::cerr << "Usage: lidar_control_udp [stop|start|reset]" << std::endl;
        lreader->closeUDP();
        return 1;
    }

    sleep(1);
    lreader->closeUDP();

    std::cout << "Done." << std::endl;
    return 0;
}
