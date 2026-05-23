#include <iostream>
#include <string>
#include <cstdlib>
#include <unistd.h>
#include "unitree_lidar_sdk.h"

using namespace unilidar_sdk2;

static void print_mode(uint32_t mode)
{
    std::cout << "mode = " << mode << std::endl;
    std::cout << "bit0 FOV        : " << ((mode & 0x01) ? "wide 192 deg" : "standard 180 deg") << std::endl;
    std::cout << "bit1 measure    : " << ((mode & 0x02) ? "2D" : "3D") << std::endl;
    std::cout << "bit2 IMU        : " << ((mode & 0x04) ? "disabled" : "enabled") << std::endl;
    std::cout << "bit3 connection : " << ((mode & 0x08) ? "serial" : "udp/ethernet") << std::endl;
    std::cout << "bit4 boot       : " << ((mode & 0x10) ? "wait start command" : "auto start") << std::endl;
}

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cout << "Usage: lidar_set_mode_udp <mode>" << std::endl;
        std::cout << "Examples:" << std::endl;
        std::cout << "  lidar_set_mode_udp 0   # standard FOV + 3D + IMU on + UDP + auto start" << std::endl;
        std::cout << "  lidar_set_mode_udp 1   # wide FOV + 3D + IMU on + UDP + auto start" << std::endl;
        std::cout << "  lidar_set_mode_udp 2   # standard FOV + 2D + IMU on + UDP + auto start" << std::endl;
        std::cout << "  lidar_set_mode_udp 16  # standard FOV + 3D + IMU on + UDP + wait start" << std::endl;
        return 1;
    }

    uint32_t mode = static_cast<uint32_t>(std::stoul(argv[1]));

    std::string lidar_ip = "192.168.1.62";
    std::string local_ip = "192.168.1.2";
    unsigned short lidar_port = 6101;
    unsigned short local_port = 6201;

    print_mode(mode);

    UnitreeLidarReader *lreader = createUnitreeLidarReader();

    int ret = lreader->initializeUDP(lidar_port, lidar_ip, local_port, local_ip);
    if (ret != 0) {
        std::cerr << "Unilidar initialization failed!" << std::endl;
        return -1;
    }

    std::cout << "Unilidar initialization succeed." << std::endl;
    std::cout << "Setting lidar work mode..." << std::endl;

    lreader->setLidarWorkMode(mode);

    sleep(1);

    lreader->closeUDP();

    std::cout << "Done. Please power-cycle the lidar if the mode change requires reboot." << std::endl;
    return 0;
}
