#include "rclcpp/rclcpp.hpp"
#include "dm_imu/imu_driver.h"
#include <memory>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    try
    {
        auto imu_node = std::make_shared<dmbot_serial::DmImu>();
        rclcpp::spin(imu_node);
        rclcpp::shutdown();
        return 0;
    }
    catch (const std::exception &ex)
    {
        RCLCPP_FATAL(rclcpp::get_logger("dm_imu_node"), "%s", ex.what());
        rclcpp::shutdown();
        return 1;
    }
}
