// serial_driver_ch343/include/serial_driver_ch343/cvmode.hpp
#pragma once

#include "rclcpp/rclcpp.hpp"

namespace auto_aim_interfaces
{
namespace msg
{

struct Cvmode
{
    float bullet_speed;
    uint8_t cur_cv_mode;
    
    // 默认构造函数
    Cvmode()
    : bullet_speed(0.0f)
    , cur_cv_mode(0)
    {
    }
};

}  // namespace msg
}  // namespace auto_aim_interfaces