#ifndef PIGEON_IMU_PUBLISHER__ROBOT_IMU_HPP_
#define PIGEON_IMU_PUBLISHER__ROBOT_IMU_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <ctre/phoenix6/Pigeon2.hpp>
#include <memory>

class robot_imu : public rclcpp::Node
{
public:
    robot_imu();

private:
    void tick();
    
    std::unique_ptr<ctre::phoenix6::hardware::Pigeon2> pigeon_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif  // PIGEON_IMU_PUBLISHER__ROBOT_IMU_HPP_