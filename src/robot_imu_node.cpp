#include "robot_imu_node.hpp"

robot_imu::robot_imu() : Node("robot_imu")
{
    this->declare_parameter("can_id", 0);
    this->declare_parameter("canbus", "canivore");
    this->declare_parameter("publish_rate", 100.0);
    
    int can_id = this->get_parameter("can_id").as_int();
    std::string canbus = this->get_parameter("canbus").as_string();
    double rate = this->get_parameter("publish_rate").as_double();
    
    // init the pigeon2
    pigeon_ = std::make_unique<ctre::phoenix6::hardware::Pigeon2>(can_id, canbus);
    
    publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    
    auto period = std::chrono::duration<double>(1.0 / rate);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&robot_imu::tick, this));
    
    RCLCPP_INFO(this->get_logger(), "Robot IMU Publisher started on /imu topic");
}

void robot_imu::tick()
{
    auto msg = sensor_msgs::msg::Imu();
    
    auto yaw = pigeon_->GetYaw();
    auto pitch = pigeon_->GetPitch();
    auto roll = pigeon_->GetRoll();
    
    auto angular_vel = pigeon_->GetAngularVelocityZWorld();
    auto angular_vel_y = pigeon_->GetAngularVelocityYWorld();
    auto angular_vel_x = pigeon_->GetAngularVelocityXWorld();
    
    auto accel_x = pigeon_->GetAccelerationX();
    auto accel_y = pigeon_->GetAccelerationY();
    auto accel_z = pigeon_->GetAccelerationZ();
    
    msg.header.stamp = this->now();
    msg.header.frame_id = "imu_link";

    // per rep-0145 https://www.ros.org/reps/rep-0145.html  , put missing covariance as -1
    msg.orientation_covariance[0] = -1;
    
    msg.angular_velocity.x = angular_vel_x.GetValue().value();
    msg.angular_velocity.y = angular_vel_y.GetValue().value();
    msg.angular_velocity.z = angular_vel.GetValue().value();
    
    msg.linear_acceleration.x = accel_x.GetValue().value();
    msg.linear_acceleration.y = accel_y.GetValue().value();
    msg.linear_acceleration.z = accel_z.GetValue().value();
    
    publisher_->publish(msg);
}