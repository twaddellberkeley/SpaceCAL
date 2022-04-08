#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bno055_imu_pub/example.hpp"
#include "bno055_imu_pub/bno055_driver.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ImuPublisher : public rclcpp::Node
{
public:
  //
  // string::string topic_name = "imu_topic";

  ImuPublisher()
      : Node("imu_publisher"), count_(0)
  {
    imu_fusion_ = bno055_imu::BNO055Driver("/dev/i2c-1", 0x28, bno055_imu::OPERATION_MODE_IMUPLUS);
    imu_raw_ = bno055_imu::BNO055Driver("/dev/i2c-1", 0x29, bno055_imu::OPERATION_MODE_ACCGYRO);
    imu_fusion_.init();
    imu_raw_.init();

    fusion_publisher_ = this->create_publisher<std_msgs::msg::String>("fusion_imu_topic", 10);
    raw_publisher_ = this->create_publisher<std_msgs::msg::String>("raw_imu_topic", 10);
    fusion_timer_ = this->create_wall_timer(
        500ms, std::bind(&ImuPublisher::fusion_callback, this));
    raw_timer_ = this->create_wall_timer(
        500ms, std::bind(&ImuPublisher::raw_callback, this));
  }

private:
  void fusion_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Fusion data! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    RCLCPP_INFO(this->get_logger(), "imu address: '%x' ", BNO055_ADDRESS_A);
    fusion_publisher_->publish(message);
  }

  void raw_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Raw data! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    RCLCPP_INFO(this->get_logger(), "imu address: '%x' ", BNO055_ADDRESS_DEFAULT);
    raw_publisher_->publish(message);
  }

  bno055_imu::BNO055Driver imu_fusion_;
  bno055_imu::BNO055Driver imu_raw_;
  rclcpp::TimerBase::SharedPtr fusion_timer_;
  rclcpp::TimerBase::SharedPtr raw_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fusion_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr raw_publisher_;
  size_t count_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}