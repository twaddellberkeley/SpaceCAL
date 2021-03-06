#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "bno055_imu_pub/bno055_driver.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class ImuPublisher : public rclcpp::Node
{
public:
  ImuPublisher()
      : Node("imu_publisher"),
        imu_fusion_("/dev/i2c-0", 0x28, bno055_imu::OPERATION_MODE_IMUPLUS),
        imu_raw_("/dev/i2c-0", 0x29, bno055_imu::OPERATION_MODE_ACCGYRO),
        count_(0)
  {
    imu_fusion_.init();
    imu_raw_.init();

    fusion_publisher_ = this->create_publisher<interfaces::msg::FusionImu>("fusion_imu_topic", 10);
    raw_publisher_ = this->create_publisher<interfaces::msg::RawImu>("raw_imu_topic", 10);
    fusion_timer_ = this->create_wall_timer(
        500ms, std::bind(&ImuPublisher::fusion_callback, this));
    raw_timer_ = this->create_wall_timer(
        500ms, std::bind(&ImuPublisher::raw_callback, this));
  }
  bno055_imu::BNO055Driver imu_fusion_;
  bno055_imu::BNO055Driver imu_raw_;

private:
  rclcpp::TimerBase::SharedPtr fusion_timer_;
  rclcpp::TimerBase::SharedPtr raw_timer_;
  rclcpp::Publisher<interfaces::msg::FusionImu>::SharedPtr fusion_publisher_;
  rclcpp::Publisher<interfaces::msg::RawImu>::SharedPtr raw_publisher_;
  size_t count_;

  void fusion_callback()
  {
    auto message = std_msgs::msg::String();
    auto fusion_data = interfaces::msg::FusionImu();
    message.data = "Fusion data! " + std::to_string(count_++);

    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    imu_fusion_.read_imu_data(fusion_data);
    fusion_data.header = std_msgs::msg::Header();
    std::cout << "fusion data: " << fusion_data.gravity_magnitude << std::endl;
    RCLCPP_INFO(this->get_logger(), "imu address: '%x' ", BNO055_ADDRESS_A);
    fusion_publisher_->publish(fusion_data);
  }

  void raw_callback()
  {
    auto message = std_msgs::msg::String();
    auto raw_data = interfaces::msg::RawImu();
    message.data = "Raw data! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    imu_raw_.read_imu_data_raw(raw_data);
    raw_data.header = std_msgs::msg::Header();
    RCLCPP_INFO(this->get_logger(), "imu address: '%x' ", BNO055_ADDRESS_DEFAULT);
    raw_publisher_->publish(raw_data);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}