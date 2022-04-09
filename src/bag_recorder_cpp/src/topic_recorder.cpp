#include <iostream>
#include <ctime>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <interfaces/msg/fusion_imu.hpp>
#include <interfaces/msg/raw_imu.hpp>
#include <interfaces/msg/motor_data.hpp>
#include <interfaces/msg/display_data.hpp>

#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

using std::placeholders::_1;

class BagRecorder : public rclcpp::Node
{
public:
  BagRecorder()
      : Node("bag_recorder")
  {
    writer_ = std::make_unique<rosbag2_cpp::Writer>();
    time_t ttime = time(0);
    // tm *local_time = localtime(&ttime);
    std::string data = ctime(&ttime);
    std::string time = "my_bag/" + data;

    writer_->open(time);

    subscription_ = create_subscription<std_msgs::msg::String>(
        "buttons_topic", 10, std::bind(&BagRecorder::topic_callback, this, _1));
  }

private:
  void topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    rclcpp::Time time_stamp = this->now();

    writer_->write(*msg, "buttons_topic", "std_msgs/msg/String", time_stamp);
  }

  rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr subscription_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagRecorder>());
  rclcpp::shutdown();
  return 0;
}