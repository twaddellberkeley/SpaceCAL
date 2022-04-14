#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <interfaces/msg/motor_data.hpp>
#include <interfaces/msg/display_data.hpp>
#include <interfaces/msg/fusion_imu.hpp>
#include <interfaces/msg/raw_imu.hpp>
#include <ctime>

#include <iostream>
#include <fstream>
#include <cstdlib>
#include <filesystem>

#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#define TYPE_STRING std_msgs::msg::String
#define TYPE_MOTOR_DATA interfaces::msg::MotorData
#define TYPE_DISPLAY_DATA interfaces::msg::DisplayData
#define TYPE_FUSION_IMU interfaces::msg::FusionImu
#define TYPE_RAW_IMU interfaces::msg::RawImu
#define BUTTONS "buttons_topic"
#define DISPLAY "display_topic"
#define FUSION_IMU "fusion_imu_topic"
#define RAW_IMU "raw_imu_topic"
#define MOTOR "motor_data"

using std::placeholders::_1;

class BagRecorder : public rclcpp::Node
{
public:
  BagRecorder()
      : Node("bag_recorder")
  {
    const rosbag2_cpp::StorageOptions storage_options({make_then_get_dir(), "sqlite3"});
    const rosbag2_cpp::ConverterOptions converter_options(
        {rmw_get_serialization_format(),
         rmw_get_serialization_format()});
    writer_ = std::make_unique<rosbag2_cpp::writers::SequentialWriter>();

    writer_->open(storage_options, converter_options);

    writer_->create_topic(
        {BUTTONS,
         "std_msgs/msg/String",
         rmw_get_serialization_format(),
         ""});
    buttons_sub_ = create_subscription<std_msgs::msg::String>(
        BUTTONS, 10, std::bind(&BagRecorder::buttons_topic_callback, this, _1));

    writer_->create_topic(
        {DISPLAY,
         "interfaces/msg/DisplayData",
         rmw_get_serialization_format(),
         ""});
    display_sub_ = create_subscription<interfaces::msg::DisplayData>(
        DISPLAY, 10, std::bind(&BagRecorder::display_topic_callback, this, _1));

    writer_->create_topic(
        {FUSION_IMU,
         "interfaces/msg/FusionImu",
         rmw_get_serialization_format(),
         ""});
    fusion_imu_sub_ = create_subscription<interfaces::msg::FusionImu>(
        FUSION_IMU, 10, std::bind(&BagRecorder::fusion_imu_topic_callback, this, _1));

    writer_->create_topic(
        {RAW_IMU,
         "interfaces/msg/RawImu",
         rmw_get_serialization_format(),
         ""});
    raw_imu_sub_ = create_subscription<interfaces::msg::RawImu>(
        RAW_IMU, 10, std::bind(&BagRecorder::raw_imu_topic_callback, this, _1));

    writer_->create_topic(
        {MOTOR,
         "interfaces/msg/MotorData",
         rmw_get_serialization_format(),
         ""});
    motor_sub_ = create_subscription<interfaces::msg::MotorData>(
        MOTOR, 10, std::bind(&BagRecorder::motor_topic_callback, this, _1));
  }

private:
  void buttons_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    callback_writer_helper(msg, BUTTONS);
  }
  void display_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    callback_writer_helper(msg, DISPLAY);
  }
  void fusion_imu_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    callback_writer_helper(msg, FUSION_IMU);
  }
  void raw_imu_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    callback_writer_helper(msg, RAW_IMU);
  }
  void motor_topic_callback(std::shared_ptr<rclcpp::SerializedMessage> msg) const
  {
    callback_writer_helper(msg, MOTOR);
  }

  void callback_writer_helper(std::shared_ptr<rclcpp::SerializedMessage> msg, std::string topic_name) const
  {
    auto bag_message = std::make_shared<rosbag2_storage::SerializedBagMessage>();

    bag_message->serialized_data = std::shared_ptr<rcutils_uint8_array_t>(
        new rcutils_uint8_array_t,
        [this](rcutils_uint8_array_t *msg)
        {
          auto fini_return = rcutils_uint8_array_fini(msg);
          delete msg;
          if (fini_return != RCUTILS_RET_OK)
          {
            RCLCPP_ERROR(get_logger(),
                         "Failed to destroy serialized message %s", rcutils_get_error_string().str);
          }
        });
    *bag_message->serialized_data = msg->release_rcl_serialized_message();

    bag_message->topic_name = topic_name;
    if (rcutils_system_time_now(&bag_message->time_stamp) != RCUTILS_RET_OK)
    {
      RCLCPP_ERROR(get_logger(), "Error getting current time: %s",
                   rcutils_get_error_string().str);
    }

    writer_->write(bag_message);
  }

  std::string make_then_get_dir(void)
  {
    namespace fs = std::filesystem;
    using namespace std;

    time_t now = time(0);
    tm *ltm = localtime(&now);
    string year = to_string(1900 + ltm->tm_year);
    string month = to_string(1 + ltm->tm_mon);
    string day = to_string(ltm->tm_mday);
    string date = get_month_name(1 + ltm->tm_mon) + "-" + day + "-" + year;
    string ttime = to_string(ltm->tm_hour) + ":" + to_string(ltm->tm_min) + ":" + to_string(ltm->tm_sec);
    string folder = "my_bag/" + date + "/" + ttime;
    fs::create_directories(folder);
    cout << "path: " << folder << endl;
    return folder;
  }

  std::string get_month_name(int month_num)
  {
    switch (month_num)
    {
    case 1:
      return "Jan";
    case 2:
      return "Feb";
    case 3:
      return "March";
    case 4:
      return "Apr";
    case 5:
      return "May";
    case 6:
      return "Jun";
    case 7:
      return "Jul";
    case 8:
      return "Aug";
    case 9:
      return "Sep";
    case 10:
      return "Oct";
    case 11:
      return "Nov";
    case 12:
      return "Dec";
    // you can have any number of case statements.
    default: // Optional
      return "Error";
    }
  }

  rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr buttons_sub_;
  rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr display_sub_;
  rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr fusion_imu_sub_;
  rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr raw_imu_sub_;
  rclcpp::Subscription<rclcpp::SerializedMessage>::SharedPtr motor_sub_;
  std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagRecorder>());
  rclcpp::shutdown();
  return 0;
}