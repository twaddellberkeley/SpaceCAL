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

#include <rcpputils/asserts.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>

#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>

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

using rosbag2_cpp::converter_interfaces::SerializationFormatConverter;

struct TypeSupport
{
  std::shared_ptr<rcpputils::SharedLibrary> introspection_type_support_library;
  const rosidl_message_type_support_t *introspection_type_support;
};

struct msg_wrapper_buttons {
  unsigned long int time_stamp;
  std_msgs::msg::String msg;
};
struct msg_wrapper_disp {
  unsigned long int time_stamp;
  interfaces::msg::DisplayData msg;
};
struct msg_wrapper_fusion {
  unsigned long int time_stamp;
  interfaces::msg::FusionImu msg;
};
struct msg_wrapper_raw {
  unsigned long int time_stamp;
  interfaces::msg::RawImu msg;
};
struct msg_wrapper_motor {
  unsigned long int time_stamp;
  interfaces::msg::MotorData msg;
};

class BagReader : public rclcpp::Node
{
  std::vector<std::string> topics_ = {BUTTONS, DISPLAY, FUSION_IMU, RAW_IMU, MOTOR};
  rosbag2_cpp::readers::SequentialReader reader_;
  rosbag2_cpp::StorageOptions storage_options_{};
  rosbag2_cpp::ConverterOptions converter_options_{};
  std::vector<std::string> bag_paths_;
  std::vector<std::string>::iterator file_iterator_{};
  std::unordered_map<std::string, const rosidl_message_type_support_t *> topics_and_types_;
  rosbag2_cpp::SerializationFormatConverterFactory factory_;
  std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer_;
  std::string base_path_;

public:
  std::vector<struct msg_wrapper_buttons> str_msgs_;
  std::vector<struct msg_wrapper_disp> display_data_msgs_;
  std::vector<struct msg_wrapper_fusion> fusion_imu_msgs_;
  std::vector<struct msg_wrapper_raw> raw_imu_msgs_;
  std::vector<struct msg_wrapper_motor> motor_data_msgs_;
  
  typedef struct message_t
  {
    void *message;
    char *topic_name;
    rcutils_time_point_value_t time_stamp;
  } message_t;
  std::vector<message_t> msgs_;

  BagReader(std::string base_path) : Node("Bag_Reader")
  {
    base_path_ = base_path;
    set_paths();
    assert(file_iterator_ != bag_paths_.end());
    storage_options_.uri = *file_iterator_;
    file_iterator_++;
    storage_options_.storage_id = "sqlite3";
    converter_options_.input_serialization_format = rmw_get_serialization_format();
    converter_options_.output_serialization_format = rmw_get_serialization_format();

    cdr_deserializer_ = factory_.load_deserializer(rmw_get_serialization_format());

    reader_.open(storage_options_, converter_options_);

    auto topics = reader_.get_all_topics_and_types();

    // about metadata
    for (auto t : topics)
    {

      std::cout << "meta name: " << t.name << std::endl;
      std::cout << "meta type: " << t.type << std::endl;
      std::cout << "meta serialization_format: " << t.serialization_format << std::endl;
      // if (topics_and_types_.find(t.name) != topics_and_types_.end()) {
      //   auto library = rosbag2_cpp::get_typesupport_library(t.type, "rosidl_typesupport_cpp" );
      //   auto type_support = rosbag2_cpp::get_typesupport_handle(t.type, "rosidl_typesupport_cpp", library);
      //   topics_and_types_.insert(std::make_pair(t.name, rosbag2_cpp::get_typesupport_handle(t.type, "rosidl_typesupport_cpp", library)));
      // }
    }

    get_all_messages();

    printf("hello world bag_reader package\n");
  }

  bool has_next_file()
  {
    return file_iterator_ + 1 != bag_paths_.end();
  }

  void load_next_file()
  {
    assert(file_iterator_ != bag_paths_.end());
    storage_options_.uri = *file_iterator_;
    reader_.open(storage_options_, converter_options_);
    file_iterator_++;
  }

  void read_current_file()
  {

    // if(reader_ == nullptr) {
    //   std::cout<< "[ERROR:read_current_file]: reader is null"<<std::endl;
    // }
    auto library_str = rosbag2_cpp::get_typesupport_library("std_msgs/msg/String", "rosidl_typesupport_cpp" );
    auto type_support_str = rosbag2_cpp::get_typesupport_handle("std_msgs/msg/String", "rosidl_typesupport_cpp", library_str);
    auto library_display = rosbag2_cpp::get_typesupport_library("interfaces/msg/DisplayData", "rosidl_typesupport_cpp" );
    auto type_support_display = rosbag2_cpp::get_typesupport_handle("interfaces/msg/DisplayData", "rosidl_typesupport_cpp", library_display);
    auto library_fusion_imu = rosbag2_cpp::get_typesupport_library("interfaces/msg/FusionImu", "rosidl_typesupport_cpp" );
    auto type_support_fusion_imu = rosbag2_cpp::get_typesupport_handle("interfaces/msg/FusionImu", "rosidl_typesupport_cpp", library_fusion_imu);
    auto library_raw_imu = rosbag2_cpp::get_typesupport_library("interfaces/msg/RawImu", "rosidl_typesupport_cpp" );
    auto type_support_raw_imu = rosbag2_cpp::get_typesupport_handle("interfaces/msg/RawImu", "rosidl_typesupport_cpp", library_raw_imu);
    auto library_motor = rosbag2_cpp::get_typesupport_library("interfaces/msg/MotorData", "rosidl_typesupport_cpp" );
    auto type_support_motor = rosbag2_cpp::get_typesupport_handle("interfaces/msg/MotorData", "rosidl_typesupport_cpp", library_motor);

    auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();

    ros_message->allocator = rcutils_get_default_allocator();

    while (reader_.has_next())
    {
      auto serialized_message = reader_.read_next();
      ros_message->message = nullptr;
      ros_message->time_stamp = serialized_message->time_stamp;

      if (serialized_message->topic_name == BUTTONS) {
        
        std_msgs::msg::String msg; // = std::make_shared<std_msgs::msg::String>();
        ros_message->message = &msg;
        // auto type_support = topics_and_types_[BUTTONS];
        cdr_deserializer_->deserialize(serialized_message, type_support_str, ros_message);
        struct msg_wrapper_buttons wrap_msg;
        wrap_msg.time_stamp = serialized_message->time_stamp;
        wrap_msg.msg = msg;
        str_msgs_.push_back(wrap_msg);
        std::cout << "Buttons time: " << serialized_message->time_stamp << std::endl;
      } 
      else if (serialized_message->topic_name == DISPLAY) {
        
        interfaces::msg::DisplayData msg; // = std::make_shared<std_msgs::msg::String>();
        ros_message->message = &msg;
        // auto type_support = topics_and_types_[DISPLAY];
        cdr_deserializer_->deserialize(serialized_message, type_support_display, ros_message);
        struct msg_wrapper_disp wrap_msg;
        wrap_msg.time_stamp = serialized_message->time_stamp;
        wrap_msg.msg = msg;
        display_data_msgs_.push_back(wrap_msg);
        std::cout << "Display time: " << serialized_message->time_stamp << std::endl;
      }
      else if (serialized_message->topic_name == FUSION_IMU) {
        
        interfaces::msg::FusionImu msg; // = std::make_shared<std_msgs::msg::String>();
        ros_message->message = &msg;
        // auto type_support = topics_and_types_[FUSION_IMU];
        cdr_deserializer_->deserialize(serialized_message, type_support_fusion_imu, ros_message);
        struct msg_wrapper_fusion wrap_msg;
        wrap_msg.time_stamp = serialized_message->time_stamp;
        wrap_msg.msg = msg;
        msg.header.stamp.nanosec = serialized_message->time_stamp;
        fusion_imu_msgs_.push_back(wrap_msg);
        std::cout << "FusionIMU time: " << serialized_message->time_stamp << std::endl;
      }
      else if (serialized_message->topic_name == RAW_IMU) {
        interfaces::msg::RawImu msg; // = std::make_shared<std_msgs::msg::String>();
        ros_message->message = &msg;
        // auto type_support = topics_and_types_[RAW_IMU];
        cdr_deserializer_->deserialize(serialized_message, type_support_raw_imu, ros_message);
        struct msg_wrapper_raw wrap_msg;
        wrap_msg.time_stamp = serialized_message->time_stamp;
        wrap_msg.msg = msg;
        msg.header.stamp.nanosec = serialized_message->time_stamp;
        raw_imu_msgs_.push_back(wrap_msg);
        std::cout << "RawImu time: " << serialized_message->time_stamp << std::endl;
      }
      else if (serialized_message->topic_name == MOTOR) {
        
        interfaces::msg::MotorData msg; // = std::make_shared<std_msgs::msg::String>();
        ros_message->message = &msg;
        // auto type_support = topics_and_types_[MOTOR];
        cdr_deserializer_->deserialize(serialized_message, type_support_motor, ros_message);
        struct msg_wrapper_motor wrap_msg;
        wrap_msg.time_stamp = serialized_message->time_stamp;
        wrap_msg.msg = msg;
        motor_data_msgs_.push_back(wrap_msg);
        std::cout << "Motor time: " << serialized_message->time_stamp << std::endl;
      }
    }
  }

  void set_paths(void)
  {
    std::cout << std::filesystem::current_path() << std::endl;
    // get all the paths
    std::cout << base_path_ << std::endl;

    // Iterate over the `std::filesystem::directory_entry` elements using `auto`
    for (auto const &dir_entry : std::filesystem::recursive_directory_iterator(base_path_))
    {
      if (dir_entry.is_regular_file() && dir_entry.path().extension() == ".db3")
      {

        std::cout << dir_entry.path().filename() << '\n';
        std::cout << dir_entry.path().u8string() << '\n';
        bag_paths_.push_back(dir_entry.path().u8string());
      }
    }
    file_iterator_ = bag_paths_.begin();
  }

  void get_all_messages()
  {
    // if(storage_options_.uri == nullptr) {
    //   std::cout << "ERROR: get_all_messages" << std::endl;
    //   return;
    // }
    read_current_file();

    while (has_next_file())
    {
      load_next_file();
      read_current_file();
    }
    // print_raw_imu();
    save_fusion_imu_file();
    save_raw_imu_file();

  }

  void print_fusion_imu() {
    
      std::cout << "************* Fusion Imu Data: ****************" << std::endl;
      
      for (auto wrap : fusion_imu_msgs_) {
        interfaces::msg::FusionImu msg;
        msg = wrap.msg;
        std::cout << std::endl;
        std::cout << "header: -------------" << std::endl;
        // std::cout << "time stamp: " << msg.header.stamp.nsec << std::endl;
        std::cout << "time stamp: " << wrap.time_stamp << std::endl;
        std::cout << "orientation: -------------" << std::endl;
        std::cout <<  "y: " << msg.orientation.y << std::endl;
        std::cout <<  "x: " << msg.orientation.x << std::endl;
        std::cout <<  "z: " << msg.orientation.z << std::endl;
        std::cout << "euler_angles: -------------" << std::endl;
        std::cout <<  "y: " << msg.euler_angles.y << std::endl;
        std::cout <<  "x: " << msg.euler_angles.x << std::endl;
        std::cout <<  "z: " << msg.euler_angles.z << std::endl;
        std::cout << "angualar_velocity: -------------" << std::endl;
        std::cout << "x: " << msg.angular_velocity.x << std::endl;
        std::cout << "y: " << msg.angular_velocity.y << std::endl;
        std::cout << "z: " << msg.angular_velocity.z << std::endl;
        std::cout << "linear_acceleration: -------------" << std::endl;
        std::cout << "x: " << msg.linear_acceleration.x << std::endl;
        std::cout << "y: " << msg.linear_acceleration.y << std::endl;
        std::cout << "z: " << msg.linear_acceleration.z << std::endl;
        std::cout << "gravity_vector: -------------" << std::endl;
        std::cout << "x: " << msg.gravity_vector.x << std::endl;
        std::cout << "y: " << msg.gravity_vector.y << std::endl;
        std::cout << "z: " << msg.gravity_vector.z << std::endl;
        std::cout << " ************************************** "<< std::endl;
        std::cout << "Gravity Magnitude: " << msg.gravity_magnitude << std::endl;

    std::cout << "************* Fusion Imu Data: ****************" << std::endl;

    for (auto msg : fusion_imu_msgs_)
    {
      std::cout << std::endl;
      std::cout << "header: -------------" << std::endl;
      // std::cout << "time stamp: " << msg.header.stamp.nsec << std::endl;
      std::cout << "time stamp: " << msg.header.stamp.nanosec << std::endl;
      std::cout << "orientation: -------------" << std::endl;
      std::cout << "y: " << msg.orientation.y << std::endl;
      std::cout << "x: " << msg.orientation.x << std::endl;
      std::cout << "z: " << msg.orientation.z << std::endl;
      std::cout << "euler_angles: -------------" << std::endl;
      std::cout << "y: " << msg.euler_angles.y << std::endl;
      std::cout << "x: " << msg.euler_angles.x << std::endl;
      std::cout << "z: " << msg.euler_angles.z << std::endl;
      std::cout << "angualar_velocity: -------------" << std::endl;
      std::cout << "x: " << msg.angular_velocity.x << std::endl;
      std::cout << "y: " << msg.angular_velocity.y << std::endl;
      std::cout << "z: " << msg.angular_velocity.z << std::endl;
      std::cout << "linear_acceleration: -------------" << std::endl;
      std::cout << "x: " << msg.linear_acceleration.x << std::endl;
      std::cout << "y: " << msg.linear_acceleration.y << std::endl;
      std::cout << "z: " << msg.linear_acceleration.z << std::endl;
      std::cout << "gravity_vector: -------------" << std::endl;
      std::cout << "x: " << msg.gravity_vector.x << std::endl;
      std::cout << "y: " << msg.gravity_vector.y << std::endl;
      std::cout << "z: " << msg.gravity_vector.z << std::endl;
      std::cout << " ************************************** " << std::endl;
      std::cout << "Gravity Magnitude: " << msg.gravity_magnitude << std::endl;
    }
  }

  void print_raw_imu() {
      // open a file in write mode.

      // write inputted data into the file.
      
      std::cout << "************* Raw Imu Data: ****************" << std::endl;
      
      for (auto wrap : raw_imu_msgs_) {
        interfaces::msg::RawImu msg;
        msg = wrap.msg;
        std::cout << std::endl;
        std::cout << "header: -------------" << std::endl;
        std::cout << "time stamp: " << wrap.time_stamp << std::endl;
        std::cout << "acceleration: -------------" << std::endl;
        std::cout <<  "x: " << msg.acceleration.x << std::endl;
        std::cout <<  "y: " << msg.acceleration.y << std::endl;
        std::cout <<  "z: " << msg.acceleration.z << std::endl;
        std::cout << "angualar_velocity: -------------" << std::endl;

        std::cout << "x: " << msg.angular_velocity.x << std::endl;
        std::cout << "y: " << msg.angular_velocity.y << std::endl;
        std::cout << "z: " << msg.angular_velocity.z << std::endl;
      }
  }  

  void save_fusion_imu_file() {

    std::ofstream outfile;
    
    outfile.open("FusionImuData.txt");

    
    outfile << "************* Fusion Imu Data: ****************" << std::endl;
    
    for (auto wrap : fusion_imu_msgs_) {
      interfaces::msg::FusionImu msg;
      msg = wrap.msg;
      outfile << std::endl;
      outfile << "header: -------------" << std::endl;
      outfile << "time stamp: " << wrap.time_stamp << std::endl;
      outfile << "orientation: -------------" << std::endl;
      outfile <<  "y: " << msg.orientation.y << std::endl;
      outfile <<  "x: " << msg.orientation.x << std::endl;
      outfile <<  "z: " << msg.orientation.z << std::endl;
      outfile << "euler_angles: -------------" << std::endl;
      outfile <<  "y: " << msg.euler_angles.y << std::endl;
      outfile <<  "x: " << msg.euler_angles.x << std::endl;
      outfile <<  "z: " << msg.euler_angles.z << std::endl;
      outfile << "angualar_velocity: -------------" << std::endl;
      outfile << "x: " << msg.angular_velocity.x << std::endl;
      outfile << "y: " << msg.angular_velocity.y << std::endl;
      outfile << "z: " << msg.angular_velocity.z << std::endl;
      outfile << "linear_acceleration: -------------" << std::endl;
      outfile << "x: " << msg.linear_acceleration.x << std::endl;
      outfile << "y: " << msg.linear_acceleration.y << std::endl;
      outfile << "z: " << msg.linear_acceleration.z << std::endl;
      outfile << "gravity_vector: -------------" << std::endl;
      outfile << "x: " << msg.gravity_vector.x << std::endl;
      outfile << "y: " << msg.gravity_vector.y << std::endl;
      outfile << "z: " << msg.gravity_vector.z << std::endl;
      outfile << " ************************************** "<< std::endl;
      outfile << "Gravity Magnitude: " << msg.gravity_magnitude << std::endl;

    }

    outfile.close();
      
  }

  void save_raw_imu_file() {
    // open a file in write mode.
      std::ofstream outfile;
      std::ofstream outfile2;
      outfile.open("RawImuData_Acceleration.txt");
      outfile2.open("RawImuData_AngularVelocity.txt");

      // write inputted data into the file.
      
      outfile << "************* Raw Imu Data Acceleration: ****************" << std::endl;
      outfile2 << "************* Raw Imu Data Angular Velocity: ****************" << std::endl;
      
      for (auto wrap : raw_imu_msgs_) {
        interfaces::msg::RawImu msg;
        msg = wrap.msg;

        outfile << std::endl;
        outfile << "header: -------------" << std::endl;
        outfile << "time stamp: " << wrap.time_stamp << std::endl;

        outfile << "acceleration: -------------" << std::endl;
        outfile <<  "x: " << msg.acceleration.x << std::endl;
        outfile <<  "y: " << msg.acceleration.y << std::endl;
        outfile <<  "z: " << msg.acceleration.z << std::endl;

        outfile2 << std::endl;
        outfile2 << "header: -------------" << std::endl;
        outfile2 << "time stamp: " << wrap.time_stamp << std::endl;

        outfile2 << "angualar_velocity: -------------" << std::endl;
        outfile2 << "x: " << msg.angular_velocity.x << std::endl;
        outfile2 << "y: " << msg.angular_velocity.y << std::endl;
        outfile2 << "z: " << msg.angular_velocity.z << std::endl;
      }

      outfile.close();
      outfile2.close();
  }

  void save_button_file() {

    std::ofstream outfile;
    
    // open file
    outfile.open("button_data.txt");

    outfile << "************* Button Data: ****************" << std::endl;


  }

};

int main(int argc, char **argv)
{

  std::string base_path = "" + std::filesystem::current_path().u8string() + "/my_bag/to_evaluate/Apr-27-2022";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagReader>(base_path));
  rclcpp::shutdown();
  return 0;
}
