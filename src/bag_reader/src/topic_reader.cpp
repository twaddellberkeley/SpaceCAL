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

class BagReader : public rclcpp::Node
{

  public:
  BagReader() : Node("Bag_Reader")
  {
    rosbag2_cpp::readers::SequentialReader reader;
    rosbag2_cpp::StorageOptions storage_options{};
    std::cout << std::filesystem::current_path() << std::endl;
    
    storage_options.uri = "/home/cjc/spaceCal/SpaceCAL/my_bag/to_evaluate/Apr-27-2022/20-8-31/20-8-31_0.db3";
    storage_options.storage_id = "sqlite3";


    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = rmw_get_serialization_format();
    converter_options.output_serialization_format = rmw_get_serialization_format();
    reader.open(storage_options, converter_options);
    
    auto topics = reader.get_all_topics_and_types();

    // about metadata
    for (auto t:topics){
      std::cout << "meta name: " << t.name << std::endl;
      std::cout << "meta type: " << t.type << std::endl;
      std::cout << "meta serialization_format: " << t.serialization_format << std::endl;
    }

    
    
    while(reader.has_next()) {

      auto serialized_message = reader.read_next();

      // rosbag2_cpp::introspection_message_set_topic_name(
      //   allocated_ros_message.get(), message->topic_name.c_str());
        
      if (serialized_message->topic_name == FUSION_IMU) {
        std::cout << "serialized message topic name: " << serialized_message->topic_name << std::endl;
        interfaces::msg::FusionImu msg;


      }
      if (serialized_message->topic_name == RAW_IMU) {
        
        interfaces::msg::RawImu msg;

        auto ros_message = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
        ros_message->time_stamp = 0;
        // ros_message->topic_name = nullptr;
        ros_message->message = nullptr;
        ros_message->allocator = rcutils_get_default_allocator();
        ros_message->message = &msg;

        
        auto library = rosbag2_cpp::get_typesupport_library("interfaces/msg/RawImu", "rosidl_typesupport_cpp" );
        auto type_support = rosbag2_cpp::get_typesupport_handle("interfaces/msg/RawImu", "rosidl_typesupport_cpp", library);

        
        rosbag2_cpp::SerializationFormatConverterFactory factory;
        std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer;
        cdr_deserializer = factory.load_deserializer(rmw_get_serialization_format());
       
        std::cout << "**************  Made it here  ***********" << std::endl;

        cdr_deserializer->deserialize(serialized_message, type_support, ros_message);
        
    

        std::cout << "serialized message topic name: " << serialized_message->topic_name << std::endl;
        // ros message data
        std::cout << std::endl;
        std::cout << "header: -------------" << std::endl;
        // std::cout << "time stamp: " << msg.header.stamp.nsec << std::endl;
        std::cout << "time stamp: " << serialized_message->time_stamp << std::endl;
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

    printf("hello world bag_reader package\n");
  }
  
  // Create a list of directories of the bsag files

  // open a bag file 

  // sort messages by topic and time stamp

  // do somthing with the information
};


int main(int argc, char ** argv)
{
  
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BagReader>());
  rclcpp::shutdown();
  return 0;
}
