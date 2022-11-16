// #include "dlpc_common.h"
// #include "dlpc34xx.h"
// #include "cypress_i2c.h"
// #include "devasys_i2c.h"
// #include "stdio.h"
// #include "stdint.h"
// #include "stdbool.h"
// #include "string.h"
// #include "time.h"
// #include <iostream>
// #include <unistd.h> 


#include "rclcpp/rclcpp.hpp"
#include "interfaces/srv/projector.hpp"
#include <string>
#include<thread>
#include <memory>
extern "C" {
#include "proj_exec/proj_utils.h"
}



void proj_callback(const std::shared_ptr<interfaces::srv::Projector::Request> request,
          std::shared_ptr<interfaces::srv::Projector::Response>      response);


int main(int argc, char **argv)
{
  	rclcpp::init(argc, argv);

	dlp_init();

  	std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("proj_exec_node");

	node->declare_parameter("projector_number", "1");

	std::string my_param =
      node->get_parameter("projector_number").get_parameter_value().get<std::string>();
	
	std::string service_name = "projector_srv_" + my_param;

  	rclcpp::Service<interfaces::srv::Projector>::SharedPtr service =
    	node->create_service<interfaces::srv::Projector>(service_name, &proj_callback);

  	RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to control projector.");

  	rclcpp::spin(node);
	relinquishI2CBuss();
  	rclcpp::shutdown();
}


void proj_callback(const std::shared_ptr<interfaces::srv::Projector::Request> request,
          std::shared_ptr<interfaces::srv::Projector::Response> response)
{
  response->cmd = request->cmd;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %d" " b: %s",
                request->id, request->cmd);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", (int)response->id);

	const char * req_cmd = request->cmd.c_str();
	std::cout << req_cmd << std::endl;

	if (strcmp(req_cmd, "on") == 0) 
	{
		system("vcgencmd display_power 1");
		dlp_init();
		std::cout << "Turning ON projector\n";
	}
	else if (strcmp(req_cmd, "off") == 0)
	{
		system("vcgencmd display_power 0");
		std::cout << "Turning OFF projector\n";
	}
	else if (strcmp(req_cmd, "led-on") == 0)
	{
		if(request->id > 0) {
			std::cout << "hello";
			std::thread (led_time_on, request->id).detach();
		} else {
			led_on();
			std::cout << "Turning LED ON \n";
		}
	} else if (strcmp(req_cmd, "led-off") == 0) {
		led_off();
		std::cout << "Turning LED OFF \n";
	} else {
		std::cout << "Commnad not recognized!!" << std::endl;
	}

}

// def process_cmd(self, req, res):
//         msg = ""
//         if req.cmd == "on":
//             if subprocess.run(["vcgencmd", "display_power", "1"]).returncode != 0:
//                 self.get_logger().error("Could not turn HDMI Power On")
//             res.status = "HDMI-Power-On"
//             res.is_power_on = True
//             msg = "Turned HDMI Power On"
//         elif req.cmd == "off":
//             if subprocess.run(["vcgencmd", "display_power", "0"]).returncode != 0:
//                 self.get_logger().error("Could not turn HDMI Power Off")
//             res.status = "HDMI-Power-Off"
//             res.is_power_on = False
//             msg = "Turned HDMI Power Off"
//         elif req.cmd == "led-on":
//             self.get_logger().info("Turning LED On")
//             # If true then led is on for that time
//             if req.id > 0: 
//                 self.timeThread = threading.Thread(target=self.timedPrint, args=[req.id])
//                 self.timeThread.daemon = True
//                 self.timeThread.start()
//                 #self.timeThread.join()
//             else:
//                 subprocess.run("ledOn")
//             res.is_led_on = True
//             res.status = "LED-On"
//             msg = "Turning LED On"
//         elif req.cmd == "led-off":
//             self.get_logger().info("Turning LED Off")
//             subprocess.run("ledZero")
//             res.status = "LED-Off"
//             res.is_led_on = False
//             msg = "Turned LED On"
//         else:
//             msg = "DID NOT FIND COMMAND"
//         res.err = 0
//         #res.id = self.proj_num
//         res.cmd = req.cmd
//         res.msg = "Projector " + str(self.proj_num) + " " + msg + " succesfully"
//         return res