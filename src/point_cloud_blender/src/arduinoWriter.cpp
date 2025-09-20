#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <serial/serial.h>
#include <thread>
#include <memory>

class ArduinoSerialNode : public rclcpp::Node{
  public: 
  ArduinoSerialNode()
  : Node("arduino_serial_node")
  {
    // Init arduino communication
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 9600);

    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();

    try 
    {
      serial_.setPort(port);
      serial_.setBaudrate(baudrate);
      serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
      serial_.open();

      if (serial_.isOpen()) 
      {
        RCLCPP_INFO(this->get_logger(), "Connected to Arduino on %s at %d baud", port.c_str(), baudrate);
      }
    } 
    catch (const std::exception &e) 
    {
      RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s", e.what());
    }

    // Init pub
    stat_pub_ = this->create_publisher<std_msgs::msg::Bool>("motion_bool", 10); // Publishes when arduino finished moving

    // Init sub
    ang_sub_ = this->create_subscription<std_msgs::msg::Int32(
      "angle", 
      10, 
      std::bind(&ArduinoSerialNode::sendToArduino, this, std::placeholders::_1)); // 
  }

  private:
    void sendToArduino(const std_msgs::msg::Int32::SharedPtr msg)
    {
      // Send to Arduino angle
      if (serial_.isOpen()) {
        std::string cmd = msg->data + "\n";
        serial_.write(cmd);
        RCLCPP_INFO(this->get_logger(), "Sent to Arduino: %s", msg->data.c_str());
      }

      // Wait for reply from Arduino
      try{
        std::string line;
        while (rclcpp::ok()) {
          line = serial_.readline(256, "\n"); // blocking read
          if (!line.empty()) break;          // exit once we got data
        }
      }
      
      // Publish message because finished moving
      std_msgs::msg::Bool status;
      status.data = True;
      stat_pub_.publish(status)
    }

    serial::Serial serial_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stat_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr ang_sub_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArduinoSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
