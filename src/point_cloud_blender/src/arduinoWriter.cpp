#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <serial/serial.h>
#include <thread>
#include <memory>

class ArduinoSerialNode : public rclcpp::Node
{
public:
  ArduinoSerialNode()
  : Node("arduino_serial_node")
  {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baudrate", 9600);

    std::string port = this->get_parameter("port").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();

    try {
      serial_.setPort(port);
      serial_.setBaudrate(baudrate);
      serial_.setTimeout(serial::Timeout::simpleTimeout(1000));
      serial_.open();

      if (serial_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Connected to Arduino on %s at %d baud", port.c_str(), baudrate);
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s", e.what());
    }

    publisher_ = this->create_publisher<std_msgs::msg::String>("arduino/data", 10);
    subscriber_ = this->create_subscription<std_msgs::msg::String>(
      "arduino/cmd",
      10,
      std::bind(&ArduinoSerialNode::sendToArduino, this, std::placeholders::_1)
    );

    // Start reading thread
    if (serial_.isOpen()) {
      read_thread_ = std::thread(&ArduinoSerialNode::readFromSerial, this);
    }
  }

  ~ArduinoSerialNode()
  {
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    if (serial_.isOpen()) {
      serial_.close();
    }
  }

private:
  void sendToArduino(const std_msgs::msg::String::SharedPtr msg)
  {
    if (serial_.isOpen()) {
      std::string cmd = msg->data + "\n";
      serial_.write(cmd);
      RCLCPP_INFO(this->get_logger(), "Sent to Arduino: %s", msg->data.c_str());
    }
  }

  void readFromSerial()
  {
    while (rclcpp::ok() && running_ && serial_.isOpen()) {
      try {
        std::string line = serial_.readline(256, "\n");
        if (!line.empty()) {
          auto ros_msg = std_msgs::msg::String();
          ros_msg.data = line;
          publisher_->publish(ros_msg);
          RCLCPP_INFO(this->get_logger(), "Received from Arduino: %s", line.c_str());
        }
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading serial: %s", e.what());
        break;
      }
    }
  }

  serial::Serial serial_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  std::thread read_thread_;
  bool running_ = true;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArduinoSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
