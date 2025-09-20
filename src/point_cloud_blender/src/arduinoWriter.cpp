#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int32.hpp>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <boost/asio.hpp>

class ArduinoSerialNode : public rclcpp::Node{
  public: 
  ArduinoSerialNode()
  : Node("arduino_serial_node"), serial_(io_)
  {
    // Init arduino communication
    try {
      serial_.open("/dev/ttyACM0");
      serial_.set_option(boost::asio::serial_port_base::baud_rate(9600));
    }
    catch (boost::system::system_error &e) 
    {
      RCLCPP_ERROR(this->get_logger(), "Could not open serial port: %s", e.what());
      return;
    }

    // Init pub
    stat_pub_ = this->create_publisher<std_msgs::msg::Header>("motion_bool", 10); // Publishes when arduino finished moving

    // Init sub
    ang_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "angle", 
      10, 
      std::bind(&ArduinoSerialNode::sendToArduino, this, std::placeholders::_1)); // 
  }

  private:
    void sendToArduino(const std_msgs::msg::Int32::SharedPtr &msg)
    {
      // Send to Arduino angle
      if (serial_.is_open()) {
        std::string cmd = msg->data + "\n";
        boost::asio::write(serial_,boost::asio::buffer(cmd));
        RCLCPP_INFO(this->get_logger(), "Arduino messaged");

        // Wait for reply from Arduino
        boost::asio::streambuf buf;
        boost::asio::read_until(serial_, buf, "\n"); // Message recieved
        
        // Publish message because finished moving
        std_msgs::msg::Header status;
        status.stamp = this->now();
        status.frame_id = "1"; // Publish Bool
        stat_pub_->publish(status);
        RCLCPP_INFO(this->get_logger(), "Arduino finished moving");
      }
    }

    boost::asio::io_service io_;
    boost::asio::serial_port serial_;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Header>::SharedPtr stat_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr ang_sub_;
};


int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArduinoSerialNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
