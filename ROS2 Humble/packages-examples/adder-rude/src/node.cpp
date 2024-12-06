#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <list>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class Calculator : public rclcpp::Node
{
public:
  Calculator() : Node("adder_node"), qos_(10)
  {
    qos_.reliable();
    this->subscriber1_ = this->create_subscription<std_msgs::msg::Float32>("addendo1", qos_, std::bind(&Calculator::addendo1_callback, this, _1));
    this->subscriber2_ = this->create_subscription<std_msgs::msg::Float32>("addendo2", qos_, std::bind(&Calculator::addendo2_callback, this, _1));
    this->adder_ = this->create_publisher<std_msgs::msg::Float32>("somma", qos_);
    this->timer_ = this->create_wall_timer(1s, std::bind(&Calculator::timer_callback, this));
  }

private:
  void addendo1_callback(const std_msgs::msg::Float32 &msg)
  {
    float data = msg.data;
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", data);
    this->subscriber1_bag_.push_back(data);
  }

  void addendo2_callback(const std_msgs::msg::Float32 &msg)
  {
    float data = msg.data;
    RCLCPP_INFO(this->get_logger(), "I heard: '%f'", data);
    this->subscriber2_bag_.push_back(data);
  }

  void timer_callback()
  {
    if (!subscriber1_bag_.empty() && !subscriber2_bag_.empty())
    {
      auto message = std_msgs::msg::Float32();
      message.data = subscriber1_bag_.front() + subscriber2_bag_.front();
      this->adder_->publish(message);
      RCLCPP_INFO(this->get_logger(), "I calculated: '%f'", message.data);
      subscriber1_bag_.pop_front();
      subscriber2_bag_.pop_front();
    }
  }

  rclcpp::QoS qos_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr adder_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber1_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber2_;
  std::list<float> subscriber1_bag_;
  std::list<float> subscriber2_bag_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Calculator>());
  rclcpp::shutdown();
  return 0;
}
