#include <adder-correct/sum.hpp>

Sum::Sum() : Node("sum_node")
{
    RCLCPP_INFO(this->get_logger(), "inizialize class");

    this->m_subA1.subscribe(this, this->m_sTA1);
    this->m_subA2.subscribe(this, this->m_sTA2);

    this->m_addSync = std::make_shared<message_filters::TimeSynchronizer<std_msgs::msg::Float32, std_msgs::msg::Float32>>(this->m_subA1, this->m_subA2, 10);
    this->m_addSync->registerCallback(std::bind(&Sum::syncCallback, this, _1, _2));

    this->m_pubSum = this->create_publisher<std_msgs::msg::Float32>(this->m_sSum, 1);
}

void Sum::syncCallback(const float32_t& add1, const float32_t& add2)
{
    RCLCPP_INFO(this->get_logger(), "[ ADD 1 ]: %f, [ ADD 2 ]: %f", add1->data, add2->data);

    std_msgs::msg::Float32 msg;
    msg.data = add1->data + add2->data;

    this->m_pubSum->publish(msg);
}

void Sum::load_params() {
    this->declare_parameter<std::string>("addendo1_topic", "/a1");
    this->declare_parameter<std::string>("addendo2_topic", "/a2");
    this->declare_parameter<std::string>("sum_topic", "/s");

    this->m_sTA1 = this->get_parameter("addendo1_topic").as_string();
    this->m_sTA2 = this->get_parameter("addendo2_topic").as_string();
    this->m_sSum = this->get_parameter("sum_topic").as_string();

    RCLCPP_INFO(this->get_logger(), "[TOPICS] Addendo1: %s | Addendo2: %s | Somma: %s", this->m_sTA1.c_str(), this->m_sTA2.c_str(), this->m_sSum.c_str());
}