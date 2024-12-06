#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <string>

using std::placeholders::_1;
using std::placeholders::_2;

class Sum : public rclcpp::Node {

    typedef std::shared_ptr<std_msgs::msg::Float32 const> float32_t;

    private:

        std::string m_sTA1 = "/addendo1", m_sTA2 = "/addendo2";
        std::string m_sSum = "/somma";

        // Subscriber
        message_filters::Subscriber<std_msgs::msg::Float32> m_subA1;
        message_filters::Subscriber<std_msgs::msg::Float32> m_subA2;

        std::shared_ptr<message_filters::TimeSynchronizer<std_msgs::msg::Float32, std_msgs::msg::Float32>> m_addSync;
        void syncCallback(const float32_t& add1, const float32_t& add2);

        // Publisher
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m_pubSum;

    public:

        Sum();
        ~Sum() {};
};