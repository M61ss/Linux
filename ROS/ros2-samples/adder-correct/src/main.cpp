#include <adder-correct/sum.hpp>

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    
    try 
    {
        auto node = std::make_shared<Sum>();
        rclcpp::spin(node);
        rclcpp::shutdown();
    }
    catch(const rclcpp::exceptions::InvalidNodeError& e) {
        std::cerr << e.what() << '\n';
    }
    
    return 1;
}