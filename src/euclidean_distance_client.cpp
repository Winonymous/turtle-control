#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

int main(int argc, char const *argv[])
{   
    rclcpp::init(argc, argv);

    if(argc != 3)
    {
        RCLCPP_INFO(rclcpp::get_logger("Usage"),  "Input two coordinates");
        return 1;
    }
    
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("euclidean_distance_client");
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client =   
        node->create_client<example_interfaces::srv::AddTwoInts>("dist_two_point");

    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();

    request->a = atoll(argv[1]);
    request->b = atoll(argv[2]);

    while(!client->wait_for_service(1s))
    {
        if(!rclcpp::ok()){
            RCLCPP_ERROR(rclcpp::get_logger("Usage"), "Interupted while waiting service: Exiting");

            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("Usage"),  "Service not availabe try again");
    }

    auto result = client->async_send_request(request);
    if(rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_INFO(rclcpp::get_logger("Usage"),  "Distance betweed Pose %ld", result.get()->sum);
        
    }else{
        RCLCPP_INFO(rclcpp::get_logger("Usage"),  "Failed to call service");
    }

    rclcpp::shutdown();
    return 0;
}
