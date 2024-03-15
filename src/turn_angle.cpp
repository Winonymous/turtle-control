#include <string.h>
#include <ctime>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;

class TurnAngle : public rclcpp::Node
{
    public:
        TurnAngle() : Node("turn_angle")
        {
            speed_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        }

        void call(float angular_velocity, float angle, float direction)
        {
            if(direction == 0){
                turn.angular.z = (angle/180) * (22/7);
            } else {
                turn.angular.z = -1 * (angle/180) * (22/7);
            }

            turn.angular.x = angular_velocity;

            speed_pub_->publish(turn);
        }
    
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_pub_;
        geometry_msgs::msg::Twist turn;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    TurnAngle turner;

    float angular_velocity;
    float angle;
    float direction;

    cout << "Enter angular_velocity: ";
    cin >> angular_velocity;

    cout << "Enter angle: ";
    cin >> angle;

    cout << "Enter direction (1-Forward, 0-Backward): ";
    cin >> direction;

    turner.call(angular_velocity, angle, direction);
    
    rclcpp::shutdown();
}