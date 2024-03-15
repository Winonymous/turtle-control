#include <string.h>
#include <ctime>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;
using namespace std;
using namespace std::chrono_literals;

class MoveLinear : public rclcpp::Node
{
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_pub_;
        geometry_msgs::msg::Twist mov;
        time_t start, finish;
        
    public:
        MoveLinear() : Node("move_linear"){
            speed_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
                "turtle1/cmd_vel", 10);
        }

        void move_distance(float speed, float distance, int direction){
            float expected_time = distance/speed;
            time(&start);

            if(direction == 0){
                mov.linear.x = -1 * speed;
            } else {
                mov.linear.x = speed;
            }

            while(difftime(time(&finish), start) < expected_time){
                // cout << difftime(time(&finish), start) << " " << expected_time << endl;
                speed_pub_->publish(mov);
            }
        }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    MoveLinear MvLinaer;

    float speed;
    float distance;
    float direction;

    cout << "Enter speed: ";
    cin >> speed;

    cout << "Enter Distance: ";
    cin >> distance;

    cout << "Enter direction (1-Forward, 0-Backward): ";
    cin >> direction;


    MvLinaer.move_distance(speed, distance, direction); 

    rclcpp::shutdown();
    return 0;
}

