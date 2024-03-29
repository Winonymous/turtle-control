#include "string"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "turtle_control/action/move.hpp"

using namespace std::placeholders;

class MoveActionServer : public rclcpp::Node 
{
    public:
        using Move = turtle_control::action::Move;
        using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;

        explicit MoveActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions()) : Node("goal_action_server", options)
        {
            pub = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
            this->action_server_ = rclcpp_action::create_server<Move>(
                this,
                "goal",
                std::bind(&MoveActionServer::handlegoal, this, _1, _2),
                std::bind(&MoveActionServer::handelcancel, this, _1),
                std::bind(&MoveActionServer::handle_accepted, this, _1));
        };
    private:
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
        rclcpp_action::Server<Move>::SharedPtr action_server_;

        // Callback for hanling new goal
        rclcpp_action::GoalResponse handlegoal(
            const rclcpp_action::GoalUUID &uuid,
            std::shared_ptr<const Move::Goal> goal
        )
        {
            RCLCPP_INFO(this->get_logger(), "Recieved goal request with distance %f", goal->desired_dist);
            (void)uuid;

            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }

        // Callback for dealing with goal cancelation 
        rclcpp_action::CancelResponse handelcancel(
            const std::shared_ptr<GoalHandleMove> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "Recieved request to cancel goal");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            }

        // Callback for new goal 
        void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle){
            std::thread{std::bind(&MoveActionServer::execute, this, _1), goal_handle}.detach();
        }

        // continue to process and update the action
        void execute(std::shared_ptr<GoalHandleMove> goal_handle)
        {
            double speed = 2.0;
            bool is_forward = true;
            RCLCPP_INFO(this->get_logger(), "Executing goal");
            rclcpp::Rate rate(5);
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<Move::Feedback>();
            auto & feedback_message = feedback->feedback;
            auto result = std::make_shared<Move::Result>();
            geometry_msgs::msg::Twist speed_msg;
            
            if (is_forward){
                speed_msg.linear.x = abs(speed);
            } else {
                speed_msg.linear.x = -1 * abs(speed);
            }

            speed_msg.linear.y = 0.0;
            speed_msg.linear.z = 0.0;
            speed_msg.angular.x = 0.0;
            speed_msg.angular.y = 0.0;
            speed_msg.angular.z = 0.0;
            
            rclcpp::Clock time;
            double t0 = time.now().seconds();
            double current_distance = 0.0;

            while(current_distance < goal->desired_dist && rclcpp::ok()){
                if(goal_handle->is_canceling()){
                    result->status = feedback_message;
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal Cancelled");
                }

                // Update feeedback
                double t1 = time.now().seconds();
                current_distance = speed_msg.linear.x * (t1 - t0);

                feedback_message = "Current distance is " + std::to_string(current_distance);
                pub->publish(speed_msg);
                goal_handle->publish_feedback(feedback);
                RCLCPP_INFO(this->get_logger(), "Publish Feedback");
                rate.sleep();
            }

            // check is done
            if (rclcpp::ok()){
                result->status = "Reached Goal";
                speed_msg.linear.x = 0;
                pub->publish(speed_msg);
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal Succeded");
            }
        }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto move_action_server = std::make_shared<MoveActionServer>();
    rclcpp::spin(move_action_server);
    rclcpp::shutdown();
    return 0;
}
