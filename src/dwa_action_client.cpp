#include "rclcpp/rclcpp.hpp"
#include <memory>
#include "rclcpp_action/rclcpp_action.hpp"
#include "dwa_motion_planner/action/dwa_planner.hpp"

using namespace std::placeholders;
using Dwa = dwa_motion_planner::action::DwaPlanner;
using GoalHandleDwa = rclcpp_action::ClientGoalHandle<Dwa>;

class DWAClient : public rclcpp::Node
{
        public:
        DWAClient() : Node("dwa_demo_client")
        {
                dwa_action_client_ = rclcpp_action::create_client<Dwa>(this, "dwa_planner");

                if(!dwa_action_client_->wait_for_action_server(std::chrono::seconds(10)))
                {
                        RCLCPP_ERROR(this-> get_logger(), "DWA Planner Action Server Not Available");
                        return;
                }

                auto goal_coord = Dwa::Goal();
                goal_coord.goal_x = 2.0;
                goal_coord.goal_y = 1.0;

                auto send_goal_options = rclcpp_action::Client<Dwa>::SendGoalOptions();

                send_goal_options.feedback_callback = std::bind(&DWAClient::feedback_callback, this, _1, _2);

                send_goal_options.result_callback = std::bind(&DWAClient::result_callback, this, _1);
                
                dwa_action_client_->async_send_goal(goal_coord, send_goal_options);
        }

        private:

        rclcpp_action::Client<Dwa>::SharedPtr dwa_action_client_;
        
        void feedback_callback(GoalHandleDwa::SharedPtr, const std::shared_ptr<const Dwa::Feedback> feedback)
        {
                RCLCPP_INFO(this->get_logger(), "Distance remaining : %.2f", feedback->distance_remaining);
        }

        void result_callback(const GoalHandleDwa::WrappedResult & result)
        {
                switch(result.code)
                {
                        case rclcpp_action::ResultCode::SUCCEEDED:
                                RCLCPP_INFO(this->get_logger(), "Goal Reached : %s", result.result->reached ? "true" : "false");
                                break;

                        default:
                                RCLCPP_ERROR(this->get_logger(), "Goal Failed");
                                break;
                }

        }
};


int main(int argc, char **argv)
{
        rclcpp::init(argc,argv);

        rclcpp::spin(std::make_shared<DWAClient>());

        rclcpp::shutdown();

        return 0;
}




























