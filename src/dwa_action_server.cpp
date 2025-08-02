#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "dwa_motion_planner/action/dwa_planner.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <memory>
#include <cmath>
#include <vector>
#include <random>
#include <algorithm>

using namespace std::placeholders;
using Dwa = dwa_motion_planner::action::DwaPlanner;
using GoalHandleDwa = rclcpp_action::ServerGoalHandle<Dwa>;

class DWAPlanner : public rclcpp::Node
{
  public:
   
  DWAPlanner() : Node("dwa_planner_action_server")
  {     
        dwa_action_server_ = rclcpp_action::create_server<Dwa>(
                          this,
                          "dwa_planner",
                          std::bind(&DWAPlanner::handle_goal, this, _1,_2),
                          std::bind(&DWAPlanner::handle_cancel, this, _1),
                          std::bind(&DWAPlanner::handle_accepted, this,_1)
                          );
        
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odom",10,std::bind(&DWAPlanner::odom_callback,this, _1));
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/scan",10,std::bind(&DWAPlanner::laserscan_callback,this, _1));

        vel_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("/cmd_vel", 10);
        path_pub_ = create_publisher<visualization_msgs::msg::Marker>("/visual_paths",10);

        speed_distribution_ = std::uniform_real_distribution<double>(0,max_speed_);
        turn_distribution_ = std::uniform_real_distribution<double>(-max_turn_,max_turn_);


        RCLCPP_INFO(this->get_logger(), "Starting DWA Planner Action Server....");
 
  }

  

  private:
  
  const double max_speed_ = 0.15;
  const double max_turn_ = 2.5;
  const double step_time_ = 0.1;
  
  double goal_x_ = 0, goal_y_ = 0, distance = 0;
  bool goal_reached_ = false;
  int marker_counter_ = 0, cycle_counter_ = 0;

  double current_x, current_y, roll, pitch, yaw;

  rclcpp_action::Server<Dwa>::SharedPtr dwa_action_server_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr path_pub_;

  nav_msgs::msg::Odometry::SharedPtr odom_data_;
  sensor_msgs::msg::LaserScan::SharedPtr scan_data_;

  rclcpp::TimerBase::SharedPtr timer_;
  
  std::random_device rd_;
  std::mt19937 gen_{rd_()};
  std::uniform_real_distribution<double> speed_distribution_;
  std::uniform_real_distribution<double> turn_distribution_;

 


  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const Dwa::Goal> goal)
  {
        goal_x_ = goal->goal_x;
        goal_y_ = goal->goal_y;

        goal_reached_ = false;
        
        RCLCPP_INFO(this->get_logger(), "Recieved goal : goal_x = %f goal_y = %f",goal_x_,goal_y_);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleDwa>)
  {
        RCLCPP_INFO(this->get_logger(), "Canceling Request");
        
        return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleDwa> goal_handle)
  {
        RCLCPP_INFO(this->get_logger(), "Handle is accepted");
        std::thread([this,goal_handle]() { execute(goal_handle); }).detach();
  } 
  
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
        odom_data_ = msg;
  }

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
        scan_data_ = msg;
  }
  
  void publish_path_marker(const std::vector<std::pair<double, double>>& path, int id, float r, float g, float b) 
  {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = this->now();
        marker.ns = "dwa_paths";
        marker.id = id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.02; // Line width

        marker.color.r = r;
        marker.color.g = g;
        marker.color.b = b;
        marker.color.a = 1.0;

        for (const auto& [x, y] : path) {
           geometry_msgs::msg::Point p;
           p.x = x;
           p.y = y;
           p.z = 0.0;
           marker.points.push_back(p);
        }

        path_pub_->publish(marker); 
  }



  void clear_all_markers()
  {
        visualization_msgs::msg::Marker delete_all;
        delete_all.action = visualization_msgs::msg::Marker::DELETEALL;
        path_pub_->publish(delete_all);
        marker_counter_ = 0;
  }
  
  std::vector<std::pair<double,double>> predict_motion(double speed, double turn_rate)
  {
        std::vector<std::pair<double,double>> path;
        if(!odom_data_) return path;
        double x = current_x, y = current_y, yaw_t = yaw;
        for(int i = 0; i< 100; ++i)
        {
                yaw_t += turn_rate * step_time_;
                x += speed * cos(yaw_t) * step_time_;
                y += speed * sin(yaw_t) * step_time_;
                path.emplace_back(x,y);
        }

        return path; 
  }


  std::pair<double,double> generate_path()
  {
          double speed = speed_distribution_(gen_);
          double turn = turn_distribution_(gen_);

          return {speed,turn};
  }

  double check_for_collisions(const std::vector<std::pair<double,double>>& path)
  {
        if(!scan_data_) return -INFINITY;
        double safety_margin = 0.3;

        for(const auto& [x,y] : path)
        {
                double scan_distance = hypot(x,y);
                double angle = atan2(y, x);
                if (angle < 0) angle += 2 * M_PI;
                int scan_index = static_cast<int>((angle/ (2 * M_PI)) * scan_data_->ranges.size());

                scan_index = std::max(0, std::min(static_cast<int>(scan_data_->ranges.size()) - 1, scan_index));

                if(scan_distance < scan_data_->ranges[scan_index] - safety_margin)
                {
                        return -100000;
                }
        }

        return 0;
  }

  std::pair<double, double> choose_best_path()
  {
        if(!odom_data_) return {0.0,0.0};
        
        current_x = odom_data_->pose.pose.position.x;
        current_y = odom_data_->pose.pose.position.y;
        
        tf2::Quaternion q(odom_data_->pose.pose.orientation.x,odom_data_->pose.pose.orientation.y,
                        odom_data_->pose.pose.orientation.z, odom_data_->pose.pose.orientation.w);
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        distance = hypot(goal_x_ - current_x, goal_y_ - current_y);

        if(distance < 0.05)
        {
                if(!goal_reached_)
                        goal_reached_ = true;   

                return {0.0,0.0};
        }

        if(++cycle_counter_ % 50 == 0)
                clear_all_markers();

        struct PathOption
        {
                double score;
                double speed;
                double turn;
                std::vector<std::pair<double,double>> path;
        };

        std::vector<PathOption> all_paths;

        for(int i = 0; i< 300; ++i)
        {
                auto [speed,turn] = generate_path();
                auto path = predict_motion(speed, turn);
                
                double goal_distance_score = -hypot(path.back().first - goal_x_, path.back().second - goal_y_) * 5;
                double angle_diff = abs(atan2(goal_y_ - current_y, goal_x_ - current_x) - yaw);
                double heading_score = -angle_diff * 2;
                double collision_risk = check_for_collisions(path);
                double smoothness_score = -0.1 * abs(turn);

                double total_score = goal_distance_score + heading_score + collision_risk + smoothness_score;

                all_paths.push_back({total_score,speed,turn,path});
        }

        std::sort(all_paths.begin(), all_paths.end(), [](const PathOption& a,const PathOption& b) { return a.score > b.score;});

        int top_n = std::min(20, static_cast<int>(all_paths.size()));
        for (int i = 0; i < top_n; ++i)
        {
                float green_intensity = static_cast<float>(i) / top_n;
                publish_path_marker(all_paths[i].path, marker_counter_++, 0.0f, green_intensity, 1.0f - green_intensity);
        }

        const auto& best = all_paths.front();
        RCLCPP_DEBUG(this->get_logger(), "Best path: speed=%.2f, turn=%.2f, score=%.2f",best.speed,best.turn,best.score);
        
        return{best.speed,best.turn};

  }

  void execute(const std::shared_ptr<GoalHandleDwa> goal_handle)
  {
        auto feedback = std::make_shared<Dwa::Feedback>();
        auto result = std::make_shared<Dwa::Result>();
        rclcpp::Rate rate(10);
        //while(true)
        while(rclcpp::ok() && !goal_reached_ && scan_data_ && odom_data_)
        {
                auto [speed, turn] = choose_best_path();

                geometry_msgs::msg::TwistStamped move_cmd;
                move_cmd.header.stamp = this->now();
                move_cmd.header.frame_id = "base_link";
                move_cmd.twist.linear.x = speed;
                move_cmd.twist.angular.z = turn;

                vel_pub_->publish(move_cmd);

                
                
                feedback->distance_remaining = distance;
                goal_handle->publish_feedback(feedback);
                RCLCPP_DEBUG(this->get_logger(), "Publishing to turtlebot : speed=%.2f, turn=%.2f", speed,turn);
        
                rate.sleep();
        }
        
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        if(goal_reached_)
        {
                result->reached = true;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(),"Goal (%.2f, %.2f) Reached!", goal_x_,goal_y_);
        }
        else
        {
                result->reached = false;
                goal_handle->abort(result);
                RCLCPP_ERROR(this->get_logger(),"Goal Aborted!");
        }
  }

};


int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<DWAPlanner>());
  
  rclcpp::shutdown();

  return 0;
}
