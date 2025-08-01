#include <rclcpp/rclcpp.hpp>
#include <memory>


class DWAPlanner : public rclcpp::Node
{
  public:
  DWAPlanner() : Node("dwa_planner_action_server")
  {
        RCLCPP_INFO(this->get_logger(), "Starting DWA Planner Action Server....");


  
  }

  

  private:



};


int main(int argc, char **argv)
{
  rclcpp::init(argc,argv);

  rclcpp::spin(std::make_shared<DWAPlanner>());
  
  rclcpp::shutdown();

  return 0;
}
