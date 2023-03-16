#include <functional>
#include <memory>
#include <vector>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/PhysicsTypes.hh>
#include <rosgraph_msgs/msg/clock.hpp>
#include <gazebo/common/Time.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include <gazebo_plugins/gazebo_ros_diff_drive.hpp>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include<std_msgs/msg/string.hpp>
#include <sdf/sdf.hh>

class PluginNode : public rclcpp::Node
{
public:
    PluginNode();
   ~PluginNode();

private:
    
    
    //rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clk_subscriber_;
    /*
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twst_subscriber_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr  publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    size_t count_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
   
    void clk_update(const rosgraph_msgs::msg::Clock::SharedPtr time) const;
    void timer_callback() ;
    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const;

    void pose_update(const geometry_msgs::msg::PoseStamped::SharedPtr pose) const;
    void twist_update(const geometry_msgs::msg::Twist::SharedPtr twst) const;
*/

};
/*
class WPose
{
private:
public:
    WPose(){
        this->position(3);
    }
    std::vector<double> position;
    std::vector<double> orientation;

    WPose operator + (WPose const x)
    {
        WPose z;
        z.position = x.position + this->position;

    }
};
*/
