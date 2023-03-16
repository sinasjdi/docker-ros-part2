#include "ros_kinematic.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/*
PluginNode::PluginNode() : rclcpp::Node("gz_plugin",
                                        rclcpp::NodeOptions()
                                            .allow_undeclared_parameters(true)
                                            .automatically_declare_parameters_from_overrides(true))
{

    ///RCLCPP_INFO(get_logger(), "Initializing gz_plugin node.\n");

    //clk_subscriber_ = this->create_subscription<rosgraph_msgs::msg::Clock>("clock", 10, std::bind(&PluginNode::clk_update, this, std::placeholders::_1));

   // publisher_ = this->create_publisher<std_msgs::msg::String>("gazebo_topic", 10);

    //subscription_ = this->create_subscription<std_msgs::msg::String>("gazebo_topic", 10, std::bind(&PluginNode::topic_callback, this, std::placeholders::_1));

    // publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

    /*
     pose_subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("uam/pose",
                                                                                   10,
                                                                                   std::bind(&PluginNode::pose_update, this, std::placeholders::_1));
    twst_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("uam/twist",
                                                                             10,
                                                                             std::bind(&PluginNode::twist_update, this, std::placeholders::_1));


    timer_ = this->create_wall_timer(500ms, std::bind(&PluginNode::timer_callback, this));
}

void PluginNode::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
}

void PluginNode::topic_callback(const std_msgs::msg::String::SharedPtr msg) const
{
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

void PluginNode::clk_update(const rosgraph_msgs::msg::Clock::SharedPtr time) const
{
    /// current::time = time->clock.sec;
    // RCLCPP_INFO(get_logger(), "HELLO.\n");
}

void PluginNode::pose_update(const geometry_msgs::msg::PoseStamped::SharedPtr pose) const
{
    // current::pose[0] = pose->pose.position.x;
    // current::pose[1] = pose->pose.position.y;
    // current::pose[2] = pose->pose.position.z;

    // current::orientation[0] = pose->pose.orientation.w;
    // current::orientation[1] = pose->pose.orientation.x;
    // current::orientation[2] = pose->pose.orientation.y;
    // current::orientation[3] = pose->pose.orientation.z;
}

void PluginNode::twist_update(const geometry_msgs::msg::Twist::SharedPtr twst) const
{
    ;
}

float g = 0.0;

*/

namespace gazebo
{
    class ModelPush : public ModelPlugin
    {
    public:
        /// A pointer to the GazeboROS node.
        gazebo_ros::Node::SharedPtr ros_node_;

        /// Subscriber to command velocities
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        /// Odometry publisher
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;

        /// Connection to event called at every world iteration.
        gazebo::event::ConnectionPtr update_connection_;

        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
        {

            // rclcpp::executors::MultiThreadedExecutor executor2;
            // executor2.add_node(std::make_shared<PluginNode>());
            // Spin the Executor in a separate thread.
            // std::thread spinThread([&executor2](){ executor2.spin(); });

            // Store the pointer to the model
            this->model = _parent;
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&ModelPush::OnUpdate, this));
            // Listen to the update event. This event is broadcast every
            // simulation iteration.

            // rclcpp::spin(std::make_shared<PluginNode>());
            // rclcpp::executors::MultiThreadedExecutor executor;
            // executor.add_node(std::make_shared<PluginNode>());
            // executor.spin();
        }

        // Called by the world update start event
    public:
        void OnUpdate()
        {
            // Apply a small linear velocity to the model.
            // this->model->SetLinearVel(ignition::math::Vector3d(.3, 0, 0));
            this->model->GetJoint("prop1")->SetVelocity(0, 10);
            this->model->GetJoint("prop2")->SetVelocity(0, 10);
            this->model->GetJoint("prop3")->SetVelocity(0, 10);
            this->model->GetJoint("prop4")->SetVelocity(0, 10);
            this->model->GetJoint("inter")->SetVelocity(0, 0);

            this->model->GetJoint("manip_base")->SetVelocity(0, 0);
            this->model->GetJoint("end")->SetVelocity(0, 0);

            this->model->GetLink("base_link")->SetLinearVel({0, 0, 0.1});
            this->model->GetLink("base_link")->SetAngularVel({0, 0, 0});
            // this->model->GetLink("base_link")->SetVelocity(0, 1.0);

            // BASE LINK
            // current::pose_here = this->model->GetLink("base_link")->WorldInertialPose();
            // current::pose_here - current::pose;
        }

        // Pointer to the model
    private:
        physics::ModelPtr model;

        // Pointer to the update event connection
    private:
        event::ConnectionPtr updateConnection;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}