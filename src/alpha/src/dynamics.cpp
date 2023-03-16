#include "dynamics.hpp"

/*
Important warning [DO NOT REMOVE UNTIL NO LONGER APPLICABLE]
  For topic /uam/pose
    Twist messages are being used to publish pose data due to Isaac Sim's constraints.
      rot_euler - published as angular speed
      tf_3d - published as linear speed
*/


class SimNode : public rclcpp::Node{
  public:
    SimNode() : Node("uam_sim_node",
                    rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)){

      RCLCPP_INFO(get_logger(), "Initializing node.\n");

      clk_publisher_ = this->create_publisher<rosgraph_msgs::msg::Clock>("clock", 10);
      pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("uam/pose", 10);
      twst_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("uam/twist", 10);

      this->get_parameter_or("sim_end_time", this->t_final_secs, rclcpp::Parameter("sim_end_time", 1000));
      this->get_parameter_or("sim_step_size", this->t_step_secs, rclcpp::Parameter("sim_step_size", 0.1));
      this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    }


    void publish_time(rosgraph_msgs::msg::Clock& time){
      clk_publisher_->publish(time);
    }

    void publish_state(geometry_msgs::msg::PoseStamped& pose, geometry_msgs::msg::Twist& twist){
      pose_publisher_->publish(pose);
      twst_publisher_->publish(twist);
    }

    rclcpp::Parameter t_final_secs;
    rclcpp::Parameter t_step_secs;

  private:
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clk_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twst_publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimNode>();

  double t_cur = 0;
  double t_final = node->t_final_secs.as_int();
  double t_step = node->t_step_secs.as_double();
  int t_factor = 1;

  // RCLCPP_INFO(rclcpp::get_logger("Hello"), "1!\n");

  rclcpp::Rate loop_rate((int) t_factor/((double)t_step));

  RCLCPP_INFO(rclcpp::get_logger("SIM_INFO"), "Starting Simulation with frequency %f Hz\n", (int) t_factor/((double)t_step));


  rosgraph_msgs::msg::Clock t_msg;
  geometry_msgs::msg::PoseStamped pose_msg;
  geometry_msgs::msg::Twist twist_msg;
  tf2::Quaternion orientation;

  geometry_msgs::msg::Point uam_pt;
  
  uam_pt.z = 0;

  std::vector<state> x;
  std::vector<state> dx;
  double temp[4] = {0, 0, 0, 0};
  control u(temp);

  while (rclcpp::ok() && t_cur < t_final) {
            RCLCPP_INFO(rclcpp::get_logger("Tick"), "%f\n", t_cur);
            
            t_msg.clock.sec = floor(t_cur);
            t_msg.clock.nanosec = (int) ((t_cur - floor(t_cur))*1000000000);
            node->publish_time(t_msg);

            uam_pt.x = 0;
            uam_pt.y = 0;
            uam_pt.z += 1;

            orientation.setEuler(0, 0, 0);

            pose_msg.pose.orientation.w = orientation.getW();
            pose_msg.pose.orientation.x = orientation.getX();
            pose_msg.pose.orientation.y = orientation.getY();
            pose_msg.pose.orientation.z = orientation.getZ();

            pose_msg.pose.position.x = uam_pt.x;
            pose_msg.pose.position.y = uam_pt.y;
            pose_msg.pose.position.z = uam_pt.z;

            node->publish_state(pose_msg, twist_msg);
            dx.push_back(dx_compute(x[int(t_step*t_cur)], u));
            rk4(x, dx, u, t_step, t_cur);

            rclcpp::spin_some(node);
            loop_rate.sleep();
            t_cur += t_step;
 }

  rclcpp::shutdown();
  return 0;
}

/*
x1 x
x2 y
x3 z
x4 \dot x
x5 \dot y
x6 \dot z
x7 \theta
x8 \phi
x9 \psi
x10 \dot \theta
x11 \dot \phi
x12 \dot \psi
*/

state dx_compute(state x, control u){
  double data[n];
  for(int i = 0; i < 3; i++){
    // dx 1, 2, 3
    data[i] = x.data[i+3];
    
  } 
  return state(data);
}

void rk4(std::vector<state> x_list, std::vector<state> dx_list, control u_cur, double Ts, double t){
  std::vector<state> k;
  k.push_back(dx_compute(x_list[int(t*Ts)], u_cur)*Ts);
  k.push_back(dx_compute(x_list[int(t*Ts)] + k[0]*0.5, u_cur)*Ts);
  k.push_back(dx_compute(x_list[int(t*Ts)] + k[1]*0.5, u_cur)*Ts);
  k.push_back(dx_compute(x_list[int(t*Ts)] + k[2], u_cur)*Ts);
  
  x_list.push_back(x_list[int(t*Ts)] + (k[0] + k[1]*2 + k[2]*2 + k[3])*(1/6.0));
}