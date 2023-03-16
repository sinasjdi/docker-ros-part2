#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cstdio>
#include <vector>

const int n = 12; // number of states
const int m = 4; // number of control inputs

/*
eg. x1 = [x, y, z] (n = 3)
x1, x2, x3, x4 are vectors forming state vector
*/

class state{
private:
  ;
public:
  state(double[n]);
  double data[n];
  ~state();
};

state::state(double data[n]){
  for (int i = 0; i < n; i++)
    this->data[i] = data[i];
}

state::~state(){
}

state operator* (const state& x, double y){
  double data[n];
  for (int i = 0; i < n; i++)
    data[i] = y*x.data[i];
  
  return state(data);
} 

state operator+ (const state& x, const state& y){
  double data[n];
  for (int i = 0; i < n; i++)
    data[i] = x.data[i] + y.data[i];

  return state(data);
} 

class control{
private:
  /* data */
public:
  control(double[m]);
  double data[n];
  ~control();
};

control::control(double data[m]){
  for (int i = 0; i < m; i++)
    this->data[i] = data[i];
}

control::~control(){
}

state dx_compute(state x, control u);
void rk4(std::vector<state> x_list, std::vector<state> dx_list, control u_cur, double Ts, double t);