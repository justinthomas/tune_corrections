// Regular Includes
//#include <math.h>
#include <iostream>

// ROS Related Includes
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>

// Custom Includes
#include <controllers_manager/Transition.h>
#include <quadrotor_msgs/SO3Command.h>

// Local Includes
#include "nano_kontrol2.h"

using namespace std;

#define RED "\e[91m"
#define GREEN "\e[92m"
#define YELLOW "\e[93m"
#define BLUE "\e[94m"
#define MAGENTA "\e[95m"
#define CYAN "\e[96m"
#define RESET "\e[0m"

// State machine
enum controller_state
{
  ESTOP,
  NONE,
};
static enum controller_state state_ = NONE;

static const std::string null_tracker_str("null_tracker/NullTracker");

// Publishers & services
static ros::Publisher pub_motors_;
static ros::Publisher pub_estop_;
static ros::Publisher pub_so3_command_;
static ros::ServiceClient srv_transition_;

static bool motors_on_(false);

// Function Prototypes
void motors_on(const bool flag);

double kR_[3], kOm_[3], corrections_[3], kf_scale_, roll_scale_, pitch_scale_, mass_;
const float kGravity = 9.80665;

// Callbacks and functions
static void nanokontrol_cb(const sensor_msgs::Joy::ConstPtr &msg)
{
  if(msg->buttons[estop_button])
  {
    // Publish the E-Stop command
    ROS_WARN("E-STOP");
    std_msgs::Empty estop_cmd;
    pub_estop_.publish(estop_cmd);

    motors_on(false);

    state_ = ESTOP;
  }

  if (state_ == ESTOP)
    return;

  // Motors on (Rec)
  if(msg->buttons[motors_on_button])
    motors_on(true);
 
  double kf_correction = corrections_[0] + kf_scale_ * msg->axes[0];  // 1e11f  int16_t
  double roll_correction = corrections_[1] + roll_scale_ * msg->axes[1];  // 2500  uint8_t
  double pitch_correction = corrections_[2] + pitch_scale_ * msg->axes[2];  // 2500  uint8_t

  std::cout << std::scientific << CYAN;
  std::cout << "Corrections: {kf, roll, pitch} = {";
  std::cout << kf_correction << ", " << roll_correction << ", " << pitch_correction << "}";
  std::cout << RESET << endl;

  // Create and publish the so3_command
  quadrotor_msgs::SO3Command::Ptr cmd(new quadrotor_msgs::SO3Command);
  cmd->header.stamp = ros::Time::now();
  cmd->force.x = 0;
  cmd->force.y = 0;
  cmd->force.z = mass_ * kGravity;

  cmd->orientation.x = 0;
  cmd->orientation.y = 0;
  cmd->orientation.z = 0;
  cmd->orientation.w = 1;
  cmd->angular_velocity.x = 0;
  cmd->angular_velocity.y = 0;
  cmd->angular_velocity.z = 0;
  for(int i = 0; i < 3; i++)
  {
    cmd->kR[i] = kR_[i];
    cmd->kOm[i] = kOm_[i];
  }
  cmd->aux.current_yaw = 0;
  cmd->aux.kf_correction = kf_correction;
  cmd->aux.angle_corrections[0] = roll_correction;
  cmd->aux.angle_corrections[1] = pitch_correction;
  cmd->aux.enable_motors = motors_on_;
  cmd->aux.use_external_yaw = true;

  pub_so3_command_.publish(cmd);
}

void motors_on(const bool flag)
{
  motors_on_ = flag;

  std::string message = flag ? "Turning motors on..." : "Turning motors off...";
  cout << YELLOW << message.c_str() << RESET << endl;
  std_msgs::Bool motors_cmd;
  motors_cmd.data = flag;
  pub_motors_.publish(motors_cmd);

  // Switch to null_tracker so that the trackers do not publish so3_commands
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = null_tracker_str;
  srv_transition_.call(transition_cmd);

  // Create and publish the so3_command
  quadrotor_msgs::SO3Command::Ptr cmd(new quadrotor_msgs::SO3Command);
  cmd->aux.enable_motors = flag;
  pub_so3_command_.publish(cmd);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tune_corrections");
  ros::NodeHandle n("~");

  n.param("mass", mass_, 0.5);
  ROS_INFO("mass = %2.2f", mass_);

  // Attitude gains
  n.param("gains/rot/x", kR_[0], 1.5);
  n.param("gains/rot/y", kR_[1], 1.5);
  n.param("gains/rot/z", kR_[2], 1.0);
  n.param("gains/ang/x", kOm_[0], 0.13);
  n.param("gains/ang/y", kOm_[1], 0.13);
  n.param("gains/ang/z", kOm_[2], 0.1);
  ROS_INFO("Attitude gains: kR: {%2.2f, %2.2f, %2.2f}, kOm: {%2.2f, %2.2f, %2.2f}", kR_[0], kR_[1], kR_[2], kOm_[0], kOm_[1], kOm_[2]);

  // Scale factors for sliders
  n.param("kf_scale", kf_scale_, 1e-8);
  n.param("roll_scale", roll_scale_, 2.85 * M_PI / 180);  // Radians
  n.param("pitch_scale", pitch_scale_, 2.85 * M_PI / 180);  // Radians
  ROS_INFO("Scales {kf, roll, pitch} = {%2.2f, %2.2f, %2.2f}", kf_scale_, roll_scale_, pitch_scale_);

  // Corrections
  n.param("corrections/kf", corrections_[0], 0.0);
  n.param("corrections/r", corrections_[1], 0.0);
  n.param("corrections/p", corrections_[2], 0.0);
  ROS_INFO("Correction baselines: {kf, r, p} = {%2.2f, %2.2f, %2.2f}",
      corrections_[0], corrections_[1], corrections_[2]);

  // Publishers
  pub_motors_ = n.advertise<std_msgs::Bool>("motors", 1);
  pub_estop_ = n.advertise<std_msgs::Empty>("estop", 1);
  pub_so3_command_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 1);

  // Subscribers
  ros::Subscriber sub_nanokontrol = n.subscribe("/nanokontrol2", 1, nanokontrol_cb, ros::TransportHints().tcpNoDelay());

  motors_on(false);

  // Switch to null_tracker so that the trackers do not publish so3_commands
  controllers_manager::Transition transition_cmd;
  transition_cmd.request.controller = null_tracker_str;
  srv_transition_.call(transition_cmd);

  ros::spin();

  return 0;
}
