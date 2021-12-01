#include "mini_pupper_controller.hpp"

#include "ros/package.h"
#include "pluginlib/class_list_macros.h"
#include <fstream>
#include <cmath>


namespace mini_pupper_trajopt {

MiniPupperController::MiniPupperController()
  : joint_handlers_(),
    q_cmd_() {
  const std::string path_to_log
      = ros::package::getPath("mini_pupper_trajopt") + "/rsc/trotting/q.log";
      // = ros::package::getPath("mini_pupper_trajopt") + "/rsc/running/q.log";
  std::ifstream log;
  log.open(path_to_log);
  std::string line;
  int rows = 0;
  while (std::getline(log, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    std::vector<double> q;
    while (std::getline(lineStream, cell, ',')) {
      q.push_back(std::stod(cell));
    }
    ++rows;
    q_cmd_.push_back(Eigen::Map<const Eigen::VectorXd>(q.data(), 19));
  }
}


bool MiniPupperController::init(
    hardware_interface::PositionJointInterface* hardware, 
    ros::NodeHandle& node_handle) {
  const std::vector<std::string> joint_names = {
      "base_lb1", "lb1_lb2", "lb2_lb3", 
      "base_lf1", "lf1_lf2", "lf2_lf3", 
      "base_rb1", "rb1_rb2", "rb2_rb3",
      "base_rf1", "rf1_rf2", "rf2_rf3"};
  if (!joint_handlers_.empty()) {
    return false;
  }
  for (const auto& joint_name : joint_names) {
    joint_handlers_.push_back(hardware->getHandle(joint_name));
  }
  crtl_start_time_ = ros::Time::now();
  return true;
}


void MiniPupperController::starting(const ros::Time& time) {
}


void MiniPupperController::stopping(const ros::Time& time) {
}


void MiniPupperController::update(const ros::Time& time, 
                                  const ros::Duration& period) { 
  const ros::Duration time_from_ctrl_start = ros::Time::now() - crtl_start_time_;
  int time_step = std::floor(time_from_ctrl_start.toSec() / 0.005); 
  if (time_step < 0) {
    time_step = 0;
  }
  else if (time_step >= q_cmd_.size()) {
    time_step = q_cmd_.size() - 1;
  }
  for (int i=0; i<12; ++i) {
    joint_handlers_[i].setCommand(q_cmd_[time_step].coeff(i+7));
  }
}

} // namespace mini_pupper_trajopt 


PLUGINLIB_EXPORT_CLASS(mini_pupper_trajopt::MiniPupperController, 
                       controller_interface::ControllerBase)