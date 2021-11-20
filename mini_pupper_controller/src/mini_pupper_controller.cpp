#include "mini_pupper_controller.hpp"

#include "ros/package.h"
#include "pluginlib/class_list_macros.h"
#include <fstream>
#include <cmath>


namespace mini_pupper_trajopt {

MiniPupperController::MiniPupperController()
  : joint_handlers_(),
    q_cmd_(),
    v_cmd_(),
    u_cmd_(),
    q_(Eigen::VectorXd::Zero(12)),
    v_(Eigen::VectorXd::Zero(12)),
    u_(Eigen::VectorXd::Zero(12)),
    Kq_(1.0),
    Kv_(0.1) {
  const std::string path_to_log = ros::package::getPath("mini_pupper_trajopt") + "/rsc/running/";
  const std::string path_to_q_log = path_to_log + "q.log";
  const std::string path_to_v_log = path_to_log + "v.log";
  const std::string path_to_u_log = path_to_log + "u.log";
  q_cmd_ = getDataFromLogFile(path_to_q_log, 19);
  v_cmd_ = getDataFromLogFile(path_to_v_log, 18);
  u_cmd_ = getDataFromLogFile(path_to_u_log, 18);
}


std::vector<Eigen::VectorXd> MiniPupperController::getDataFromLogFile(
    const std::string& path_to_log_file, const int dim) {
  std::vector<Eigen::VectorXd> res;
  std::ifstream log;
  log.open(path_to_log_file);
  std::string line;
  int rows = 0;
  while (std::getline(log, line)) {
    std::stringstream lineStream(line);
    std::string cell;
    std::vector<double> var;
    while (std::getline(lineStream, cell, ',')) {
      var.push_back(std::stod(cell));
    }
    ++rows;
    res.push_back(Eigen::Map<const Eigen::VectorXd>(var.data(), dim));
  }
  return res;
}


bool MiniPupperController::init(
    hardware_interface::EffortJointInterface* hardware, 
    ros::NodeHandle& node_handle) {
  const std::vector<std::string> joint_names = {
      "base_lf1", "lf1_lf2", "lf2_lf3", 
      "base_rf1", "rf1_rf2", "rf2_rf3", 
      "base_lb1", "lb1_lb2", "lb2_lb3", 
      "base_rb1", "rb1_rb2", "rb2_rb3"};
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
  for (int i=0; i<12; ++i) {
    q_.coeffRef(i) = joint_handlers_[i].getPosition();
    v_.coeffRef(i) = joint_handlers_[i].getVelocity();
  }
  const ros::Duration time_from_ctrl_start = ros::Time::now() - crtl_start_time_;
  int time_step = std::floor(time_from_ctrl_start.toSec() / 0.005); 
  if (time_step < 0) {
    time_step = 0;
  }
  else if (time_step >= u_cmd_.size()) {
    time_step = u_cmd_.size() - 1;
  }
  u_ = u_cmd_[time_step].tail(12);
  u_.noalias() += Kq_ * (q_ - q_cmd_[time_step].tail(12));
  u_.noalias() += Kv_ * (v_ - v_cmd_[time_step].tail(12));
  for (int i=0; i<12; ++i) {
    joint_handlers_[i].setCommand(u_.coeff(i));
  }
}

} // namespace mini_pupper_trajopt 


PLUGINLIB_EXPORT_CLASS(mini_pupper_trajopt::MiniPupperController, 
                       controller_interface::ControllerBase)