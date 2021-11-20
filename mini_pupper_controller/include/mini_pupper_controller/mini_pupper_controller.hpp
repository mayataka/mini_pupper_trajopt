#ifndef MINI_PUPPER_CONTROLLER_HPP_
#define MINI_PUPPER_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "Eigen/Core"

#include "ros/ros.h"
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"


namespace mini_pupper_trajopt {

class MiniPupperController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  MiniPupperController();
  bool init(hardware_interface::EffortJointInterface* hardware, 
            ros::NodeHandle& node_handler) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:
  void update(const ros::Time& time, const ros::Duration& period) override;

  static std::vector<Eigen::VectorXd> getDataFromLogFile(
      const std::string& path_to_log_file, const int dim);

  std::vector<hardware_interface::JointHandle> joint_handlers_;
  std::vector<Eigen::VectorXd> q_cmd_, v_cmd_, u_cmd_;
  Eigen::VectorXd q_, v_, u_;
  double Kq_, Kv_;
  ros::Time crtl_start_time_; 
};

} // namespace mini_pupper_trajopt 

#endif // MINI_PUPPER_CONTROLLER_HPP_ 