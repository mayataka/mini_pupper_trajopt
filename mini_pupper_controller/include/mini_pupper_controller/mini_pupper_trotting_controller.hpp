#ifndef MINI_PUPPER_TROTTING_CONTROLLER_HPP_
#define MINI_PUPPER_TROTTING_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "Eigen/Core"

#include "ros/ros.h"
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"


namespace mini_pupper_trajopt {

class MiniPupperTrottingController : public controller_interface::Controller<hardware_interface::PositionJointInterface> {
public:
  MiniPupperTrottingController();
  bool init(hardware_interface::PositionJointInterface* hardware, 
            ros::NodeHandle& node_handler) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:
  void update(const ros::Time& time, const ros::Duration& period) override;

  std::vector<hardware_interface::JointHandle> joint_handlers_;
  std::vector<Eigen::VectorXd> q_cmd_;
  ros::Time crtl_start_time_; 
};

} // namespace mini_pupper_trajopt 

#endif // MINI_PUPPER_TROTTING_CONTROLLER_HPP_ 