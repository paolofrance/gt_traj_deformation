#pragma once

#include <cmath>
#include <Eigen/Core>
#include <ros/time.h>
#include <geometry_msgs/WrenchStamped.h>

#include <state_space_filters/filtered_values.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>


namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief The GtTrajDeformation class
 */
class GtTrajDeformation:
    public cnr::control::JointCommandController<
//        hardware_interface::PosVelEffJointHandle, hardware_interface::PosVelEffJointInterface>
        hardware_interface::JointHandle, hardware_interface::VelocityJointInterface>
{
public:
  GtTrajDeformation();
  bool doInit();
  bool doUpdate(const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);
  void callback(const geometry_msgs::WrenchStampedConstPtr& msg );

protected:

  std::mutex m_mtx;

  ect::FilteredVectorXd m_vel_fitler_sp;
  ect::FilteredVectorXd m_wrench_fitler;

  rosdyn::VectorXd m_dq_sp;
  rosdyn::VectorXd m_q_sp;
  rosdyn::VectorXd m_vel_sp_last;
  rosdyn::VectorXd m_dist_to_pos_sp;

  Eigen::VectorXd   m_X_zero;
  Eigen::VectorXd   m_X_ref;
  Eigen::VectorXd   m_X;
  Eigen::VectorXd   m_dX;

  Eigen::Matrix<double, 4, 4> m_A;
  Eigen::Matrix<double, 4, 2> m_Bh;
  Eigen::Matrix<double, 4, 2> m_Br;
  Eigen::MatrixXd m_B;
  Eigen::MatrixXd m_C;
  Eigen::MatrixXd m_D;
  Eigen::MatrixXd m_S;

  Eigen::Matrix<double, 4, 4> m_Q_hat;
  Eigen::Matrix<double, 4, 4> m_R_hat;
  Eigen::Matrix<double, 4, 4> m_Qr;
  Eigen::Matrix<double, 4, 4> m_Rr;

  bool m_w_b_init;
  bool m_use_filtered_wrench;
  Eigen::Vector6d m_w_b_filt;
  Eigen::Vector6d m_w_b;
  Eigen::Vector6d m_w_b_0;
  Eigen::Vector6d m_wrench_deadband;

  Eigen::Affine3d T_b_t_;

  rosdyn::ChainPtr m_chain_bs;

  size_t filtered_wrench_base_pub;
  size_t wrench_base_pub;
  size_t wrench_tool_pub;

  size_t cart_pos_ref_pub;
  size_t cart_pos_cur_pub;
  size_t cart_pos_sp_pub;
  size_t cart_pos_traj_pub;
  size_t alpha_pub;
  size_t human_wrench_pub;
  size_t human_u_pub;
  size_t robot_u_pub;
  size_t joint_sp_pub;
  size_t D_pub;
  size_t K_pub;
  size_t activate_gripper_pub;

  double m_init_time;
  double m_rho;
  double m_omega;
  double m_width ;
  double m_half_x;
  double m_height;
  double m_max_y ;

  double m_stiffness;
  double m_damping;
  double m_mass;

  boost::shared_ptr<actionlib::SimpleActionClient<control_msgs::GripperCommandAction>> gripper_action_;

  ros::ServiceClient m_gripper_srv;

  double sigma(double x);
  bool solveRiccati(const Eigen::MatrixXd &A,
                               const Eigen::MatrixXd &B,
                               const Eigen::MatrixXd &Q,
                               const Eigen::MatrixXd &R, Eigen::MatrixXd &P) ;
};


}
}
