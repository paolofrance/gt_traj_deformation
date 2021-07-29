#pragma once

#include <cmath>
#include <Eigen/Core>
#include <ros/time.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/JointState.h>

#include <state_space_filters/filtered_values.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>
#include <control_msgs/GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

//#include <cnr_cartesian_velocity_controller/cnr_cartesian_velocity_controller.h>


namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief The GtTrajDeformation class
 */
class GtTrajDeformation: public cnr::control::JointCommandController<
        hardware_interface::JointHandle, hardware_interface::VelocityJointInterface>
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  GtTrajDeformation();
  bool doInit();
  bool doUpdate  (const ros::Time& time, const ros::Duration& period);
  bool doStarting(const ros::Time& time);
  bool doStopping(const ros::Time& time);

  void callback              (const geometry_msgs::WrenchStampedConstPtr&  msg );
  void setTargetPoseCallback (const geometry_msgs::PoseStampedConstPtr&    msg );
  void setTargetTwistCallback(const geometry_msgs::TwistStampedConstPtr&   msg );

protected:

//  cnr::control::CartesianVelocityToJointVelocityController vc_;

  std::mutex m_mtx;

  ect::FilteredVectorXd m_vel_fitler_sp;
  ect::FilteredVectorXd m_wrench_fitler;

  rosdyn::VectorXd m_dq_sp;
  rosdyn::VectorXd m_q_sp;
  rosdyn::VectorXd m_vel_sp_last;
  rosdyn::VectorXd m_dist_to_pos_sp;

  Eigen::Vector6d m_X_zero;
  Eigen::Vector6d m_X_ref;
  Eigen::Vector6d m_X_sp;
  Eigen::Vector6d m_X;
  Eigen::Vector6d m_X_prev;
  Eigen::Vector6d m_dX;

  Eigen::Matrix<double, 6, 6> m_A;
  Eigen::Matrix<double, 6, 3> m_Bh;
  Eigen::Matrix<double, 6, 3> m_Br;
  Eigen::MatrixXd m_B;
  Eigen::MatrixXd m_C;
  Eigen::MatrixXd m_D;
  Eigen::MatrixXd m_S;

  Eigen::Matrix<double, 6, 6> m_Q_hat;
  Eigen::Matrix<double, 6, 6> m_R_hat;
  Eigen::Matrix<double, 6, 6> m_Qr;
  Eigen::Matrix<double, 6, 6> m_Rr;

  bool m_target_ok;
  bool init_X_;

  int count_update_;

  bool m_w_b_init;
  bool m_use_filtered_wrench;
  Eigen::Vector6d m_w_b_filt;
  Eigen::Vector6d m_w_b;
  Eigen::Vector6d m_w_b_0;
  Eigen::Vector6d m_wrench_deadband;

  Eigen::Affine3d T_b_t_;

  rosdyn::ChainPtr m_chain_bs;
  rosdyn::ChainPtr m_chain_bt;

  size_t filtered_wrench_base_pub;
  size_t wrench_base_pub;
  size_t wrench_tool_pub;

  size_t cart_pos_ref_pub;
  size_t cart_pos_cur_pub;
  size_t cart_pos_traj_pub;
  size_t alpha_pub;
  size_t human_wrench_pub;
  size_t human_u_pub;
  size_t robot_u_pub;
  size_t joint_sp_pub;
  size_t D_pub;
  size_t K_pub;
  size_t path_pub;
  size_t activate_gripper_pub;

  size_t target_twist_pub_;

  double m_width ;
  double m_half_x;
  double m_height;
  double m_max_y ;

  double exponential_;
  double min_val_;

  double m_stiffness;
  double m_damping;
  double m_mass;

  Eigen::Vector6d M_;
  Eigen::Vector6d D_;
  Eigen::Vector6d K_;

  std::vector<geometry_msgs::PoseStamped> traj_path_;

  std::shared_ptr<ros_helper::SubscriptionNotifier<geometry_msgs::PoseStamped>> m_target_sub;

  double sigma(double x);
  bool solveRiccati(const Eigen::MatrixXd &A,
                               const Eigen::MatrixXd &B,
                               const Eigen::MatrixXd &Q,
                               const Eigen::MatrixXd &R, Eigen::MatrixXd &P) ;

//  bool prepareInit(hardware_interface::VelocityJointInterface* hw,
//                   const std::string& hw_name, const std::string& ctrl_name,
//                   ros::NodeHandle& root_nh, ros::NodeHandle& ctrl_nh) override
//  {
//    this->cnr::control::JointCommandController<
//        hardware_interface::JointHandle, hardware_interface::VelocityJointInterface>::
//          prepareInit(hw, hw_name, ctrl_name, root_nh, ctrl_nh);

//    CNR_TRACE_START(m_logger);
//    if(!vc_.prepareInit(m_hw, hw_name, "vc",
//                        this->getRootNh(), this->getControllerNh() ))
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    CNR_RETURN_TRUE(m_logger);
//  }

//  bool enterInit() override
//  {
//    CNR_TRACE_START(m_logger);
//    if(!cnr::control::JointCommandController<
//            hardware_interface::JointHandle, hardware_interface::VelocityJointInterface>::enterInit())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    if(!vc_.enterInit())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    CNR_RETURN_TRUE(m_logger);
//  }
//  bool enterStarting() override
//  {
//    CNR_TRACE_START(m_logger);
//    if(!cnr::control::JointCommandController<
//            hardware_interface::JointHandle, hardware_interface::VelocityJointInterface>::enterStarting())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    if(!vc_.enterStarting())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    CNR_RETURN_TRUE(m_logger);
//  }
//  bool exitStarting() override
//  {
//    CNR_TRACE_START(m_logger);
//    if(!cnr::control::JointCommandController<
//            hardware_interface::JointHandle, hardware_interface::VelocityJointInterface>::exitStarting())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    if(!vc_.exitStarting())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    CNR_RETURN_TRUE(m_logger);
//  }

//  bool enterUpdate() override
//  {
//    CNR_TRACE_START(m_logger);
//    if(!cnr::control::JointCommandController<
//            hardware_interface::JointHandle, hardware_interface::VelocityJointInterface>::enterUpdate())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    if(!vc_.enterUpdate())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    CNR_RETURN_TRUE(m_logger);
//  }
//  bool exitUpdate() override
//  {
//    CNR_TRACE_START(m_logger);
//    if(!cnr::control::JointCommandController<
//            hardware_interface::JointHandle, hardware_interface::VelocityJointInterface>::exitUpdate())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    if(!vc_.exitUpdate())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    CNR_RETURN_TRUE(m_logger);
//  }

//  bool enterStopping() override
//  {
//    CNR_TRACE_START(m_logger);
//    if(!cnr::control::JointCommandController<
//            hardware_interface::JointHandle, hardware_interface::VelocityJointInterface>::enterStopping())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    if(!vc_.enterStopping())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    CNR_RETURN_TRUE(m_logger);
//  }

//  bool exitStopping() override
//  {
//    CNR_TRACE_START(m_logger);
//    if(!cnr::control::JointCommandController<
//            hardware_interface::JointHandle, hardware_interface::VelocityJointInterface>::exitStopping())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    if(!vc_.exitStopping())
//    {
//      CNR_RETURN_FALSE(m_logger);
//    }
//    CNR_RETURN_TRUE(m_logger);
//  }

};


}
}
