#ifndef cnr_joint_teleop_controller__20188101642
#define cnr_joint_teleop_controller__20188101642

#include <cmath>
#include <Eigen/Core>
#include <ros/time.h>
#include <geometry_msgs/WrenchStamped.h>

#include <state_space_filters/filtered_values.h>
#include <cnr_controller_interface/cnr_joint_command_controller_interface.h>
#include <cnr_hardware_interface/posveleff_command_interface.h>
#include <cnr_hardware_interface/veleff_command_interface.h>


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
  bool       m_has_pos_sp;

  ect::FilteredVectorXd m_vel_fitler_sp;
  rosdyn::VectorXd m_vel_sp;
  rosdyn::VectorXd m_pos_sp;
  rosdyn::VectorXd m_pos_init;
  rosdyn::VectorXd m_dpos_sp;
  rosdyn::VectorXd m_vel_sp_last;
  rosdyn::VectorXd m_dist_to_pos_sp;
  
  Eigen::VectorXd   m_X_init;
  Eigen::VectorXd   m_X_ref;
  Eigen::VectorXd   m_X;
  Eigen::VectorXd   m_dX;
  
  Eigen::Vector6d   m_M;
  Eigen::Vector6d   m_D;
  Eigen::Vector6d   m_K;
    
  Eigen::Matrix<double, 4, 4> m_A;
  Eigen::Matrix<double, 4, 2> m_Bh;
  Eigen::Matrix<double, 4, 2> m_Br;
  Eigen::MatrixXd m_B;

  Eigen::Matrix<double, 4, 4> m_Q_hat;
  Eigen::Matrix<double, 4, 4> m_R_hat;
  Eigen::Matrix<double, 4, 4> m_Qr;
  Eigen::Matrix<double, 4, 4> m_Rr;

  rosdyn::VectorXd m_pos;
  
  bool   m_w_b_init;
  Eigen::Vector6d   m_w_b;
  Eigen::Vector6d   m_w_b_0;
  Eigen::Vector6d   m_wrench_deadband;

  rosdyn::ChainPtr m_chain_bs;

  size_t wrench_base_pub;
  size_t wrench_tool_pub;

  size_t cart_pos_ref_pub;
  size_t cart_pos_cur_pub;
  size_t cart_pos_err_pub;
  
  double m_init_time;
  double m_rho;
  double m_omega;
  double m_width;
  double m_offset;

  double sigma(double x);
  bool solveRiccatiArimotoPotter(const Eigen::MatrixXd &A,
                               const Eigen::MatrixXd &B,
                               const Eigen::MatrixXd &Q,
                               const Eigen::MatrixXd &R, Eigen::MatrixXd &P) ;
};


}
}

// #include <cnr_joint_teleop_controller/internal/cnr_joint_teleop_controller_impl.h>



#endif
