#include <gt_traj_deformation/gt_traj_deformation.h>
#include <state_space_filters/filtered_values.h>
#include <eigen_matrix_utils/overloads.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/JointState.h>
#include <rosdyn_core/primitives.h>
#include <std_srvs/SetBool.h>

PLUGINLIB_EXPORT_CLASS(cnr::control::GtTrajDeformation  , controller_interface::ControllerBase)

namespace std
{
inline std::string to_string ( const std::vector<std::string>& vals )
{
    std::string ret = "< ";
    for ( auto const & val : vals ) ret += val + ", ";
    ret += " >";
    return ret;
}

inline std::string to_string ( const std::string& val )
{
    return val;
}

inline std::string to_string ( const bool& val )
{
    return val ? "TRUE" : "FALSE";
}

}


#define GET_AND_RETURN( nh, param, value )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_ERROR("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    return false;\
  }



#define GET_AND_DEFAULT( nh, param, value, def )\
  if (!nh.getParam(param,value) )\
  {\
    ROS_WARN("The param '%s/%s' is not defined", nh.getNamespace().c_str(), std::string( param ).c_str() );\
    ROS_WARN("Default value '%s' superimposed. ", std::to_string( def ).c_str() );\
    value=def;\
  }

#define GET_PARAM_VECTOR_AND_RETURN(nh, P, X , N, DEQ )\
  if (!nh.getParam( std::string(P).c_str(), X))\
  {\
    ROS_FATAL_STREAM("[ " << nh.getNamespace().c_str() << "] Parameter '"<<  P <<"' does not exist");\
    ROS_FATAL_STREAM("[ " << nh.getNamespace().c_str() << "] ERROR DURING INITIALIZATION. ABORT.");\
    return false;\
  }\
  if( X.size() != N )\
  {\
    ROS_FATAL_STREAM("[ " << nh.getNamespace().c_str() << "] The size '"<< X.size() <<"' of the param '" << P << "' does not match with the foreseen dimension '"<< N <<"'");\
    ROS_FATAL_STREAM("[ " << nh.getNamespace().c_str() << "] ERROR DURING INITIALIZATION. ABORT.");\
    return false;\
  }\
  for( size_t i=0; i<X.size(); i++)\
  {\
    if( std::string(DEQ)==std::string("<"))\
    {\
      if( X.at(i) < 0) \
      {\
        ROS_FATAL_STREAM("[ " << nh.getNamespace().c_str() << "] Parameter '"<<  P <<"' has negative values. Abort");\
        return false;\
      }\
    }\
    else if( std::string(DEQ)==std::string("<="))\
    {\
      if( X.at(i) <= 0) \
      {\
        ROS_FATAL_STREAM("[ " << nh.getNamespace().c_str() << "] Parameter '"<<  P <<"' has negative or null values. Abort");\
        return false;\
      }\
    }\
  }\

namespace eu = eigen_utils;
namespace ect = eigen_control_toolbox;

namespace cnr
{
namespace control
{


/**
 * @brief GtTrajDeformation::GtTrajDeformation
 */
GtTrajDeformation::GtTrajDeformation()
{
}

/**
 * @brief GtTrajDeformation::doInit
 * @return
 */
bool GtTrajDeformation::doInit()
{
  //INIT PUB/SUB
  std::string external_wrench_topic = "/robotiq_ft_wrench";

  this->template add_subscriber<geometry_msgs::WrenchStamped>(
        external_wrench_topic,5,boost::bind(&GtTrajDeformation::callback,this,_1), false);

  filtered_wrench_base_pub = this->template add_publisher<geometry_msgs::WrenchStamped>("filtered_wrench_base",5);
  wrench_base_pub          = this->template add_publisher<geometry_msgs::WrenchStamped>("wrench_base",5);
  wrench_tool_pub          = this->template add_publisher<geometry_msgs::WrenchStamped>("wrench_tool",5);

  cart_pos_ref_pub  = this->template add_publisher<geometry_msgs::PoseStamped>("pose_ref"    ,5);
  cart_pos_cur_pub  = this->template add_publisher<geometry_msgs::PoseStamped>("pose_cur"    ,5);
  cart_pos_sp_pub   = this->template add_publisher<geometry_msgs::PoseStamped>("pose_sp"     ,5);
  cart_pos_traj_pub = this->template add_publisher<geometry_msgs::PoseStamped>("pose_traj"   ,5);
  joint_sp_pub      = this->template add_publisher<sensor_msgs::JointState>   ("joint_sp"    ,5);
  alpha_pub         = this->template add_publisher<std_msgs::Float32>         ("alpha"       ,5);
  human_wrench_pub  = this->template add_publisher<std_msgs::Float32>         ("human_wrench",5);
  D_pub             = this->template add_publisher<std_msgs::Float32>         ("var_D"       ,5);
  K_pub             = this->template add_publisher<std_msgs::Float32>         ("var_K"       ,5);

  std::string gripper_name;
  GET_AND_DEFAULT(this->getControllerNh(),"gripper_name", gripper_name, "gripper");
  gripper_name += "/goal";
  activate_gripper_pub = this->template add_publisher<control_msgs::GripperCommandActionGoal>(gripper_name,5);

  human_u_pub = this->template add_publisher<geometry_msgs::WrenchStamped>("human_u",5);
  robot_u_pub = this->template add_publisher<geometry_msgs::WrenchStamped>("robot_u",5);

  this->setPriority(this->QD_PRIORITY);
  {
      ect::FilteredVectorXd::Value dead_band;
      ect::FilteredVectorXd::Value saturation;
      ect::FilteredVectorXd::Value init_value;

      dead_band = 0.0 * m_chain.getDQMax();
      saturation = m_chain.getDQMax();
      init_value = dead_band;
      if(!m_vel_fitler_sp.activateFilter ( dead_band, saturation, (10.0 / 2.0 / M_PI), this->m_sampling_period, init_value ))
      {
        CNR_RETURN_FALSE(this->logger());
      }
  }
  m_dq_sp = m_vel_fitler_sp.getUpdatedValue();
  m_q_sp = this->getPosition();

  m_wrench_deadband .setZero();
  m_w_b             .setZero();

  // system params initi
  m_X .resize(4);
  m_dX.resize(4);

  m_Q_hat.setZero();
  m_R_hat.setZero();
  m_Qr   .setZero();
  m_Rr   .setZero();

  m_A .setZero();
  m_Bh.setZero();
  m_Br.setZero();

  m_B.resize(m_Bh.rows(), m_Bh.cols()+m_Br.cols());


  urdf::Model urdf_model;
  if ( !urdf_model.initParam ( "/robot_description" ) ) {
      ROS_ERROR ( "Urdf robot_description '%s' does not exist", (  this->getControllerNamespace()+"/robot_description" ).c_str() );
      return false;
  }
  Eigen::Vector3d gravity;
  gravity << 0, 0, -9.806;

  m_chain_bs = rosdyn::createChain ( urdf_model,"ur5_base_link","robotiq_ft_frame_id",gravity );
  // =========================== INIT VAR END

  std::vector<double> wrench_deadband(6,0);
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "wrench_deadband", wrench_deadband, 6, "<=" );
  m_wrench_deadband   = Eigen::Vector6d( wrench_deadband.data() );

  GET_AND_DEFAULT(this->getControllerNh(),"use_filtered_wrench",m_use_filtered_wrench,false);

  {
      double omega;
      GET_AND_DEFAULT(this->getControllerNh(),"omega_wrench",omega,10.0);
      ect::FilteredVectorXd::Value dead_band;
      ect::FilteredVectorXd::Value saturation;
      ect::FilteredVectorXd::Value init_value;

      dead_band  = m_wrench_deadband;
      saturation = 10.0 * dead_band;
      init_value = 0.0 * dead_band;
      if(!m_wrench_fitler.activateFilter ( dead_band, saturation, (omega / (2 * M_PI)), this->m_sampling_period, init_value ))
      {
        CNR_RETURN_FALSE(this->logger());
      }
  }
  m_w_b_filt = m_wrench_fitler.getUpdatedValue();






  std::vector<double> M_r(6,0), D_r(6,0), K_r(6,0);
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "M_r", M_r, 6 , "<=" );
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "K_r", K_r, 6 , "<"  );
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "D_r", D_r, 6 , "<"  );

  bool is_damping_ratio;
  GET_AND_RETURN( this->getControllerNh(), "damping_is_ratio", is_damping_ratio);


  Eigen::Vector6d M = Eigen::Vector6d( M_r.data() );
  Eigen::Vector6d K = Eigen::Vector6d( K_r.data() );

  Eigen::Vector6d D;
  if (is_damping_ratio)
      for (int i=0; i<D_r.size();i++)
        D(i) = D_r.data()[i] * 2 * std::sqrt( M_r.data()[i] * K_r.data()[i] );
  else
      D = Eigen::Vector6d( D_r.data() );

  CNR_FATAL(this->logger(),"damping: "<<D.transpose());

  m_mass = M(0);
  m_damping = D(0);
  m_stiffness = K(0);

 std::vector<double> Q_hat(6,0), R_hat(6,0), Qr(6,0), Rr(6,0);
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "Q_hat", Q_hat, 4 , "<" );
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "R_hat", R_hat, 4 , "<" );
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "Qr"   , Qr   , 4 , "<" );
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "Rr"   , Rr   , 4 , "<" );

  m_Q_hat.diagonal() << Eigen::Vector4d( Q_hat.data() );
  m_R_hat.diagonal() << Eigen::Vector4d( R_hat.data() );
  m_Qr   .diagonal() << Eigen::Vector4d( Qr.data() );
  m_Rr   .diagonal() << Eigen::Vector4d( Rr.data() );

  Eigen::Vector2d one(1,1);
  m_A.block(0,2,2,2) = one.asDiagonal();

  Eigen::Vector2d stiff;
  stiff << -K(0)/M(0), -K(1)/M(1);
  m_A.block(2,0,2,2) = stiff.asDiagonal();

  Eigen::Vector2d damp;
  damp << -D(0)/M(0), -D(1)/M(1);
  m_A.block(2,2,2,2) = damp.asDiagonal();

  Eigen::Vector2d bb;
  bb << 1/M(0), 1/M(1);
  m_Bh.block(2,0,2,2) = bb.asDiagonal();
  m_Br.block(2,0,2,2) = bb.asDiagonal();

  m_B << m_Bh, m_Br;

  m_C.resize(2,4);
  m_C.setZero();
  m_C.block(0,0,2,2) = one.asDiagonal();


  m_D.resize(2,4);
  m_D.setZero();

  m_S.resize(m_A.rows()+m_C.rows(),m_A.cols()+m_B.cols());
  m_S << m_A,m_B,
         m_C,m_D;

  ROS_FATAL_STREAM("\n"<<m_S);
  m_w_b_init = false;

  GET_AND_RETURN(this->getControllerNh(), "rho", m_rho);
  GET_AND_RETURN(this->getControllerNh(), "omega", m_omega);

  GET_AND_RETURN(this->getControllerNh(), "sigmoid_width" , m_width );
  GET_AND_RETURN(this->getControllerNh(), "sigmoid_half_x", m_half_x);
  GET_AND_RETURN(this->getControllerNh(), "sigmoid_height", m_height);
  GET_AND_RETURN(this->getControllerNh(), "sigmoid_max_y" , m_max_y );

//  gripper_action_.reset(new actionlib::SimpleActionClient<control_msgs::GripperCommandAction>(gripper_name, true));
//  CNR_INFO(this->logger(),"Waiting for "<<gripper_name<<" action server to start.");
//  gripper_action_->waitForServer();


  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief GtTrajDeformation::doStarting
 * @param time
 */
bool GtTrajDeformation::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");

  m_q_sp = this->getPosition();

  Eigen::Vector3d x = this->chainState().toolPose().translation();

  T_b_t_ = this->chainState().toolPose();

  m_X << x[0],x[1],0,0;

  ROS_FATAL_STREAM("m_X 1: "<<m_X.transpose()<<"\n\n\n\n\n\n\n\n\n");
  m_X_zero = m_X;
  m_X_ref = m_X;
  m_dX << 0,0,0,0;
  m_init_time = 0;

  m_dq_sp = 0 * this->getVelocity();

  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief GtTrajDeformation::stopping
 * @param time
 */
bool GtTrajDeformation::doStopping(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Stopping Controller");
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief GtTrajDeformation::doUpdate
 * @param time
 * @param period
 * @return
 */
bool GtTrajDeformation::doUpdate(const ros::Time& /*time*/, const ros::Duration& period)
{
//    auto start = std::chrono::steady_clock::now();

  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  std::stringstream report;

  std::lock_guard<std::mutex> lock(m_mtx);
//  rosdyn::VectorXd dq_sp = m_dq_sp;
  rosdyn::VectorXd q_sp = m_q_sp;

  ROS_DEBUG_STREAM_THROTTLE(.2,"ext wrench: "<<m_w_b.transpose());

  Eigen::Vector2d human_wrench;
  if (m_use_filtered_wrench)
    human_wrench << m_w_b_filt(0), m_w_b_filt(1);
  else
    human_wrench << m_w_b(0), m_w_b(1);

  double norm_wrench = human_wrench.norm();

  double alpha = sigma(norm_wrench);

  Eigen::MatrixXd A = m_A;
  Eigen::MatrixXd B = m_B;
  Eigen::MatrixXd Q = alpha*(m_Q_hat)+(1-alpha)*m_Qr;
  Eigen::MatrixXd R = alpha*(m_R_hat)+(1-alpha)*m_Rr;
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(4, 4);

  solveRiccati(A, B, Q, R, P);

  Eigen::MatrixXd K = R.inverse()*B.transpose()*P;

  m_init_time += period.toSec();
  m_X_ref(0) = m_X_zero(0) + m_rho*sin(m_omega*(m_init_time));
  m_X_ref(1) = m_X_zero(1) + m_rho*cos(m_omega*(m_init_time));

  rosdyn::VectorXd Xi = m_X-m_X_zero;

  Eigen::Vector4d ufb = -K*(Xi);

  double var_K = m_stiffness + K(2,0);
  double var_D = m_damping   + K(2,2);

  Eigen::Vector4d one(1,1,1,1);
  Eigen::MatrixXd eye = one.asDiagonal();

  Eigen::MatrixXd kk;
  kk.resize(K.rows(),K.cols()+eye.cols());
  kk << K, eye;

  Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> cqr(m_S);
  Eigen::MatrixXd pinS= cqr.pseudoInverse();

  Eigen::Matrix<double, 6, 2> g;

  g << 0,0,
       0,0,
       0,0,
       0,0,
       1,0,
       0,1;

  Eigen::VectorXd uff = kk*pinS*g*(m_X_ref - m_X_zero);

  Eigen::Vector2d u_shr;
  u_shr << uff.segment(0,2)+uff.segment(2,2);

//  Eigen::Vector2d uh;
//  uh(0) = ufb(0) + (1 - alpha)*u_shr(0);
//  uh(1) = ufb(1) + (1 - alpha)*u_shr(1);

//  Eigen::Vector2d ur;
//  ur(0) = ufb(2) + alpha*u_shr(0);
//  ur(1) = ufb(3) + alpha*u_shr(1);


  Eigen::Vector2d uh;
  uh(0) = ufb(0) + uff(0);
  uh(1) = ufb(1) + uff(1);

  Eigen::Vector2d ur;
  ur(0) = ufb(2) + uff(2);
  ur(1) = ufb(3) + uff(3);


  m_dX = m_A*Xi + m_Bh*human_wrench+ m_Br*ur;
  Xi = Xi + m_dX * period.toSec();

  m_X = Xi + m_X_zero;

  Eigen::Vector6d dx;
  dx.setZero();
  dx(0) = m_dX(0);
  dx(1) = m_dX(1);

//  Eigen::Matrix6Xd J_b = this->chainState().jacobian();

//  Eigen::FullPivLU<Eigen::MatrixXd> pinv_J ( J_b );
//  pinv_J.setThreshold ( 1e-2 );

//  if(pinv_J.rank()<6)
//  {
//    CNR_FATAL(this->logger(),"rank: "<<pinv_J.rank()<<"\nJacobian\n"<<J_b);
//  }

//  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
//  if (svd.singularValues()(svd.cols()-1)==0)
//    ROS_WARN_THROTTLE(1,"SINGULARITY POINT");
//  else if (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2)
//    ROS_WARN_THROTTLE(1,"SINGULARITY POINT");




//  Eigen::VectorXd sol(this->jointNames().size());
//  Eigen::Vector3d trasl = T_b_t_.translation();
//  trasl(0) = m_X(0);
//  trasl(1) = m_X(1);
//  T_b_t_.translation() = trasl;
//  Eigen::VectorXd seed = this->chainState().q();

//  m_chain.computeLocalIk(sol, T_b_t_, seed);
//  CNR_FATAL_THROTTLE(this->logger(),0.5,"-----ik "<<sol.transpose());
//  rosdyn::VectorXd dq_sp(this->jointNames().size());
//  q_sp = sol;
//  dq_sp = (q_sp - m_q_sp)/ period.toSec();










  //  Eigen::Vector6d cart_acc_of_t_in_b;
  //  cart_acc_of_t_in_b.setZero();
  //  cart_acc_of_t_in_b(0) = m_dX(2);
  //  cart_acc_of_t_in_b(1) = m_dX(3);

  //  Eigen::Vector6d cart_acc_nl_of_t_in_b  = m_chain.getDTwistNonLinearPartTool(m_q_sp,m_dq_sp);

  //  rosdyn::VectorXd ddq_sp  = svd.solve(cart_acc_of_t_in_b-cart_acc_nl_of_t_in_b);
  //  rosdyn::VectorXd dq_sp = m_dq_sp  + ddq_sp  * period.toSec();







  //  rosdyn::VectorXd dq_sp = pinv_J.solve(dx);



  Eigen::Matrix6Xd J_b = this->chainState().jacobian();
  Eigen::FullPivLU<Eigen::MatrixXd> pinv_J ( J_b );
  pinv_J.setThreshold ( 1e-2 );

  if(pinv_J.rank()<6)
  {
    CNR_FATAL(this->logger(),"rank: "<<pinv_J.rank()<<"\nJacobian\n"<<J_b);
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
  if (svd.singularValues()(svd.cols()-1)==0)
    ROS_WARN_THROTTLE(1,"SINGULARITY POINT");
  else if (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2)
    ROS_WARN_THROTTLE(1,"SINGULARITY POINT");



    rosdyn::VectorXd dq_sp = svd.solve(dx);
    q_sp = m_q_sp  + dq_sp  * period.toSec();

    for(int i=0;i<7;i++)
    {

        J_b = m_chain.getJacobian(q_sp);
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
        if (svd.singularValues()(svd.cols()-1)==0)
          ROS_WARN_THROTTLE(1,"SINGULARITY POINT");
        else if (svd.singularValues()(0)/svd.singularValues()(svd.cols()-1) > 1e2)
          ROS_WARN_THROTTLE(1,"SINGULARITY POINT");

        dq_sp = svd.solve(dx);
        q_sp = m_q_sp  + dq_sp  * period.toSec();
    }


    //TODO:: iterare integrazione



    m_q_sp  = q_sp;
    m_dq_sp  = dq_sp;
    this->setCommandPosition( q_sp );
    this->setCommandVelocity( dq_sp);

  //------------------------------------------
    Eigen::Vector3d x_curr = this->chainState().toolPose().translation();
    Eigen::Vector3d sp = m_chain.getTransformation(q_sp).translation();

    geometry_msgs::PoseStamped ref_pos;
    geometry_msgs::PoseStamped cur_pos;
    geometry_msgs::PoseStamped sp_pos;
    geometry_msgs::PoseStamped traj_pos;

    ros::Time stamp = ros::Time::now();

    ref_pos.header.frame_id = "ur5_base_link";
    ref_pos.header.stamp = stamp;
    ref_pos.pose.position.x = m_X(0);
    ref_pos.pose.position.y = m_X(1);

    cur_pos.header.frame_id = "ur5_base_link";
    cur_pos.header.stamp = stamp;
    cur_pos.pose.position.x = x_curr(0);
    cur_pos.pose.position.y = x_curr(1);

    sp_pos.header.frame_id = "ur5_base_link";
    sp_pos.header.stamp = stamp;
    sp_pos.pose.position.x = sp(0);
    sp_pos.pose.position.y = sp(1);

    traj_pos.header.frame_id = "ur5_base_link";
    traj_pos.header.stamp = stamp;
    traj_pos.pose.position.x = m_X_ref(0);
    traj_pos.pose.position.y = m_X_ref(1);

    std_msgs::Float32 alpha_msg;
    alpha_msg.data = alpha;

    std_msgs::Float32 hw_msg;
    hw_msg.data = norm_wrench;

    std_msgs::Float32 D_msg;
    D_msg.data = var_D;

    std_msgs::Float32 K_msg;
    K_msg.data = var_K;

    sensor_msgs::JointState joint_sp_msg;

    std::vector<double> jvec;
    for (int i=0; i<this->jointNames().size();i++)
    {
        jvec.push_back( q_sp(i) );
    }

    joint_sp_msg.name = this->jointNames();
    joint_sp_msg.position = jvec;
    joint_sp_msg.header.stamp = stamp;

    geometry_msgs::WrenchStamped hu_msg;
    geometry_msgs::WrenchStamped ru_msg;

    hu_msg.header.stamp = stamp;
    ru_msg.header.stamp = stamp;

    hu_msg.wrench.force.x = uh(0);
    hu_msg.wrench.force.y = uh(1);

    ru_msg.wrench.force.x = ur(0);
    ru_msg.wrench.force.y = ur(1);

    this->publish(cart_pos_ref_pub ,ref_pos);
    this->publish(cart_pos_cur_pub ,cur_pos);
    this->publish(cart_pos_sp_pub  ,sp_pos);
    this->publish(cart_pos_traj_pub,traj_pos);
    this->publish(joint_sp_pub     ,joint_sp_msg);
    this->publish(alpha_pub        ,alpha_msg);
    this->publish(human_wrench_pub ,hw_msg);
    this->publish(human_u_pub      ,hu_msg);
    this->publish(robot_u_pub      ,ru_msg);
    this->publish(D_pub            ,D_msg);
    this->publish(K_pub            ,K_msg);



  //  auto end = std::chrono::steady_clock::now();
  //  CNR_FATAL_THROTTLE(this->logger(),0.1,"time: "<<std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());

    CNR_RETURN_TRUE_THROTTLE_DEFAULT(this->logger());

  }

  /**
   * @brief GtTrajDeformation::callback
   * @param msg
   */
  void GtTrajDeformation::callback(const geometry_msgs::WrenchStampedConstPtr& msg )
  {
    if(!m_w_b_init)
    {
      m_w_b_0 ( 0 ) = msg->wrench.force.x;
      m_w_b_0 ( 1 ) = msg->wrench.force.y;
      m_w_b_0 ( 2 ) = msg->wrench.force.z;
      m_w_b_0 ( 3 ) = msg->wrench.torque.x;
      m_w_b_0 ( 4 ) = msg->wrench.torque.y;
      m_w_b_0 ( 5 ) = msg->wrench.torque.z;

      m_w_b_init = true;
    }

    Eigen::Vector6d wrench_s;
    wrench_s( 0 ) = msg->wrench.force.x  - m_w_b_0 ( 0 );
    wrench_s( 1 ) = msg->wrench.force.y  - m_w_b_0 ( 1 );
    wrench_s( 2 ) = msg->wrench.force.z  - m_w_b_0 ( 2 );
    wrench_s( 3 ) = msg->wrench.torque.x - m_w_b_0 ( 3 );
    wrench_s( 4 ) = msg->wrench.torque.y - m_w_b_0 ( 4 );
    wrench_s( 5 ) = msg->wrench.torque.z - m_w_b_0 ( 5 );

    Eigen::Affine3d T_bs = m_chain_bs->getTransformation ( this->getPosition() );
    Eigen::Affine3d T_bt = m_chain    .getTransformation ( this->getPosition() );
    Eigen::Affine3d T_ts = T_bt.inverse() * T_bs;
    Eigen::Vector6d w_t = rosdyn::spatialDualTranformation ( wrench_s , T_ts );
    Eigen::Vector6d wrench;
    wrench = rosdyn::spatialRotation ( w_t, T_bt.linear() );

    for ( unsigned int idx=0; idx<6; idx++ )
    {
      if ( ( wrench ( idx ) >m_wrench_deadband ( idx ) ) )
      {
          m_w_b ( idx ) = wrench ( idx )-m_wrench_deadband ( idx );
      }
      else if ( ( wrench ( idx ) <-m_wrench_deadband ( idx ) ) )
      {
          m_w_b ( idx ) = wrench ( idx )+m_wrench_deadband ( idx );
      }
      else
      {
          m_w_b ( idx ) =0;
      }
    }

    geometry_msgs::WrenchStamped tool_w;

    tool_w.header.frame_id = "robotiq_ft_frame_id";
    tool_w.header.stamp = ros::Time::now();
    tool_w.wrench.force.x  = wrench_s( 0 );
    tool_w.wrench.force.y  = wrench_s( 1 );
    tool_w.wrench.force.z  = wrench_s( 2 );
    tool_w.wrench.torque.x = wrench_s( 3 );
    tool_w.wrench.torque.y = wrench_s( 4 );
    tool_w.wrench.torque.z = wrench_s( 5 );

    geometry_msgs::WrenchStamped base_w;

    base_w.header.frame_id = "ur5_base_link";
    base_w.header.stamp = ros::Time::now();
    base_w.wrench.force.x  = m_w_b( 0 );
    base_w.wrench.force.y  = m_w_b( 1 );
    base_w.wrench.force.z  = m_w_b( 2 );
    base_w.wrench.torque.x = m_w_b( 3 );
    base_w.wrench.torque.y = m_w_b( 4 );
    base_w.wrench.torque.z = m_w_b( 5 );

    geometry_msgs::WrenchStamped filter_base_w;

    m_wrench_fitler.update(wrench);
    m_w_b_filt = m_wrench_fitler.getUpdatedValue();

    filter_base_w.header.frame_id = "ur5_base_link";
    filter_base_w.header.stamp = ros::Time::now();
    filter_base_w.wrench.force.x  = m_w_b_filt( 0 );
    filter_base_w.wrench.force.y  = m_w_b_filt( 1 );
    filter_base_w.wrench.force.z  = m_w_b_filt( 2 );
    filter_base_w.wrench.torque.x = m_w_b_filt( 3 );
    filter_base_w.wrench.torque.y = m_w_b_filt( 4 );
    filter_base_w.wrench.torque.z = m_w_b_filt( 5 );


    this->publish(wrench_base_pub,base_w);
    this->publish(filtered_wrench_base_pub,filter_base_w);
    this->publish(wrench_tool_pub,tool_w);

    if (wrench_s(5)>1.5)
    {
        control_msgs::GripperCommandActionGoal goal;
        goal.goal.command.position = -0.01;
        goal.goal.command.max_effort = 30.0;
        this->publish(activate_gripper_pub,goal);
    }
    else if (wrench_s(5)<-1.5)
    {
        control_msgs::GripperCommandActionGoal goal;
        goal.goal.command.position = 0.08;
        goal.goal.command.max_effort = 30.0;
        this->publish(activate_gripper_pub,goal);
    }

  }


  double GtTrajDeformation::sigma(double x)
  {
    return m_max_y -(m_height/(1+exp(-m_width*(x-m_half_x))));
  }

  bool GtTrajDeformation::solveRiccati(const Eigen::MatrixXd &A,
                                 const Eigen::MatrixXd &B,
                                 const Eigen::MatrixXd &Q,
                                 const Eigen::MatrixXd &R, Eigen::MatrixXd &P)
  {

    const uint dim_x = A.rows();
    const uint dim_u = B.cols();

    Eigen::MatrixXd Ham = Eigen::MatrixXd::Zero(2 * dim_x, 2 * dim_x);
    Ham << A, -B * R.inverse() * B.transpose(), -Q, -A.transpose();

    Eigen::EigenSolver<Eigen::MatrixXd> Eigs(Ham);

    Eigen::MatrixXcd eigvec = Eigen::MatrixXcd::Zero(2 * dim_x, dim_x);
    int j = 0;
    for (int i = 0; i < 2 * dim_x; ++i) {
      if (Eigs.eigenvalues()[i].real() < 0.) {
        eigvec.col(j) = Eigs.eigenvectors().block(0, i, 2 * dim_x, 1);
        ++j;
      }
    }

    Eigen::MatrixXcd Vs_1, Vs_2;
    Vs_1 = eigvec.block(0, 0, dim_x, dim_x);
    Vs_2 = eigvec.block(dim_x, 0, dim_x, dim_x);
    P = (Vs_2 * Vs_1.inverse()).real();

    return true;
  }


  }
  }
