#include <gt_traj_deformation/gt_traj_deformation.h>
#include <state_space_filters/filtered_values.h>
#include <eigen_matrix_utils/overloads.h>
#include <geometry_msgs/PoseStamped.h>
#include <pluginlib/class_list_macros.h>
#include <Eigen/Dense>

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
  
  wrench_base_pub = this->template add_publisher<geometry_msgs::WrenchStamped>("wrench_base",5);
  wrench_tool_pub = this->template add_publisher<geometry_msgs::WrenchStamped>("wrench_tool",5);

  cart_pos_ref_pub = this->template add_publisher<geometry_msgs::PoseStamped>("pose_ref",5);
  cart_pos_cur_pub = this->template add_publisher<geometry_msgs::PoseStamped>("pose_cur",5);
  cart_pos_err_pub = this->template add_publisher<geometry_msgs::PoseStamped>("pose_err",5);


  this->setPriority(this->QD_PRIORITY);

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
  m_vel_sp = m_vel_fitler_sp.getUpdatedValue();
  m_pos_sp = this->getPosition();
  m_pos = this->getPosition();

  m_has_pos_sp = false;
  
  m_wrench_deadband .setZero();
  m_w_b             .setZero();
  
  // system params initi
  m_X .resize(4);
  m_dX.resize(4);
  
  m_M.setZero();
  m_D.setZero();
  m_K.setZero(); 
  
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

  std::vector<double> M_r(6,0), D_r(6,0), K_r(6,0);
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "M_r", M_r, 6 , "<=" ); 
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "K_r", K_r, 6 , "<"  ); 
  GET_PARAM_VECTOR_AND_RETURN ( this->getControllerNh(), "D_r", D_r, 6 , "<"  ); 
  
  m_M = Eigen::Vector6d( M_r.data() );
  m_D = Eigen::Vector6d( D_r.data() );
  m_K = Eigen::Vector6d( K_r.data() );
  
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
  stiff << -m_K(0)/m_M(0), -m_K(1)/m_M(1);
  m_A.block(2,0,2,2) = stiff.asDiagonal();

  Eigen::Vector2d damp;
  damp << -m_D(0)/m_M(0), -m_D(1)/m_M(1);
  m_A.block(2,2,2,2) = damp.asDiagonal();

  ROS_FATAL_STREAM(m_A);
//  m_A(0,2) = 1;
//  m_A(1,3) = 1;
//  m_A(2,0) = -m_K(0)/m_M(0);
//  m_A(3,1) = -m_K(1)/m_M(1);
//  m_A(2,2) = -m_D(0)/m_M(0);
//  m_A(3,3) = -m_D(1)/m_M(1);
  
  m_Bh(2,0) = 1/m_M(0);
  m_Bh(3,1) = 1/m_M(1);
  
  m_Br(2,0) = 1/m_M(0);
  m_Br(3,1) = 1/m_M(1);
  
  m_B << m_Bh, m_Br;
  
  m_w_b_init = false;
  
  GET_AND_RETURN(this->getControllerNh(), "rho", m_rho);
  GET_AND_RETURN(this->getControllerNh(), "omega", m_omega);

  GET_AND_RETURN(this->getControllerNh(), "sigmoid_width", m_width);
  GET_AND_RETURN(this->getControllerNh(), "sigmoid_offset", m_offset);

  
  CNR_RETURN_TRUE(this->logger());
}

/**
 * @brief GtTrajDeformation::doStarting
 * @param time
 */
bool GtTrajDeformation::doStarting(const ros::Time& /*time*/)
{
  CNR_TRACE_START(this->logger(),"Starting Controller");
  m_pos_sp = this->getPosition();
  m_pos_init = m_pos_sp;
  Eigen::Vector3d x = m_chain.getTransformation(m_pos_init).translation();
  
  m_X << x[0],x[1],0,0;
  m_X_init = m_X;
  m_X_ref = m_X;
  m_dX << 0,0,0,0;
  m_init_time = 0;
  
  m_vel_sp = 0 * this->getVelocity();
  CNR_RETURN_TRUE(this->logger());
  m_vel_sp_last = m_vel_sp;  
  
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
  CNR_TRACE_START_THROTTLE_DEFAULT(this->logger());
  std::stringstream report;

  std::lock_guard<std::mutex> lock(m_mtx);
//  rosdyn::VectorXd vel_sp = m_vel_sp;
  rosdyn::VectorXd pos_sp = m_pos_sp;
  
  
  ROS_DEBUG_STREAM_THROTTLE(.2,"ext wrench: "<<m_w_b.transpose());
  
  Eigen::Vector2d human_wrench;
  human_wrench << m_w_b(0), m_w_b(1);
  double norm_wrench = human_wrench.norm();
//   ROS_FATAL_STREAM_THROTTLE(.2,"human wrench: "<<human_wrench.transpose());
//   ROS_FATAL_STREAM_THROTTLE(.2,"norm wrench: "<<norm_wrench);
  
  double alpha = sigma(norm_wrench);
     ROS_FATAL_STREAM_THROTTLE(.2,"alpha: "<<alpha);
  
  Eigen::MatrixXd A = m_A;
  Eigen::MatrixXd B = m_B;
  Eigen::MatrixXd Q = alpha*(m_Q_hat)+(1-alpha)*m_Qr;
  Eigen::MatrixXd R = alpha*(m_R_hat)+(1-alpha)*m_Rr;
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(4, 4);
  
  solveRiccatiArimotoPotter(A, B, Q, R, P);
  
  Eigen::MatrixXd K = R.inverse()*B.transpose()*P;
  
  m_init_time += period.toSec();
//  m_X_ref(0) = m_X_init(0) + m_rho*sin(m_omega*(m_init_time));
//  m_X_ref(1) = m_X_init(1) + m_rho*cos(m_omega*(m_init_time));
  m_X_ref(0) = m_X_init(0);
  m_X_ref(1) = m_X_init(1);
  
  Eigen::Vector4d u = -K*(m_X-m_X_ref);
  
  Eigen::Vector2d ur;
  ur(0) = u(2);
  ur(1) = u(3);
  
//   human_wrench <<0,0;
  
  m_dX = m_A*m_X + m_Bh*human_wrench+ m_Br*ur;
  m_X = m_X + m_dX * period.toSec();
  
  Eigen::Vector3d x = m_chain.getTransformation(m_pos_sp).translation();
//   ROS_FATAL_STREAM_THROTTLE(.2,"dX: "<<m_dX.transpose());
//   ROS_FATAL_STREAM_THROTTLE(.2,"X: "<<m_X.transpose());
//   ROS_FATAL_STREAM_THROTTLE(.2,"current_x: "<<x.transpose());
//   ROS_FATAL_STREAM_THROTTLE(.2,"x ref: "<m_X_ref.transpose());
  
  Eigen::Vector6d dx;
  dx.setZero();
  dx(0) = m_dX(0);
  dx(1) = m_dX(1);
  
  
  m_pos = this->getPosition();
  Eigen::Matrix6Xd J_b = m_chain.getJacobian ( m_pos );
  Eigen::FullPivLU<Eigen::MatrixXd> pinv_J ( J_b );
  pinv_J.setThreshold ( 1e-6 );
  
  rosdyn::VectorXd vel_sp = pinv_J.solve(dx);
  m_vel_fitler_sp.update(vel_sp);
  pos_sp = m_pos_sp  + m_vel_fitler_sp.getUpdatedValue()  * period.toSec();

  m_pos_sp  = pos_sp;
  m_vel_sp  = vel_sp;
  rosdyn::VectorXd q_error = m_pos_init - pos_sp;
//  ROS_FATAL_STREAM_THROTTLE(.2,"q_sp: "<<pos_sp.transpose());
//  ROS_FATAL_STREAM_THROTTLE(.2,"q_error: "<<q_error.transpose());
//   ROS_FATAL_STREAM_THROTTLE(.2,"vel_sp: "<<vel_sp.transpose());
  
  this->setCommandPosition( pos_sp );
  this->setCommandVelocity( m_vel_fitler_sp.getUpdatedValue() );


  rosdyn::VectorXd curr_q = this->getPosition();
//  ROS_FATAL_STREAM_THROTTLE(.2,"curr_q: "<<curr_q.transpose());

  Eigen::Vector3d x_curr = m_chain.getTransformation(curr_q).translation();
  Eigen::Vector3d init = m_chain.getTransformation(m_pos_init).translation();
  Eigen::Vector3d rrr = init - x_curr;
  Eigen::Vector3d x_sp = m_chain.getTransformation(pos_sp).translation();

    ROS_FATAL_STREAM_THROTTLE(.2,"x_sp: "<<x_sp.transpose());
  //  ROS_FATAL_STREAM_THROTTLE(.2,"x_curr: "<<x_curr.transpose());
//  ROS_FATAL_STREAM_THROTTLE(.2,"init: "<<init.transpose());
//  ROS_FATAL_STREAM_THROTTLE(.2,"rr: "<<rrr.transpose());
//  ROS_FATAL_STREAM_THROTTLE(.2,"X: "<<m_X.transpose());
//  ROS_FATAL_STREAM_THROTTLE(.2,"X ref: "<<m_X_ref.transpose());

  geometry_msgs::PoseStamped ref_pos;
  geometry_msgs::PoseStamped cur_pos;
  geometry_msgs::PoseStamped err_pos;

  ref_pos.header.frame_id = "ur5_base_link";
  ref_pos.header.stamp = ros::Time::now();
  ref_pos.pose.position.x = m_X(0);
  ref_pos.pose.position.y = m_X(1);

  cur_pos.header.frame_id = "ur5_base_link";
  cur_pos.header.stamp = ros::Time::now();
  cur_pos.pose.position.x = x_curr(0);
  cur_pos.pose.position.y = x_curr(1);

  err_pos.header.frame_id = "ur5_base_link";
  err_pos.header.stamp = ros::Time::now();
  err_pos.pose.position.x = m_X(0)-x_curr(0);
  err_pos.pose.position.y = m_X(1)-x_curr(1);

  this->publish(cart_pos_ref_pub,ref_pos);
  this->publish(cart_pos_cur_pub,cur_pos);
  this->publish(cart_pos_err_pub,err_pos);


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
  
  
  this->publish(wrench_base_pub,base_w);
  this->publish(wrench_tool_pub,tool_w);
  
  
  
}


double GtTrajDeformation::sigma(double x)
{
  double height = 0.98;
  return 1-(0.01 + height/(1+exp(-m_width*(x-m_offset))));
}

bool GtTrajDeformation::solveRiccatiArimotoPotter(const Eigen::MatrixXd &A,
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
