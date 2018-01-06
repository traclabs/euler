/**
 * Put copyright notice here
 */
#include <euler_gazebo/euler_path_follower.hpp>

namespace gazebo {
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(EulerPathFollowerPlugin)
  
  EulerPathFollowerPlugin::EulerPathFollowerPlugin()
      : ModelPlugin()
      , m_world(NULL)
      , m_updatePeriod(0.01)
      // , m_engageTimeout(1.0)
      , m_modelName("euler")
      , m_model(NULL)
      // , m_threshTvel(0.001)
      // , m_threshRvel(0.01)
      // , m_engaged(true)
  {
    if (!ros::isInitialized()) {
      ROS_FATAL_STREAM("EulerPathFollowerPlugin: not initialized, unable to load");
      return;
    }

    m_xyz.x = 0.0;
    m_xyz.y = 0.0;
    m_xyz.z = 0.0;

    m_rpy.x = 0.0;
    m_rpy.y = 0.0;
    m_rpy.z = 0.0;

  }
  
  // Called by the world update start event
  void EulerPathFollowerPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf) {
    ROS_INFO("EulerPathFollowerPlugin: begin loading...");
    this->m_world = parent->GetWorld();
    this->m_sdf   = sdf;
    
    if (this->m_sdf->HasElement("update_period")) {
       m_updatePeriod = this->m_sdf->Get<double>("update_period");
    } else {
     ROS_WARN("EulerPathFollowerPlugin::Load:: no update specified; default %f", m_updatePeriod);
    }

    // if (this->m_sdf->HasElement("id")) {
  //      m_modelName = this->m_sdf->Get<std::string>("id");
    // } else {
    //  ROS_WARN("EulerPathFollowerPlugin::Load:: no ID specified; default %s", m_modelName.c_str());
    // }
    // if (this->m_sdf->HasElement("thresh_vel_trans")) {
  //      m_threshTvel = this->m_sdf->Get<double>("thresh_vel_trans");
    // } else {
    //  ROS_WARN("EulerPathFollowerPlugin::Load:: no v threshold specified; default %f", m_threshTvel);
    // }
    // if (this->m_sdf->HasElement("thresh_vel_rot")) {
  //      m_threshRvel = this->m_sdf->Get<double>("thresh_vel_rot");
    // } else {
    //  ROS_WARN("EulerPathFollowerPlugin::Load:: no omega threshold specified; default %f", m_threshRvel);
    // }

    std::string cmdVelTopic("/cmd_vel");
    if (this->m_sdf->HasElement("cmd_vel_topic")) {
       cmdVelTopic = this->m_sdf->Get<std::string>("cmd_vel_topic");
    } else {
     ROS_WARN("EulerPathFollowerPlugin::Load:: no velocity topic specified; default %s", cmdVelTopic.c_str());
    }
    
    // TODO: it appears that the m_model is causing gazebo to crash!!!
    m_model = m_world->GetModel(m_modelName);
    if (m_model != NULL) {
      m_pose = m_model->GetWorldPose();
      ROS_INFO("EulerPathFollowerPlugin: model %s, pos (%f, %f, %f)", m_modelName.c_str(), m_pose.pos.x, m_pose.pos.y, m_pose.pos.z);
    } else {
      ROS_ERROR("EulerPathFollowerPlugin: failed to get model %s", m_modelName.c_str());
    }
    
    ros::NodeHandle nh;
    m_velSub = nh.subscribe<geometry_msgs::Twist>(cmdVelTopic, 1,
                  boost::bind(&EulerPathFollowerPlugin::cmdVelCb, this, _1));
    m_lastUpdateTime = getWorldTime();
    m_lastCmdVelTime = m_lastUpdateTime;
    this->m_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&EulerPathFollowerPlugin::OnUpdate, this, _1));
    ROS_INFO("EulerPathFollowerPlugin: finished loading...");
  }
  

  void EulerPathFollowerPlugin::OnUpdate(const common::UpdateInfo &info) {
    


  }

  void EulerPathFollowerPlugin::cmdVelCb( const geometry_msgs::Twist::ConstPtr &_msg ) {
    m_velMsg = *_msg;
    //m_lastCmdVelTime = getWorldTime();

    ///////////////////////////////////////////////////////////////////
    m_currentTime = getWorldTime();
    ros::Duration elapsed = m_currentTime - m_lastUpdateTime;
    double thresh_vel_z = 0.001;
    double dt = elapsed.toSec();

    if( dt > 1.0 ) { m_lastUpdateTime = m_currentTime; return; }
    if (dt > m_updatePeriod) {

      //m_velMsg.linear.x + m_velMsg.linear.y +   
      m_xyz.x += (m_velMsg.linear.x*cos(m_rpy.z))*dt;
      m_xyz.y += (m_velMsg.linear.x*sin(m_rpy.z))*dt;
      if( fabs(m_velMsg.linear.z) > thresh_vel_z ) {
        m_xyz.z += m_velMsg.linear.z*dt;
      }
      m_rpy.z += m_velMsg.angular.z*dt;
    
      // make a pose msg so we can set the gazebo model
      m_pose.Set(m_xyz.x, m_xyz.y, m_xyz.z, m_rpy.x, m_rpy.y, m_rpy.z);

      m_model->SetWorldPose(m_pose, true, true);

      ROS_WARN_THROTTLE(1, "EulerPathFollowerPlugin() -- updating: [%.3f, %.3f, %.3f], [%.3f, %.3f, %.3f]", 
        m_xyz.x, m_xyz.y, m_xyz.z, m_rpy.x, m_rpy.y, m_rpy.z);
    
      // if (elapsed.toSec() > m_engageTimeout && !m_engaged) {
      //   ROS_WARN("EulerPathFollowerPlugin: velocity timeout, engaging!");
      //   m_pose = m_model->GetWorldPose();
      //   m_engaged = true;
      // }
      // // TODO: mutex
      // if (m_engaged) {
      //   ROS_DEBUG_THROTTLE(2, "EulerPathFollowerPlugin: %s set pose to (%f, %f, %f) (%f, %f, %f, %f)",
      //            m_modelName.c_str(), m_pose.pos.x, m_pose.pos.y, m_pose.pos.z, m_pose.rot.x, m_pose.rot.y, m_pose.rot.z, m_pose.rot.w);
      //   m_model->SetWorldPose(m_pose, true, true);
      // } else {
      //   math::Pose pose = m_model->GetWorldPose();
      //   ROS_DEBUG_THROTTLE(2, "EulerPathFollowerPlugin: %s pose is (%f, %f, %f) (%f, %f, %f, %f)",
      //            m_modelName.c_str(), pose.pos.x, pose.pos.y, pose.pos.z, pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
      // }
      m_lastUpdateTime = m_currentTime;
    }

    
    // tf_base.setOrigin( tf::Vector3(m_xyz.x, m_xyz.y, m_xyz.z) );

    ///////////////////////////////////////////////////////////////////


  }


  
  // using this for debug; maybe useful to have?
  //bool EulerPathFollowerPlugin::toggleBrakeCb(std_srvs::Empty::Request &req,
  //                                     std_srvs::Empty::Response &rsp) {
  //  m_engaged = !m_engaged;
  //}
  
  // bool EulerPathFollowerPlugin::within(double val, double set, double thresh) {
  //   return (fabs(set - val) < thresh) && (fabs(val - set) < thresh);
  // }
  
  ros::Time EulerPathFollowerPlugin::getWorldTime() {
    if (this->m_world == NULL) return ros::Time(0);
    common::Time gtime = this->m_world->GetSimTime();
    return ros::Time(gtime.Double());
  }
  
}
