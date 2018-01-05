/**
 * Put copyright notice here
 */
#ifndef EULER_PATH_FOLLOWER_HPP
#define EULER_PATH_FOLLOWER_HPP

#include <ros/ros.h>

// #include <tf/tf.h>
 
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

#include <std_srvs/Empty.h>


#include <math.h>

namespace gazebo {
	
	class EulerPathFollowerPlugin : public ModelPlugin {
	public:
		EulerPathFollowerPlugin();
		~EulerPathFollowerPlugin() { }
		void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
		void OnUpdate(const common::UpdateInfo &info);
		
	private:

		// gazebo sim data
		physics::WorldPtr m_world;
		sdf::ElementPtr m_sdf;
		event::ConnectionPtr m_updateConnection;
		ros::Time m_lastUpdateTime, m_lastCmdVelTime, m_currentTime;
		double m_updatePeriod;
		// double m_engageTimeout;

		geometry_msgs::Twist m_velMsg;
		
		// tracbot data
		std::string m_modelName;
		physics::ModelPtr m_model;

		// double m_threshTvel;
		// double m_threshRvel;
		bool m_engaged;
		math::Pose m_pose;

		ros::Subscriber m_velSub;

		geometry_msgs::Vector3 m_xyz, m_rpy;

		//ros::ServiceServer m_toggleBrakeSrv;
		
		void cmdVelCb(const geometry_msgs::Twist::ConstPtr &msg);
		// bool toggleBrakeCb(std_srvs::Empty::Request &req,
		//                    std_srvs::Empty::Response &rsp);
		// bool within(double val, double set, double thresh);
		ros::Time getWorldTime();
	};
}
#endif
