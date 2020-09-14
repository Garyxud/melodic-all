#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <boost/thread/thread.hpp>
#include </opt/jderobot/include/jderobot/types/pose3d.h>
#include "../interfaces/pose3dClient.hpp"
#include "translators.hpp"

namespace Comm {
	class ListenerPose: public Comm::Pose3dClient {
		
	public:
		ListenerPose(int argc, char** argv, std::string nodeName, std::string topic);
		~ListenerPose();

		void start();
		void stop();
		virtual JdeRobotTypes::Pose3d  getPose();
		

	private:
		pthread_mutex_t mutex;
		ros::Subscriber sub;
		std::string topic;
		std::string nodeName;

		ros::AsyncSpinner* spinner;
		
		void posecallback (const nav_msgs::OdometryConstPtr& odom_msg);

		



	};//class

} 
