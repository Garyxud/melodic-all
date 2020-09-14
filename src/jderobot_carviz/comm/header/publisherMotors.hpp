#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/thread/thread.hpp>
#include </opt/jderobot/include/jderobot/types/cmdvel.h>
#include "../interfaces/motorsClient.hpp"
#include "translators.hpp"

namespace Comm {
	class PublisherMotors: public Comm::MotorsClient {
		
	public:
		PublisherMotors(int argc, char** argv, std::string nodeName, std::string topic);
		~PublisherMotors();

		void start();
		void stop();
		
		virtual void sendVelocities(JdeRobotTypes::CMDVel vel);
		virtual void sendVX (float vx);
		virtual void sendVY (float vy);
		virtual void sendAZ (float az);
		virtual void sendV (float v);
		virtual void sendW (float w);
		virtual void sendL (float l );
		

	private:
		pthread_mutex_t mutex;
		ros::Publisher pub;
		std::string topic;
		std::string nodeName;

		ros::AsyncSpinner* spinner;
		ros::Timer timer_;
		bool forPublish;

		void publish();
		

	};//class

} 
