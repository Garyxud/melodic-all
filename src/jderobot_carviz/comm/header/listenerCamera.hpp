#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread/thread.hpp>
#include </opt/jderobot/include/jderobot/types/image.h>
#include "../interfaces/cameraClient.hpp"
#include "translators.hpp"
#include <time.h>

namespace Comm {
	class ListenerCamera: public Comm::CameraClient {
		
	public:
		ListenerCamera(int argc, char** argv, std::string nodeName, std::string topic);
		~ListenerCamera();

		void start();
		void stop();
		virtual JdeRobotTypes::Image getImage();
		virtual int getRefreshRate();
		

	private:
		pthread_mutex_t mutex;
		ros::Subscriber sub;
		std::string topic;
		std::string nodeName;

		int cont = 0; //used to count Frames per seconds
		time_t timer; // used to save time for FPS

		ros::AsyncSpinner* spinner;
		
		
		void imagecallback (const sensor_msgs::ImageConstPtr& image_msg);

		



	};//class

} 
