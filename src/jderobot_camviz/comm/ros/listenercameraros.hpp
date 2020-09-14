#include "ros/ros.h"
#include "std_msgs/String.h"  
#include </opt/jderobot/include/jderobot/types/image.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <boost/thread/thread.hpp>
#include "translators.hpp"

#include <iostream>  
//#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <time.h>


namespace camViz {
      ///*********Class ListenerCamera*****************///////

	class ListenerCamera {
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
			std::string  nodeName;
		protected:
			JdeRobotTypes::Image image;
			int refreshRate;
			bool on;
			int cont = 0; //used to count Frames per seconds
			time_t timer; // used to save time for FPS

			ros::AsyncSpinner* spinner;
			
			
		void imagecallback (const sensor_msgs::ImageConstPtr& image_msg);
	};

	ListenerCamera::ListenerCamera(int argc, char** argv, std::string nodeName, std::string topic){
		pthread_mutex_init(&mutex, NULL);
		if ("" == topic){
			this->on = false;
			std::cerr <<"Invalid camera topic" <<std::endl;
		}else{
			this->on = true;
			this->topic = topic;
			this->nodeName = nodeName;

			const std::string name = std::string(this->nodeName);

			time(&timer);
			int a = 0;
			ros::init(a, nullptr, name);
			ros::NodeHandle nh;
			this->sub = nh.subscribe(this->topic, 1001, &ListenerCamera::imagecallback, this);
			std::cout << "listen from "+ this->topic << std::endl;

			this->spinner = new ros::AsyncSpinner(1);
		}
	}



	ListenerCamera::~ListenerCamera(){
		this->stop();
	}

	 void 
	ListenerCamera::start(){
		this->spinner->start();

	}

	void 
	ListenerCamera::stop(){
		this->spinner->stop();
		ros::shutdown();
	}

	void 
	ListenerCamera::imagecallback(const sensor_msgs::ImageConstPtr& image_msg){
		this->cont++;
		time_t now;
		time(&now);
		pthread_mutex_lock(&mutex);
		//this->image = Comm::translate_image_messages(image_msg);
    this->image = translate_image_messages(image_msg);

		if (difftime(this->timer, now)>=1){
			this->refreshRate = this->cont;
			this->cont = 0;
			this->timer = now;
		}
		pthread_mutex_unlock(&mutex);

	}

	JdeRobotTypes::Image  ListenerCamera::getImage(){
		JdeRobotTypes::Image img;
		pthread_mutex_lock(&mutex);
		img = this->image;
		pthread_mutex_unlock(&mutex);
		return img;
	}

	int ListenerCamera::getRefreshRate(){

		int rr;
		pthread_mutex_lock(&mutex);
		rr = this->refreshRate;
		pthread_mutex_unlock(&mutex);

		return rr;
	}

}

