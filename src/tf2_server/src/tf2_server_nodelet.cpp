#include <tf2_server/tf2_server.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace tf2_server {

class Tf2ServerNodelet: public nodelet::Nodelet {
  private: std::unique_ptr<TF2Server> tf2Server;
  public: void onInit() override
  {
    tf2Server = std::make_unique<TF2Server>(this->getMTNodeHandle(), this->getMTPrivateNodeHandle());
    tf2Server->start();
  }
};

}

PLUGINLIB_EXPORT_CLASS(tf2_server::Tf2ServerNodelet, nodelet::Nodelet)
