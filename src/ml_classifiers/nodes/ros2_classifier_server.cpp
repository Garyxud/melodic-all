// Copyright (c) 2012, 2019 Scott Niekum, Joshua Whitley
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <pluginlib/class_loader.hpp>

#include <string>
#include <map>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "ml_classifiers/zero_classifier.hpp"
#include "ml_classifiers/nearest_neighbor_classifier.hpp"
#include "ml_classifiers/srv/add_class_data.hpp"
#include "ml_classifiers/srv/classify_data.hpp"
#include "ml_classifiers/srv/clear_classifier.hpp"
#include "ml_classifiers/srv/create_classifier.hpp"
#include "ml_classifiers/srv/load_classifier.hpp"
#include "ml_classifiers/srv/save_classifier.hpp"
#include "ml_classifiers/srv/train_classifier.hpp"

using namespace ml_classifiers;  // NOLINT
using std::string;
using std::cout;
using std::endl;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class ClassifierServer : public rclcpp::Node
{
public:
  ClassifierServer()
  : Node("classifier_server"),
    c_loader("ml_classifiers", "ml_classifiers::Classifier")
  {
    create_srv = this->create_service<ml_classifiers::srv::CreateClassifier>(
      "create_classifier",
      std::bind(&ClassifierServer::createCallback, this, _1, _2, _3));
    add_srv = this->create_service<ml_classifiers::srv::AddClassData>(
      "add_class_data",
      std::bind(&ClassifierServer::addCallback, this, _1, _2, _3));
    train_srv = this->create_service<ml_classifiers::srv::TrainClassifier>(
      "train_classifier",
      std::bind(&ClassifierServer::trainCallback, this, _1, _2, _3));
    clear_srv = this->create_service<ml_classifiers::srv::ClearClassifier>(
      "clear_classifier",
      std::bind(&ClassifierServer::clearCallback, this, _1, _2, _3));
    save_srv = this->create_service<ml_classifiers::srv::SaveClassifier>(
      "save_classifier",
      std::bind(&ClassifierServer::saveCallback, this, _1, _2, _3));
    load_srv = this->create_service<ml_classifiers::srv::LoadClassifier>(
      "load_classifier",
      std::bind(&ClassifierServer::loadCallback, this, _1, _2, _3));
    classify_srv = this->create_service<ml_classifiers::srv::ClassifyData>(
      "classify_data",
      std::bind(&ClassifierServer::classifyCallback, this, _1, _2, _3));

    RCLCPP_INFO(this->get_logger(), "Classifier services now ready");
  }

private:
  pluginlib::ClassLoader<Classifier> c_loader;
  std::map<string, std::shared_ptr<Classifier>> classifier_list;

  rclcpp::Service<ml_classifiers::srv::CreateClassifier>::SharedPtr create_srv;
  rclcpp::Service<ml_classifiers::srv::AddClassData>::SharedPtr add_srv;
  rclcpp::Service<ml_classifiers::srv::TrainClassifier>::SharedPtr train_srv;
  rclcpp::Service<ml_classifiers::srv::ClearClassifier>::SharedPtr clear_srv;
  rclcpp::Service<ml_classifiers::srv::SaveClassifier>::SharedPtr save_srv;
  rclcpp::Service<ml_classifiers::srv::LoadClassifier>::SharedPtr load_srv;
  rclcpp::Service<ml_classifiers::srv::ClassifyData>::SharedPtr classify_srv;

  bool createHelper(string class_type, std::shared_ptr<Classifier> & c)
  {
    try {
      c = std::shared_ptr<Classifier>(c_loader.createUnmanagedInstance(class_type));
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Classifer plugin failed to load! Error: %s",
        ex.what());
      return false;
    }

    return true;
  }

  void createCallback(
    const std::shared_ptr<rmw_request_id_t> req_hdr,
    const std::shared_ptr<ml_classifiers::srv::CreateClassifier::Request> req,
    std::shared_ptr<ml_classifiers::srv::CreateClassifier::Response> res)
  {
    (void)req_hdr;
    string id = req->identifier;
    std::shared_ptr<Classifier> c;

    if (!createHelper(req->class_type, c)) {
      res->success = false;
    } else {
      if (classifier_list.find(id) != classifier_list.end()) {
        RCLCPP_INFO(
          this->get_logger(),
          "WARNING: ID already exists, overwriting: %s",
          req->identifier.c_str());
        classifier_list.erase(id);
      }

      classifier_list[id] = c;

      res->success = true;
    }
  }

  void addCallback(
    const std::shared_ptr<rmw_request_id_t> req_hdr,
    const std::shared_ptr<ml_classifiers::srv::AddClassData::Request> req,
    std::shared_ptr<ml_classifiers::srv::AddClassData::Response> res)
  {
    (void)req_hdr;
    string id = req->identifier;

    if (classifier_list.find(id) == classifier_list.end()) {
      res->success = false;
    } else {
      for (size_t i = 0; i < req->data.size(); i++) {
        classifier_list[id]->addTrainingPoint(req->data[i].target_class, req->data[i].point);
      }

      res->success = true;
    }
  }

  void trainCallback(
    const std::shared_ptr<rmw_request_id_t> req_hdr,
    const std::shared_ptr<ml_classifiers::srv::TrainClassifier::Request> req,
    std::shared_ptr<ml_classifiers::srv::TrainClassifier::Response> res)
  {
    (void)req_hdr;
    string id = req->identifier;

    if (classifier_list.find(id) == classifier_list.end()) {
      res->success = false;
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Training %s",
        id.c_str());

      classifier_list[id]->train();
      res->success = true;
    }
  }

  void clearCallback(
    const std::shared_ptr<rmw_request_id_t> req_hdr,
    const std::shared_ptr<ml_classifiers::srv::ClearClassifier::Request> req,
    std::shared_ptr<ml_classifiers::srv::ClearClassifier::Response> res)
  {
    (void)req_hdr;
    string id = req->identifier;

    if (classifier_list.find(id) == classifier_list.end()) {
      res->success = false;
    } else {
      classifier_list[id]->clear();
      res->success = true;
    }
  }

  void saveCallback(
    const std::shared_ptr<rmw_request_id_t> req_hdr,
    const std::shared_ptr<ml_classifiers::srv::SaveClassifier::Request> req,
    std::shared_ptr<ml_classifiers::srv::SaveClassifier::Response> res)
  {
    (void)req_hdr;
    string id = req->identifier;

    if (classifier_list.find(id) == classifier_list.end()) {
      res->success = false;
    } else {
      classifier_list[id]->save(req->filename);
      res->success = true;
    }
  }

  void loadCallback(
    const std::shared_ptr<rmw_request_id_t> req_hdr,
    const std::shared_ptr<ml_classifiers::srv::LoadClassifier::Request> req,
    std::shared_ptr<ml_classifiers::srv::LoadClassifier::Response> res)
  {
    (void)req_hdr;
    string id = req->identifier;

    std::shared_ptr<Classifier> c;

    if (!createHelper(req->class_type, c)) {
      res->success = false;
    } else {
      if (!c->load(req->filename)) {
        res->success = false;
      } else {
        if (classifier_list.find(id) != classifier_list.end()) {
          RCLCPP_WARN(
            this->get_logger(),
            "WARNING: ID already exists, overwriting: %s",
            req->identifier.c_str());
          classifier_list.erase(id);
        }
        classifier_list[id] = c;

        res->success = true;
      }
    }
  }

  void classifyCallback(
    const std::shared_ptr<rmw_request_id_t> req_hdr,
    const std::shared_ptr<ml_classifiers::srv::ClassifyData::Request> req,
    std::shared_ptr<ml_classifiers::srv::ClassifyData::Response> res)
  {
    (void)req_hdr;
    string id = req->identifier;

    for (size_t i = 0; i < req->data.size(); i++) {
      string class_num = classifier_list[id]->classifyPoint(req->data[i].point);
      res->classifications.push_back(class_num);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<ClassifierServer>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
