#ifndef DYNAMIC_ROBOT_STATE_PUBLISHER_JOINT_STATE_LISTENER_H
#define DYNAMIC_ROBOT_STATE_PUBLISHER_JOINT_STATE_LISTENER_H

#include <mutex>

#include <dynamic_reconfigure/server.h>

#include <robot_state_publisher/joint_state_listener.h>
#include <dynamic_robot_state_publisher/robot_state_publisher.h>
#include <dynamic_robot_state_publisher/DynamicRobotStateConfig.h>

namespace robot_state_publisher
{
/**
 * \brief A joint state listener that first reads the robot model from (static)
 * `robot_description` parameter, and then watches the dynamic model at
 * `robot_state_publisher/robot_description`. When the dynamic model changes,
 * this listener reloads the 3D representation of the model.
 */
class DynamicJointStateListener : public JointStateListener
{
public:
  /**
   * \brief Construct the listener using the given KDL tree, URDF model and
   * mimic map.
   * \param [in] tree Parsed KDL tree.
   * \param [in] m Mimic map.
   * \param [in] model URDF model.
   */
  DynamicJointStateListener(
    const Tree &tree, const MimicMap &m, const urdf::Model &model);

  /**
   * \brief Reload the robot model.
   * \param [in] urdf If empty, load it from (static) param `robot_description`,
   *                  otherwise load it from the given string.
   * \return True if the model was correctly read and parsed.
   */
  bool reloadRobotModel(const std::string &urdf = "");

  /**
   * \brief Parses the given URDF string (or `robot_description` param if the
   *        string is empty) to a KDL tree, URDF model and a mimic map.
   * \param [out] tree The parsed KDL tree.
   * \param [out] mimic_map The parsed mimic map.
   * \param [out] The parsed URDF model.
   * \param [in] urdf If empty, parse the model from (static) param
   *                  `robot_description`, otherwise parse it from the given
   *                  string.
   * \return True if the model was correctly read and parsed.
   */
  static bool loadModel(KDL::Tree &tree, MimicMap &mimic_map,
                        urdf::Model &model, const std::string &urdf = "");

protected:
  /**
   * \brief Called when new joint state arrives.
   * \param [in] state The joint state to process.
   */
  void callbackJointState(const JointStateConstPtr &state) override;

  /** \brief Mutex for updates. */
  std::mutex updateOngoing;

  /** \brief The RobotStatePublisher with update functionality. */
  DynamicRobotStatePublisher dynamicPublisher;

  /** \brief The dynamic parameter server. */
  dynamic_reconfigure::Server<DynamicRobotStateConfig> dynparamServer;

  /**
   * \brief The callback that is called whenever the dynamic robot model changes.
   * \param config The updated config.
   * \param level Nonzero on startup, zero otherwise.
   */
  void configChangedCb(DynamicRobotStateConfig &config, uint32_t level);

};
}

#endif //DYNAMIC_ROBOT_STATE_PUBLISHER_JOINT_STATE_LISTENER_H
