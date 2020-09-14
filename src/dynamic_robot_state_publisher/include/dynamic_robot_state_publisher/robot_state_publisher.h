#ifndef DYNAMIC_ROBOT_STATE_PUBLISHER_ROBOT_STATE_PUBLISHER_H
#define DYNAMIC_ROBOT_STATE_PUBLISHER_ROBOT_STATE_PUBLISHER_H

#include <robot_state_publisher/robot_state_publisher.h>

namespace robot_state_publisher
{
/**
 * \brief An alternative RobotStatePublisher with update option.
 */
class DynamicRobotStatePublisher
{
public:
  /** \brief The TF frame name of the virtual frame that's parent of all
   * deleted static TF frames. */
  const std::string DELETED_STATIC_TFS_FRAME = "__deleted_static_tfs__";

  /**
   * \brief Create the publisher.
   * \param [in] publisher The underlying RobotStatePublisher that is hijacked.
   */
  explicit DynamicRobotStatePublisher(RobotStatePublisher *publisher);

  /**
   * \brief Sets the robot model.
   * \param tree The kinematic model of a robot, represented by a KDL Tree.
   */
  virtual void updateTree(const KDL::Tree& tree);

  /**
   * \brief Return the number of moving joints in the currently represented model.
   * \return The number of moving joints in the currently represented model.
   */
  virtual size_t getNumMovingJoints() const;

  /**
   * \brief Return the number of fixed joints in the currently represented model.
   * \return The number of fixed joints in the currently represented model.
   */
  virtual size_t getNumFixedJoints() const;

protected:
  /** \brief The underlying (hacked) publisher. */
  RobotStatePublisher *publisher;
};
}

#endif //DYNAMIC_ROBOT_STATE_PUBLISHER_ROBOT_STATE_PUBLISHER_H
