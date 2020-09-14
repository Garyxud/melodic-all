#ifndef SRC_ASSERTIONS_H
#define SRC_ASSERTIONS_H

#include <parameter_assertions/type_traits.h>
#include <ros/ros.h>

namespace assertions
{
enum class NumberAssertionType
{
  POSITIVE,
  NEGATIVE,
  NON_NEGATIVE,
  NON_POSITIVE,
  LESS_THAN_EQ_ONE,
  ABS_LESS_THAN_EQ_ONE,
};

template <typename T>
using AssertionFP = typename std::add_pointer<bool(const T&)>::type;

/**
 * Calls nh.param with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns the
 * result of nh.param
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param param_val
 * @param default_val
 * @return
 */
template <typename T>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val);

/**
 * Calls nh.param with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns the
 * result of nh.param
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param default_val
 * @return
 */
template <typename T>
[[nodiscard]] T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val);

/**
 * Calls nh.param with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns the
 * result of nh.param
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param param_val
 * @param default_val
 * @param assertions List of assertions to apply to the parameter
 * @return
 */
template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val,
           const std::vector<NumberAssertionType>& assertions);

/**
 * Calls nh.param with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns the
 * result of nh.param
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param param_val
 * @param default_val
 * @param assertions List of assertions to apply to the parameter
 * @return
 */
template <typename T, typename type_traits::disable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val,
           const std::vector<NumberAssertionType>& assertions);

/**
 * Calls nh.param with the passed in values. Logs with ROS_WARN_STREAM if the default value is used. Returns the
 * result of nh.param
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param default_val
 * @param assertions List of assertions to apply to the parameter
 * @return
 */
template <typename T>
[[nodiscard]] T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val,
                      const std::vector<NumberAssertionType>& assertions);

/**
 * Calls nh.getParam with the passed in values. If getParam returns false, calls ros::shutdown if exit_on_failure_
 * is false, otherwise does std::exit(-1).
 *
 * @tparam T
 * @param nh ros::NodeHandle to use
 * @param param_name
 * @param param_val
 */
template <typename T>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var);

template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
              const std::vector<NumberAssertionType>& assertions);

template <typename T, typename type_traits::disable_if<type_traits::is_number<T>::value, T>::type* = nullptr>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
              const std::vector<NumberAssertionType>& assertions);

}  // namespace assertions

#endif  // SRC_ASSERTIONS_H
