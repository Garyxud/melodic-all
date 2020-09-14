#include <parameter_assertions/assertions.h>
#include <optional>

namespace assertions
{
namespace
{
template <typename T>
std::string vectorToString(const std::vector<T>& vector)
{
  std::stringstream ss;

  ss << "{ ";
  if (!vector.empty())
  {
    ss << vector[0];

    for (size_t i = 0; i < vector.size(); i++)
    {
      ss << ", " << vector[i];
    }
  }
  ss << " }";

  return ss.str();
}

template <typename T, typename type_traits::disable_if<type_traits::is_vector<T>::value, T>::type* = nullptr>
void warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name, T& variable,
                            const std::string& message)
{
  ROS_WARN_STREAM("[" << node_namespace << "] " << variable_name << message << ". Continuing with default values "
                      << variable);
}

template <typename V, typename std::enable_if<type_traits::is_vector<V>::value, V>::type* = nullptr>
void warnDefaultWithMessage(const std::string& node_namespace, const std::string& variable_name, V& variable,
                            const std::string& message)
{
  std::string vector_str = vectorToString(variable);
  warnDefaultWithMessage(node_namespace, variable_name, vector_str, message);
}

template <typename T>
AssertionFP<T> getAssertionFunction(NumberAssertionType assertion_type)
{
  switch (assertion_type)
  {
    case NumberAssertionType::POSITIVE:
      return [](const T& param) { return param > 0; };
    case NumberAssertionType::NON_NEGATIVE:
      return [](const T& param) { return param >= 0; };
    case NumberAssertionType::NEGATIVE:
      return [](const T& param) { return param < 0; };
    case NumberAssertionType::NON_POSITIVE:
      return [](const T& param) { return param <= 0; };
    case NumberAssertionType::LESS_THAN_EQ_ONE:
      return [](const T& param) { return param <= 1; };
    case NumberAssertionType::ABS_LESS_THAN_EQ_ONE:
      return [](const T& param) { return std::abs(param) <= 1; };
    default:
      ROS_ERROR_STREAM("default case reached in getAssertionFunction even though match was exhaustive");
      return [](const T& /*param*/) { return false; };
  }
}

template <typename T>
std::string getAssertionErrorMessage(NumberAssertionType assertion_type, const T& variable)
{
  switch (assertion_type)
  {
    case NumberAssertionType::POSITIVE:
      return std::to_string(variable) + " must be > 0.";
    case NumberAssertionType::NON_NEGATIVE:
      return std::to_string(variable) + " must be >= 0.";
    case NumberAssertionType::NEGATIVE:
      return std::to_string(variable) + " must be < 0.";
    case NumberAssertionType::NON_POSITIVE:
      return std::to_string(variable) + " must be <= 0.";
    case NumberAssertionType::LESS_THAN_EQ_ONE:
      return std::to_string(variable) + " must be <= 1.";
    case NumberAssertionType::ABS_LESS_THAN_EQ_ONE:
      return std::to_string(variable) + " must have an absolute value <= 1.";
    default:
      ROS_ERROR_STREAM("default case reached in getAssertionFunction even though match was exhaustive");
      return "";
  }
}

template <typename T, typename type_traits::disable_if<type_traits::is_vector<T>::value, T>::type* = nullptr>
std::optional<std::string> getErrorMessage(const T& variable, const std::vector<NumberAssertionType>& assertions)
{
  for (const auto& assertion : assertions)
  {
    if (!getAssertionFunction<T>(assertion)(variable))
    {
      return getAssertionErrorMessage(assertion, variable);
    }
  }
  return std::nullopt;
}

template <typename V, typename std::enable_if<type_traits::is_vector<V>::value, V>::type* = nullptr>
bool passesAssertion(V& variable, const std::vector<NumberAssertionType>& assertions)
{
  for (const auto& element : variable)
  {
    if (!passesAssertion(element, assertions))
    {
      return false;
    }
  }
  return true;
}

void fail()
{
  ros::shutdown();
}

}  // namespace

template <typename T>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val)
{
  if (!nh.param(param_name, param_var, default_val))
  {
    warnDefaultWithMessage(nh.getNamespace(), param_name, default_val, " is not set");
    return false;
  }
  return true;
}

template <typename T>
T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val)
{
  T param_var;
  param(nh, param_name, param_var, default_val);
  return param_var;
}

template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type*>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val,
           const std::vector<NumberAssertionType>& assertions)
{
  if (nh.getParam(param_name, param_var))
  {
    auto message = getErrorMessage(param_var, assertions);
    if (!message)
    {
      return true;
    }
    else
    {
      ROS_WARN_STREAM("[" << nh.getNamespace() << "] " << *message << " Continuing with default parameter.");
    }
  }

  param_var = default_val;
  auto message = getErrorMessage(param_var, assertions);
  if (!message)
  {
    return true;
  }
  ROS_ERROR_STREAM("[" << nh.getNamespace() << "] " << *message << " Exiting...");
  fail();

  return false;
}

template <typename T, typename type_traits::disable_if<type_traits::is_number<T>::value, T>::type*>
bool param(const ros::NodeHandle& nh, const std::string& param_name, T& param_var, const T& default_val,
           const std::vector<NumberAssertionType>& assertions)
{
  ROS_WARN_STREAM("[" << nh.getNamespace() << "] An assertion was set for " << param_name
                      << " but it is not a number. Continuing without assertions.");

  return param(nh, param_name, param_var, default_val);
}

template <typename T>
T param(const ros::NodeHandle& nh, const std::string& param_name, const T& default_val,
        const std::vector<NumberAssertionType>& assertions)
{
  T param_var;
  param(nh, param_name, param_var, default_val, assertions);
  return param_var;
}

template <typename T>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var)
{
  if (!nh.getParam(param_name, param_var))
  {
    ROS_ERROR_STREAM("[" << nh.getNamespace() << "] " << param_name << " is not set. Exiting...");
    fail();
    return false;
  }

  return true;
}

template <typename T, typename std::enable_if<type_traits::is_number<T>::value, T>::type*>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
              const std::vector<NumberAssertionType>& assertions)
{
  if (getParam(nh, param_name, param_var))
  {
    if (auto message = getErrorMessage(param_var, assertions))
    {
      ROS_ERROR_STREAM("[" << nh.getNamespace() << "] " << *message << " Exiting...");
      fail();
      return false;
    }
  }

  return true;
}

template <typename T, typename type_traits::disable_if<type_traits::is_number<T>::value, T>::type*>
bool getParam(const ros::NodeHandle& nh, const std::string& param_name, T& param_var,
              const std::vector<NumberAssertionType>& /*assertions*/)
{
  ROS_WARN_STREAM("[" << nh.getNamespace() << "] An assertion was set for " << param_name
                      << " but it is not a number. Continuing without assertions.");

  return getParam(nh, param_name, param_var);
}

template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::string& param_var);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, double& param_var);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, float& param_var);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, int& param_var);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, bool& param_var);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<std::string>& param_var);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<double>& param_var);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<float>& param_var);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<int>& param_var);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<bool>& param_var);

template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::string& param_var,
                       const std::vector<NumberAssertionType>& assertions);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, double& param_var,
                       const std::vector<NumberAssertionType>& assertions);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, float& param_var,
                       const std::vector<NumberAssertionType>& assertions);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, int& param_var,
                       const std::vector<NumberAssertionType>& assertions);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, bool& param_var,
                       const std::vector<NumberAssertionType>& assertions);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<std::string>& param_var,
                       const std::vector<NumberAssertionType>& assertions);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<double>& param_var,
                       const std::vector<NumberAssertionType>& assertions);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<float>& param_var,
                       const std::vector<NumberAssertionType>& assertions);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<int>& param_var,
                       const std::vector<NumberAssertionType>& assertions);
template bool getParam(const ros::NodeHandle& nh, const std::string& param_name, std::vector<bool>& param_var,
                       const std::vector<NumberAssertionType>& assertions);

template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::string& param_var,
                    const std::string& default_val);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, double& param_var,
                    const double& default_val);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, float& param_var,
                    const float& default_val);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, int& param_var, const int& default_val);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, bool& param_var, const bool& default_val);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<std::string>& param_var,
                    const std::vector<std::string>& default_val);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<double>& param_var,
                    const std::vector<double>& default_val);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<float>& param_var,
                    const std::vector<float>& default_val);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<int>& param_var,
                    const std::vector<int>& default_val);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<bool>& param_var,
                    const std::vector<bool>& default_val);

template std::string param(const ros::NodeHandle& nh, const std::string& param_name, const std::string& default_val);
template double param(const ros::NodeHandle& nh, const std::string& param_name, const double& default_val);
template float param(const ros::NodeHandle& nh, const std::string& param_name, const float& default_val);
template int param(const ros::NodeHandle& nh, const std::string& param_name, const int& default_val);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, const bool& default_val);
template std::vector<std::string> param(const ros::NodeHandle& nh, const std::string& param_name,
                                        const std::vector<std::string>& default_val);
template std::vector<double> param(const ros::NodeHandle& nh, const std::string& param_name,
                                   const std::vector<double>& default_val);
template std::vector<float> param(const ros::NodeHandle& nh, const std::string& param_name,
                                  const std::vector<float>& default_val);
template std::vector<int> param(const ros::NodeHandle& nh, const std::string& param_name,
                                const std::vector<int>& default_val);
template std::vector<bool> param(const ros::NodeHandle& nh, const std::string& param_name,
                                 const std::vector<bool>& default_val);

template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::string& param_var,
                    const std::string& default_val, const std::vector<NumberAssertionType>& assertions);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, double& param_var,
                    const double& default_val, const std::vector<NumberAssertionType>& assertions);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, float& param_var,
                    const float& default_val, const std::vector<NumberAssertionType>& assertions);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, int& param_var, const int& default_val,
                    const std::vector<NumberAssertionType>& assertions);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, bool& param_var, const bool& default_val,
                    const std::vector<NumberAssertionType>& assertions);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<std::string>& param_var,
                    const std::vector<std::string>& default_val, const std::vector<NumberAssertionType>& assertions);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<double>& param_var,
                    const std::vector<double>& default_val, const std::vector<NumberAssertionType>& assertions);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<float>& param_var,
                    const std::vector<float>& default_val, const std::vector<NumberAssertionType>& assertions);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<int>& param_var,
                    const std::vector<int>& default_val, const std::vector<NumberAssertionType>& assertions);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, std::vector<bool>& param_var,
                    const std::vector<bool>& default_val, const std::vector<NumberAssertionType>& assertions);

template std::string param(const ros::NodeHandle& nh, const std::string& param_name, const std::string& default_val,
                           const std::vector<NumberAssertionType>& assertions);
template double param(const ros::NodeHandle& nh, const std::string& param_name, const double& default_val,
                      const std::vector<NumberAssertionType>& assertions);
template float param(const ros::NodeHandle& nh, const std::string& param_name, const float& default_val,
                     const std::vector<NumberAssertionType>& assertions);
template int param(const ros::NodeHandle& nh, const std::string& param_name, const int& default_val,
                   const std::vector<NumberAssertionType>& assertions);
template bool param(const ros::NodeHandle& nh, const std::string& param_name, const bool& default_val,
                    const std::vector<NumberAssertionType>& assertions);
template std::vector<std::string> param(const ros::NodeHandle& nh, const std::string& param_name,
                                        const std::vector<std::string>& default_val,
                                        const std::vector<NumberAssertionType>& assertions);
template std::vector<double> param(const ros::NodeHandle& nh, const std::string& param_name,
                                   const std::vector<double>& default_val,
                                   const std::vector<NumberAssertionType>& assertions);
template std::vector<float> param(const ros::NodeHandle& nh, const std::string& param_name,
                                  const std::vector<float>& default_val,
                                  const std::vector<NumberAssertionType>& assertions);
template std::vector<int> param(const ros::NodeHandle& nh, const std::string& param_name,
                                const std::vector<int>& default_val,
                                const std::vector<NumberAssertionType>& assertions);
template std::vector<bool> param(const ros::NodeHandle& nh, const std::string& param_name,
                                 const std::vector<bool>& default_val,
                                 const std::vector<NumberAssertionType>& assertions);
}  // namespace assertions
