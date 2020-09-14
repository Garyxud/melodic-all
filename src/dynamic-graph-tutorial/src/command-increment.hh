//
// Copyright 2010 CNRS
//
// Author: Florent Lamiraux
//

#ifndef DYNAMIC_GRAPH_TUTORIAL_COMMAND_INCREMENT_HH
#define DYNAMIC_GRAPH_TUTORIAL_COMMAND_INCREMENT_HH

#include <boost/assign/list_of.hpp>

namespace dynamicgraph {
namespace tutorial {
namespace command {
using ::dynamicgraph::command::Command;
using ::dynamicgraph::command::Value;
class Increment : public Command {
 public:
  virtual ~Increment() {}
  /// Create a command and store it in Entity
  /// \param entity Instance of Entity owning of the command
  /// \param docstring documentation of the command
  Increment(InvertedPendulum& entity, const std::string& docstring)
      : Command(entity, boost::assign::list_of(Value::DOUBLE), docstring) {}
  virtual Value doExecute() {
    Entity& entity = owner();
    InvertedPendulum& ip = static_cast<InvertedPendulum&>(entity);
    std::vector<Value> values = getParameterValues();
    double timeStep = values[0].value();
    ip.incr(timeStep);
    return Value();
  }
};  // class Increment
}  // namespace command
}  // namespace tutorial
}  // namespace dynamicgraph
#endif  // DYNAMIC_GRAPH_TUTORIAL_COMMAND_INCREMENT_HH
