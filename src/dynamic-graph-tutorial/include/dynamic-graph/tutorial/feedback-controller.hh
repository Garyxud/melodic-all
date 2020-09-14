/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#ifndef DG_TUTORIAL_FEEDBACK_CONTROLLER_HH
#define DG_TUTORIAL_FEEDBACK_CONTROLLER_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>

namespace dynamicgraph {
namespace tutorial {
/**
   \brief Feedback controller for an inverted pendulum

   This class implements a feedback control for the inverted pendulum
   represented by class InvertedPendulum
*/
class FeedbackController : public Entity {
 public:
  /**
     \brief Constructor by name
  */
  FeedbackController(const std::string& inName);

  ~FeedbackController();

  /// Each entity should provide the name of the class it belongs to
  virtual const std::string& getClassName(void) const { return CLASS_NAME; }

  /// Header documentation of the python class
  virtual std::string getDocString() const {
    return "Feedback controller aimed at maintaining the pendulum vertical\n";
  }
  /**
      \name Parameters
      @{
  */
  /**
     \brief Get feedback gain
  */
  void setGain(const ::dynamicgraph::Matrix& inGain) { gain_ = inGain; }

  /**
     \brief Get feedback gain
  */
  ::dynamicgraph::Matrix getGain() const { return gain_; }

  /**
     @}
  */

 protected:
  /*
    \brief Class name
  */
  static const std::string CLASS_NAME;

 private:
  /**
     Compute the control law
  */
  double& computeForceFeedback(double& force, const int& inTime);

  /**
     \brief State of the inverted pendulum
  */
  SignalPtr< ::dynamicgraph::Vector, int> stateSIN;
  /**
     \brief Force computed by the control law
  */
  SignalTimeDependent<double, int> forceSOUT;

  /// \brief Gain of the controller
  ::dynamicgraph::Matrix gain_;
};
}  // namespace tutorial
}  // namespace dynamicgraph

#endif  // DG_TUTORIAL_FEEDBACK_CONTROLLER_HH
