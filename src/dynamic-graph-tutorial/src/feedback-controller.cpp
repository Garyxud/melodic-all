/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include "dynamic-graph/tutorial/feedback-controller.hh"
#include "constant.hh"

using namespace dynamicgraph;
using namespace dynamicgraph::tutorial;

// Register new Entity type in the factory
// Note that the second argument is the type name of the python class
// that will be created when importing the python module.
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(FeedbackController, "FeedbackController");

FeedbackController::FeedbackController(const std::string& inName)
    : Entity(inName),
      stateSIN(NULL, "FeedbackController(" + inName + ")::input(vector)::state"),
      forceSOUT(stateSIN, "FeedbackController(" + inName + ")::output(double)::force"),
      gain_(Matrix(4, 1)) {
  // Register signals into the entity.
  signalRegistration(stateSIN);
  signalRegistration(forceSOUT);

  // Set signals as constant to size them
  double force = 0.;
  Vector state(4);
  state.fill(0.);
  forceSOUT.setConstant(force);
  stateSIN.setConstant(state);

  // Define refresh function for output signal
  boost::function2<double&, double&, const int&> ftest =
      boost::bind(&FeedbackController::computeForceFeedback, this, _1, _2);

  forceSOUT.setFunction(boost::bind(&FeedbackController::computeForceFeedback, this, _1, _2));
  std::string docstring;
  // setGain
  docstring =
      "\n"
      "    Set gain of controller\n"
      "      takes a tuple of 4 floating point numbers as input\n"
      "\n";
  addCommand(std::string("setGain"), new ::dynamicgraph::command::Setter<FeedbackController, Matrix>(
                                         *this, &FeedbackController::setGain, docstring));

  // getGain
  docstring =
      "\n"
      "    Get gain of controller\n"
      "      return a tuple of 4 floating point numbers\n"
      "\n";
  addCommand(std::string("getGain"), new ::dynamicgraph::command::Getter<FeedbackController, Matrix>(
                                         *this, &FeedbackController::getGain, docstring));
}

FeedbackController::~FeedbackController() {}

double& FeedbackController::computeForceFeedback(double& force, const int& inTime) {
  const Vector& state = stateSIN(inTime);

  if (state.size() != 4)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC, "state signal size is ",
                                        "%d, should be 4.", state.size());
  Vector v(-gain_ * state);
  force = v(0);
  return force;
}
