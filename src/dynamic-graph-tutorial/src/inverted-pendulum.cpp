/*
 *  Copyright 2010 CNRS
 *
 *  Florent Lamiraux
 */

#include <boost/format.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include "dynamic-graph/tutorial/inverted-pendulum.hh"
#include "command-increment.hh"
#include "constant.hh"

using namespace dynamicgraph;
using namespace dynamicgraph::tutorial;

const double Constant::gravity = 9.81;

// Register new Entity type in the factory
// Note that the second argument is the type name of the python class
// that will be created when importing the python module.
DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(InvertedPendulum, "InvertedPendulum");

InvertedPendulum::InvertedPendulum(const std::string& inName)
    : Entity(inName),
      forceSIN(NULL, "InvertedPendulum(" + inName + ")::input(double)::force"),
      stateSOUT("InvertedPendulum(" + inName + ")::output(vector)::state"),
      cartMass_(1.0),
      pendulumMass_(1.0),
      pendulumLength_(1.0),
      viscosity_(0.1) {
  // Register signals into the entity.
  signalRegistration(forceSIN);
  signalRegistration(stateSOUT);

  // Set signals as constant to size them
  Vector state(4);
  state.fill(0.);
  double input = 0.;
  stateSOUT.setConstant(state);
  forceSIN.setConstant(input);

  // Commands
  std::string docstring;

  // Incr
  docstring =
      "\n"
      "    Integrate dynamics for time step provided as input\n"
      "\n"
      "      take one floating point number as input\n"
      "\n";
  addCommand(std::string("incr"), new command::Increment(*this, docstring));

  // setCartMass
  docstring =
      "\n"
      "    Set cart mass\n"
      "\n";
  addCommand(std::string("setCartMass"), new ::dynamicgraph::command::Setter<InvertedPendulum, double>(
                                             *this, &InvertedPendulum::setCartMass, docstring));

  // getCartMass
  docstring =
      "\n"
      "    Get cart mass\n"
      "\n";
  addCommand(std::string("getCartMass"), new ::dynamicgraph::command::Getter<InvertedPendulum, double>(
                                             *this, &InvertedPendulum::getCartMass, docstring));

  // setPendulumMass
  docstring =
      "\n"
      "    Set pendulum mass\n"
      "\n";
  addCommand(std::string("setPendulumMass"), new ::dynamicgraph::command::Setter<InvertedPendulum, double>(
                                                 *this, &InvertedPendulum::setPendulumMass, docstring));

  // getPendulumMass
  docstring =
      "\n"
      "    Get pendulum mass\n"
      "\n";
  addCommand(std::string("getPendulumMass"), new ::dynamicgraph::command::Getter<InvertedPendulum, double>(
                                                 *this, &InvertedPendulum::getPendulumMass, docstring));

  // setPendulumLength
  docstring =
      "\n"
      "    Set pendulum length\n"
      "\n";
  addCommand(std::string("setPendulumLength"), new ::dynamicgraph::command::Setter<InvertedPendulum, double>(
                                                   *this, &InvertedPendulum::setPendulumLength, docstring));

  // getPendulumLength
  docstring =
      "\n"
      "    Get pendulum length\n"
      "\n";
  addCommand(std::string("getPendulumLength"), new ::dynamicgraph::command::Getter<InvertedPendulum, double>(
                                                   *this, &InvertedPendulum::getPendulumLength, docstring));
}

InvertedPendulum::~InvertedPendulum() {}

Vector InvertedPendulum::computeDynamics(const Vector& inState, const double& inControl, double inTimeStep) {
  if (inState.size() != 4)
    throw dynamicgraph::ExceptionSignal(dynamicgraph::ExceptionSignal::GENERIC, "state signal size is ",
                                        "%d, should be 4.", inState.size());

  double dt = inTimeStep;
  double dt2 = dt * dt;
  double g = Constant::gravity;
  double x = inState(0);
  double th = inState(1);
  double dx = inState(2);
  double dth = inState(3);
  double F = inControl;
  double m = pendulumMass_;
  double M = cartMass_;
  double l = pendulumLength_;
  double lambda = viscosity_;
  double l2 = l * l;
  double dth2 = dth * dth;
  double sth = sin(th);
  double cth = cos(th);
  double sth2 = sth * sth;

  double b1 = F - m * l * dth2 * sth - lambda * dx;
  double b2 = m * l * g * sth - lambda * dth;

  double det = m * l2 * (M + m * sth2);

  double ddx = (b1 * m * l2 + b2 * m * l * cth) / det;
  double ddth = ((M + m) * b2 + m * l * cth * b1) / det;

  Vector nextState(4);
  nextState(0) = x + dx * dt + .5 * ddx * dt2;
  nextState(1) = th + dth * dt + .5 * ddth * dt2;
  nextState(2) = dx + dt * ddx;
  nextState(3) = dth + dt * ddth;

  return nextState;
}

void InvertedPendulum::incr(double inTimeStep) {
  int t = stateSOUT.getTime();
  Vector nextState = computeDynamics(stateSOUT(t), forceSIN(t), inTimeStep);
  stateSOUT.setConstant(nextState);
  stateSOUT.setTime(t + 1);
  forceSIN(t + 1);
}
