/*
 * Copyright 2010,
 * Florent Lamiraux
 *
 * CNRS
 *
 */

#ifndef DG_TUTORIAL_INVERTED_PENDULUM_HH
#define DG_TUTORIAL_INVERTED_PENDULUM_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/linear-algebra.h>

namespace dynamicgraph {
namespace tutorial {

/**
   \brief Inverted Pendulum on a cart

   This class represents the classical inverted pendulum on a cart.
   The equation of motion is:

   \f{eqnarray*}{
   \left ( M + m \right ) \ddot x - m l \ddot \theta \cos \theta + m l \dot \theta^2 \sin \theta &=& F\\
   m l (-g \sin \theta - \ddot x \cos \theta + l \ddot \theta) &=& 0
   \f}

   where
   \li the state is a vector of dimension 4
       \f$(x,\theta,\dot{x},\dot{\theta})\f$ represented by signal
       stateSOUT,
   \li \f$x\f$ is the position of the cart on an horizontal axis,
       \f$\theta\f$ is the angle of the pendulum with respect to the
        vertical axis,
   \li the input is a vector of dimension 1 \f$(F)\f$ reprensented by signal
       forceSIN,
   \li m, M and l are respectively the mass of the pendulum, the mass of the
       cart and the length of the pendulum.

   A more natural form of the above equation for roboticists is
   \f[
   \textbf{M}(\textbf{q})\ddot{\textbf{q}} +
   \textbf{N}(\textbf{q},\dot{\textbf{q}})\dot{\textbf{q}} +
   \textbf{G}(\textbf{q}) = \textbf{F}
   \f]
   where
   \f{eqnarray*}
   \textbf{q} &=& (x, \theta) \\
   \textbf{M}(\textbf{q}) &=& \left( \begin{array}{cc}
   M + m & -m\ l\ \cos\theta \\
   -m\ l\ \cos\theta & m\ l^2 \end{array}\right) \\
   \textbf{N}(\textbf{q},\dot{\textbf{q}}) &=& \left( \begin{array}{cc}
   0 & m\ l\ \dot{\theta} \sin\theta \\
   0 & 0 \end{array}\right)\\
   \textbf{G}(\textbf{q}) &=& \left( \begin{array}{c}
   0 \\ -m\ l\ g\ \sin\theta \end{array}\right)\\
   \textbf{F} &=& \left( \begin{array}{c}
   F \\ 0 \end{array}\right)
   \f}
   In order to make the system intrinsically stable, we add some viscosity
   by rewriting:
   \f{eqnarray*}
   \textbf{N}(\textbf{q},\dot{\textbf{q}}) &=& \left( \begin{array}{cc}
   \lambda & m\ l\ \dot{\theta} \sin\theta\\
   0 & \lambda \end{array}\right)
   \f}
   where \f$\lambda\f$ is a positive coefficient.
*/

class InvertedPendulum : public Entity {
 public:
  /**
     \brief Constructor by name
  */
  InvertedPendulum(const std::string& inName);

  ~InvertedPendulum();

  /// Each entity should provide the name of the class it belongs to
  virtual const std::string& getClassName(void) const { return CLASS_NAME; }

  /// Header documentation of the python class
  virtual std::string getDocString() const { return "Classical inverted pendulum dynamic model\n"; }

  /// Integrate equation of motion over time step given as input
  void incr(double inTimeStep);

  /**
      \name Parameters
      @{
  */
  /**
     \brief Set the mass of the cart
  */
  void setCartMass(const double& inMass) { cartMass_ = inMass; }

  /**
     \brief Get the mass of the cart
  */
  double getCartMass() const { return cartMass_; }

  /**
     \brief Set the mass of the cart
  */
  void setPendulumMass(const double& inMass) { pendulumMass_ = inMass; }

  /**
     \brief Get the mass of the pendulum
  */
  double getPendulumMass() const { return pendulumMass_; }

  /**
     \brief Set the length of the cart
  */
  void setPendulumLength(const double& inLength) { pendulumLength_ = inLength; }

  /**
     \brief Get the length of the pendulum
  */
  double getPendulumLength() const { return pendulumLength_; }

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
     \brief Input force acting on the inverted pendulum
  */
  SignalPtr<double, int> forceSIN;
  /**
     \brief State of the inverted pendulum
  */
  Signal< ::dynamicgraph::Vector, int> stateSOUT;

  /// \brief Mass of the cart
  double cartMass_;
  /// \brief Mass of the pendulum
  double pendulumMass_;
  /// \brief Length of the pendulum
  double pendulumLength_;
  /// \brief Viscosity coefficient
  double viscosity_;

  /**
     \brief Compute the evolution of the state of the pendulum
  */
  ::dynamicgraph::Vector computeDynamics(const ::dynamicgraph::Vector& inState, const double& inControl,
                                         double inTimeStep);
};
}  // namespace tutorial
}  // namespace dynamicgraph
#endif
