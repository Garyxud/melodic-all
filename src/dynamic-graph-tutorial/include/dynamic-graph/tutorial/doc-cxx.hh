/**
\page dg_tutorial_inverted_pendulum_cxx C++ implementation

\section dg_tutorial_inverted_pendulum_cxx_intro Introduction

New entity types are defined by
\li deriving dynamicgraph::Entity class,
\li adding signals, and
\li adding commands.
As an example, we review below class dynamicgraph::tutorial::InvertedPendulum.

\section dg_tutorial_inverted_pendulum_cxx_interface Interface

The interface is defined in file <c>include/dynamic-graph/tutorial/inverted-pendulum.hh</c>.

First, we include
\li the header defining Entity class and
\li the header defining SignalPtr template class,
\li the header defining matrix and vector types:

\code
    #include <dynamic-graph/entity.h>
    #include <dynamic-graph/signal-ptr.h>
    #include <dynamic-graph/linear-algebra.h>
\endcode

Then in namespace <c>dynamicgraph::tutorial</c> we define class InvertedPendulum
\code
    namespace dynamicgraph {
      namespace tutorial {
        class InvertedPendulum : public Entity
        {
\endcode
with a constructor taking a name as an input
\code
          InvertedPendulum(const std::string& inName);
\endcode

For the internal machinery, each entity can provide the name of the class it
belongs to:
\code
          virtual const std::string& getClassName (void) const {
            return CLASS_NAME;
          }
\endcode

Class InvertedPendulum represents a dynamical system. The following method
integrates the equation of motion over a time step
\code
          void incr(double inTimeStep);
\endcode

Setters and getters will enable us later to control parameters through commands.
\code
          void setCartMass (const double& inMass) {
            cartMass_ = inMass;
          }

          double getCartMass () const {
            return cartMass_;
          }

          void setPendulumMass (const double& inMass) {
            pendulumMass_ = inMass;
          }

          double getPendulumMass () const {
           return pendulumMass_;
          }

          void setPendulumLength (const double& inLength) {
            pendulumLength_ = inLength;
          }

          double getPendulumLength () const {
            return pendulumLength_;
          }
\endcode

The name of the class is stored as a static member
\code
          static const std::string CLASS_NAME;
\endcode
In the private part of the class, we store signals
\code
          SignalPtr< double, int > forceSIN;
          Signal< Vector, int> stateSOUT;
\endcode
and parameters
\code
          double cartMass_;
          double pendulumMass_;
          double pendulumLength_;
          double viscosity_;
\endcode

\section dg_tutorial_inverted_pendulum_cxx_implementation Implementation

The implementation is written in file <c>src/inverted-pendulum.cc</c>.

First, we include headers defining
\li class FactoryStorage,
\li general setter and getter commands
\li the previously defined header,
\li local Increment command class, and
\li gravity constant:

\subsection dg_tutorial_inverted_pendulum_cxx_implementation_headers Headers

\code
    #include <dynamic-graph/factory.h>
    #include <dynamic-graph/command-setter.h>
    #include <dynamic-graph/command-getter.h>
    #include "dynamic-graph/tutorial/inverted-pendulum.hh"
    #include "command-increment.hh"
\endcode

\subsection dg_tutorial_inverted_pendulum_cxx_implementation_entity_registration Entity registration

The second step consists in
\li registering our new class into the entity factory and
\li instantiating the static variable CLASS_NAME

using a macro defined in <c>dynamic-graph/factory.h</c>:
\code
    DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(InvertedPendulum, "InvertedPendulum");
\endcode
\note The two parameters of the macros are respectively
\li the C++ type of the new Entity,
\li the name of the type of the corresponding python class. It is highly
recommended to use the same name for both.

\subsection dg_tutorial_inverted_pendulum_cxx_implementation_constructor Constructor

Then we define the class constructor
\li passing the instance name to Entity class constructor,
\li initializing signals with a string following the specified format and
\li initializing parameters with default values:

\code
    InvertedPendulum::InvertedPendulum(const std::string& inName) :
      Entity(inName),
      forceSIN(NULL, "InvertedPendulum("+inName+")::input(vector)::force"),
      stateSOUT("InvertedPendulum("+inName+")::output(vector)::state"),
      cartMass_(1.0), pendulumMass_(1.0), pendulumLength_(1.0), viscosity_(0.1)
\endcode

We register signals into an associative array stored into Entity class
\code
      signalRegistration (forceSIN);
      signalRegistration (stateSOUT);
\endcode

We set input and output signal as constant with a given value
\code
      Vector state = boost::numeric::ublas::zero_vector<double>(4);
      double input = 0.;
      stateSOUT.setConstant(state);
      forceSIN.setConstant(input);
\endcode

The following lines of code define and register commands into the entity.
A \ref dynamicgraph::command::Command "command" is created by calling a constructor with
\li a string: the name of the command,
\li a pointer to a newly created command and
\li a string documenting the command:

\code
      std::string docstring;

      // Incr
      docstring =
        "\n"
        "    Integrate dynamics for time step provided as input\n"
        "\n"
        "      take one floating point number as input\n"
        "\n";
      addCommand(std::string("incr"),
                 new command::Increment(*this, docstring));
\endcode

In this example, command::Increment is a command specific to our class
InvertedPendulum. This new command is explained in page \ref dg_tutorial_inverted_pendulum_command.

Setter and getter commands are available through classes templated by the type of entity using the command and the type
of the parameter. Be aware that only a prespecified set of types are supported for commands, see class
dynamicgraph::command::Value. \code docstring =
    "\n"
    "    Set cart mass\n"
    "\n";
  addCommand(std::string("setCartMass"),
             new ::dynamicgraph::command::Setter<InvertedPendulum, double>
             (*this, &InvertedPendulum::setCartMass, docstring));

  docstring =
    "\n"
    "    Get cart mass\n"
    "\n";
  addCommand(std::string("getCartMass"),
             new ::dynamicgraph::command::Getter<InvertedPendulum, double>
             (*this, &InvertedPendulum::getCartMass, docstring));
\endcode

\note
It is important to notice that
\li commands passed to method Entity::addCommand will be destroyed automatically by Entity class destructor. The user
should therefore not destroy them, \li commands should be defined and registered in the class constructor. Commands
defined later on will not be reachable by python bindings.

\subsection dg_tutorial_inverted_pendulum_cxx_implementation_newtypes Registering new types: advanced feature

Signals are templated by the type of data they convey. In this example, we hae defined our own class of vectors
InvertedPendulum::Vector. In order to be able to create signals with this type, we need to register the new type: \code
dynamicgraph::DefaultCastRegisterer<InvertedPendulum::Vector> IPVectorCast;
\endcode

\note
The new type should implement operator<< and operator>> in order to store variables in streams.

*/
