/**
\page dg_tutorial_inverted_pendulum_command Creating a new command

\section dg_tutorial_inverted_pendulum_command_intro Introduction

Command (dynamicgraph::command::Command) are objects that encapsulate an
action to be performed on an entity.

In this page, we define a new command that will call method
InvertedPendulum::incr. The source code is in
<c>src/command-increment.hh</c>.

\section dg_tutorial_inverted_pendulum_command_implementation Implementation

We first define the new class by deriving dynamicgraph::command::Command:
\code
    namespace dynamicgraph {
      namespace tutorial {
        namespace command {
          class Increment : public Command
          {
\endcode

The constructor takes
\li a reference to a InvertedPendulum and calls parent class,
\li a vector a types specifying the number and types of input arguments of the command and
\li a string documenting the command.
In this case, there is only one argument of type <c>double</c>. Note the use of <c>boost::assign::list_of</c> to build
a vector in one command line: \code Increment(InvertedPendulum& entity, const std::string& docstring) : Command(entity,
boost::assign::list_of(Value::DOUBLE), docstring)
        {
        }
\endcode

We then define the action of the command in virtual method <c>doExecute</c>.
We need to get a reference to the object on which the command will act. Note
that we can straightfowardly statically cast in <c>InvertedPendulum</c> the
Entity object returned by method <c>owner</c>:
\code
    virtual Value doExecute()
    {
      Entity& entity = owner();
      InvertedPendulum& ip = static_cast<InvertedPendulum&>(entity);
\endcode
We then get the parameters as a vector of dynamicgraph::command::Value objects
and cast them into the appropriate types specified at construction:
\code
      std::vector<Value> values = getParameterValues();
      double timeStep = values[0].value();
\endcode
Finally, we execute the action and return a value of the appropriate type, here
the command return no value:
\code
      ip.incr(timeStep);
      return Value();
    }
\endcode
*/
