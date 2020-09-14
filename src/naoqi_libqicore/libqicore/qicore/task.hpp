/**
* @author Aldebaran Robotics
* Aldebaran Robotics (c) 2013 All Rights Reserved
*/


#ifndef QICORE_TASK_H_
# define QICORE_TASK_H_

#include <qi/signal.hpp>
#include <qi/property.hpp>

namespace qi
{
  class Task
  {
  public:
    Task() { running.set(false); }
    virtual ~Task() {}

    /** Ask the task to interrupt itself.
    *
    * @return true if the request was successful. Does not imply that the
    *         Task is effectively stopped, but that it will eventually stop.
    */
    virtual bool interrupt() = 0;

    /// RO, Indicates if the task is currently running
    Property<bool> running;

    /** RO, Indicates if the task encountered an error. Empty for no error.
     * Should be set to "interrupted" If interrupt() was called successfuly
     * and the normal Task output will not trigger.
     */
    Property<std::string> error;
  };
}

#define QI_TASK_MEMBERS interrupt, running, error

#endif
