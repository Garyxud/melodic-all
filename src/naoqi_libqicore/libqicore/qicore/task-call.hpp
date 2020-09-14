/**
* @author Aldebaran Robotics
* Aldebaran Robotics (c) 2013 All Rights Reserved
*/
#include <qi/jsoncodec.hpp>

#include <qicore/task.hpp>


#ifndef QICORE_TASK_CALL_H_
# define QICORE_TASK_CALL_H_

namespace qi
{
  /** Task wrapper over a function of return type T
  */
  template<typename T>
  class TaskCall: public Task
  {
  public:
    TaskCall(AnyFunction f)
    : result(typeOf<T>())
    , _f(f)
    {
    }
    virtual bool interrupt() { return false;}
    virtual void start(const AnyVarArguments& args)
    {
      running.set(true);
      try
      {
        std::vector<AnyReference> nargs;
        nargs.reserve(args.args().size());
        for (unsigned i=0; i<args.args().size(); ++i)
          nargs.push_back(AnyReference(args.args()[i].type(), args.args()[i].rawValue()));
        T res = _f.call(nargs);
        result.setValue(AnyReference(res));
      }
      catch(const std::exception& e)
      {
        error.set(e.what());
      }
      catch(...)
      {
        error.set("Unknown exception");
      }
      running.set(false);
    }
    GenericProperty result;
    AnyFunction _f;
  };

  template<typename T>
  class TaskCall<Future<T> >: public Task
  {
  public:
    TaskCall()
    : result(typeOf<T>()) {}
    // Accept an explicit type that overrides
    TaskCall(AnyFunction f, TypeInterface* eff = 0)
    : result(eff?eff:typeOf<T>())
    , _f(f)
    , _live(new bool(true))
    {
      if (eff && qi::typeOf<T>()->info() != qi::typeOf<qi::AnyValue>()->info())
        qiLogWarning("TaskCall") << "Overriding TaskCall type for a template different from GenericValue";
      qiLogDebug("TaskCall") << "TaskCall " << this;
    }
    ~TaskCall()
    {
      qiLogDebug("TaskCall") << "~TaskCall " << this;
      *_live = false;
    }
    virtual void start(const AnyVarArguments& args)
    {
      qiLogDebug("TaskCall") << "start " << this;
      running.set(true);
      try
      {
        std::vector<AnyReference> nargs;
        nargs.reserve(args.args().size());
        for (unsigned i=0; i<args.args().size(); ++i)
          nargs.push_back(AnyReference(args.args()[i].type(), args.args()[i].rawValue()));
        AnyReference v = _f.call(nargs);
        _fut = *v.ptr<Future<T> >();
        v.destroy();
        _fut.connect(boost::bind(&TaskCall<Future<T> >::onResult, this, _1, _live));
      }
      catch(const std::exception& e)
      {
        error.set(e.what());
      }
      catch(...)
      {
        error.set("Unknown exception");
      }
    }
    void onResult(Future<T> fut, boost::shared_ptr<bool> live)
    {
      qiLogDebug("TaskCall") << "onResult " << this;
      if (!*_live) // shouldn't you be threadsafe or something?
        return;
      switch(fut.wait(0))
      {
      case FutureState_Canceled:
        error.set("interrupted");
        break;
      case FutureState_FinishedWithError:
        error.set(fut.error());
        break;
      case FutureState_FinishedWithValue:
        qiLogDebug("TaskCall") << "setResult " << encodeJSON(AnyReference::from(fut.value()));
        result.setValue(fut.value());
        break;
      case FutureState_None:
      case FutureState_Running:
        error.set("Inconsistent state");
        break;
      }
      running.set(false);
    }
    virtual bool interrupt()
    {
      try
      {
        _fut.cancel();
        return true;
      }
      catch(...)
      {
        return false;
      }
      return true; // for distracted compilers
    }
    GenericProperty result;
    AnyFunction _f;
    Future<T> _fut;
    boost::shared_ptr<bool> _live;
  };
}
QI_TEMPLATE_OBJECT(qi::TaskCall, start, interrupt, running, error, result);
#endif
