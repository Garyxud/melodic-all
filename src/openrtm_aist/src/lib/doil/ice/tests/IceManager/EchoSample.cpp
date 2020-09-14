// **********************************************************************
//
// Copyright (c) 2003-2007 ZeroC, Inc. All rights reserved.
//
// This copy of Ice is licensed to you under the terms described in the
// ICE_LICENSE file included in this distribution.
//
// **********************************************************************

// Ice version 3.2.1
// Generated from file `EchoSample.ice'

#include <EchoSample.h>
#include <Ice/LocalException.h>
#include <Ice/ObjectFactory.h>
#include <Ice/BasicStream.h>
#include <Ice/Object.h>
#include <IceUtil/Iterator.h>
#include <IceUtil/ScopedArray.h>

#ifndef ICE_IGNORE_VERSION
#   if ICE_INT_VERSION / 100 != 302
#       error Ice version mismatch!
#   endif
#   if ICE_INT_VERSION % 100 > 50
#       error Beta header file detected
#   endif
#   if ICE_INT_VERSION % 100 < 1
#       error Ice patch level mismatch!
#   endif
#endif

static const ::std::string __Demo__EchoSample__echo_name = "echo";

void
IceInternal::incRef(::Demo::EchoSample* p)
{
    p->__incRef();
}

void
IceInternal::decRef(::Demo::EchoSample* p)
{
    p->__decRef();
}

void
IceInternal::incRef(::IceProxy::Demo::EchoSample* p)
{
    p->__incRef();
}

void
IceInternal::decRef(::IceProxy::Demo::EchoSample* p)
{
    p->__decRef();
}

void
Demo::__write(::IceInternal::BasicStream* __os, const ::Demo::EchoSamplePrx& v)
{
    __os->write(::Ice::ObjectPrx(v));
}

void
Demo::__read(::IceInternal::BasicStream* __is, ::Demo::EchoSamplePrx& v)
{
    ::Ice::ObjectPrx proxy;
    __is->read(proxy);
    if(!proxy)
    {
        v = 0;
    }
    else
    {
        v = new ::IceProxy::Demo::EchoSample;
        v->__copyFrom(proxy);
    }
}

void
Demo::__write(::IceInternal::BasicStream* __os, const ::Demo::EchoSamplePtr& v)
{
    __os->write(::Ice::ObjectPtr(v));
}

void
Demo::__addObject(const EchoSamplePtr& p, ::IceInternal::GCCountMap& c)
{
    p->__addObject(c);
}

bool
Demo::__usesClasses(const EchoSamplePtr& p)
{
    return p->__usesClasses();
}

void
Demo::__decRefUnsafe(const EchoSamplePtr& p)
{
    p->__decRefUnsafe();
}

void
Demo::__clearHandleUnsafe(EchoSamplePtr& p)
{
    p.__clearHandleUnsafe();
}

void
IceProxy::Demo::EchoSample::echo(const ::std::string& msg, const ::Ice::Context* __ctx)
{
    int __cnt = 0;
    while(true)
    {
        ::IceInternal::Handle< ::IceDelegate::Ice::Object> __delBase;
        try
        {
            __delBase = __getDelegate();
            ::IceDelegate::Demo::EchoSample* __del = dynamic_cast< ::IceDelegate::Demo::EchoSample*>(__delBase.get());
            __del->echo(msg, __ctx);
            return;
        }
        catch(const ::IceInternal::LocalExceptionWrapper& __ex)
        {
            __handleExceptionWrapper(__delBase, __ex);
        }
        catch(const ::Ice::LocalException& __ex)
        {
            __handleException(__delBase, __ex, __cnt);
        }
    }
}

const ::std::string&
IceProxy::Demo::EchoSample::ice_staticId()
{
    return ::Demo::EchoSample::ice_staticId();
}

::IceInternal::Handle< ::IceDelegateM::Ice::Object>
IceProxy::Demo::EchoSample::__createDelegateM()
{
    return ::IceInternal::Handle< ::IceDelegateM::Ice::Object>(new ::IceDelegateM::Demo::EchoSample);
}

::IceInternal::Handle< ::IceDelegateD::Ice::Object>
IceProxy::Demo::EchoSample::__createDelegateD()
{
    return ::IceInternal::Handle< ::IceDelegateD::Ice::Object>(new ::IceDelegateD::Demo::EchoSample);
}

bool
IceProxy::Demo::operator==(const ::IceProxy::Demo::EchoSample& l, const ::IceProxy::Demo::EchoSample& r)
{
    return static_cast<const ::IceProxy::Ice::Object&>(l) == static_cast<const ::IceProxy::Ice::Object&>(r);
}

bool
IceProxy::Demo::operator!=(const ::IceProxy::Demo::EchoSample& l, const ::IceProxy::Demo::EchoSample& r)
{
    return static_cast<const ::IceProxy::Ice::Object&>(l) != static_cast<const ::IceProxy::Ice::Object&>(r);
}

bool
IceProxy::Demo::operator<(const ::IceProxy::Demo::EchoSample& l, const ::IceProxy::Demo::EchoSample& r)
{
    return static_cast<const ::IceProxy::Ice::Object&>(l) < static_cast<const ::IceProxy::Ice::Object&>(r);
}

bool
IceProxy::Demo::operator<=(const ::IceProxy::Demo::EchoSample& l, const ::IceProxy::Demo::EchoSample& r)
{
    return l < r || l == r;
}

bool
IceProxy::Demo::operator>(const ::IceProxy::Demo::EchoSample& l, const ::IceProxy::Demo::EchoSample& r)
{
    return !(l < r) && !(l == r);
}

bool
IceProxy::Demo::operator>=(const ::IceProxy::Demo::EchoSample& l, const ::IceProxy::Demo::EchoSample& r)
{
    return !(l < r);
}

void
IceDelegateM::Demo::EchoSample::echo(const ::std::string& msg, const ::Ice::Context* __context)
{
    ::IceInternal::Outgoing __og(__connection.get(), __reference.get(), __Demo__EchoSample__echo_name, ::Ice::Normal, __context, __compress);
    try
    {
        ::IceInternal::BasicStream* __os = __og.os();
        __os->write(msg);
    }
    catch(const ::Ice::LocalException& __ex)
    {
        __og.abort(__ex);
    }
    bool __ok = __og.invoke();
    try
    {
        ::IceInternal::BasicStream* __is = __og.is();
        if(!__ok)
        {
            try
            {
                __is->throwException();
            }
            catch(const ::Ice::UserException& __ex)
            {
                throw ::Ice::UnknownUserException(__FILE__, __LINE__, __ex.ice_name());
            }
        }
    }
    catch(const ::Ice::LocalException& __ex)
    {
        throw ::IceInternal::LocalExceptionWrapper(__ex, false);
    }
}

void
IceDelegateD::Demo::EchoSample::echo(const ::std::string& msg, const ::Ice::Context* __context)
{
    ::Ice::Current __current;
    __initCurrent(__current, __Demo__EchoSample__echo_name, ::Ice::Normal, __context);
    while(true)
    {
        ::IceInternal::Direct __direct(__current);
        try
        {
            ::Demo::EchoSample* __servant = dynamic_cast< ::Demo::EchoSample*>(__direct.servant().get());
            if(!__servant)
            {
                ::Ice::OperationNotExistException __opEx(__FILE__, __LINE__);
                __opEx.id = __current.id;
                __opEx.facet = __current.facet;
                __opEx.operation = __current.operation;
                throw __opEx;
            }
            try
            {
                __servant->echo(msg, __current);
            }
            catch(const ::Ice::LocalException& __ex)
            {
                throw ::IceInternal::LocalExceptionWrapper(__ex, false);
            }
        }
        catch(...)
        {
            __direct.destroy();
            throw;
        }
        __direct.destroy();
        return;
    }
}

::Ice::ObjectPtr
Demo::EchoSample::ice_clone() const
{
    throw ::Ice::CloneNotImplementedException(__FILE__, __LINE__);
    return 0; // to avoid a warning with some compilers
}

static const ::std::string __Demo__EchoSample_ids[2] =
{
    "::Demo::EchoSample",
    "::Ice::Object"
};

bool
Demo::EchoSample::ice_isA(const ::std::string& _s, const ::Ice::Current&) const
{
    return ::std::binary_search(__Demo__EchoSample_ids, __Demo__EchoSample_ids + 2, _s);
}

::std::vector< ::std::string>
Demo::EchoSample::ice_ids(const ::Ice::Current&) const
{
    return ::std::vector< ::std::string>(&__Demo__EchoSample_ids[0], &__Demo__EchoSample_ids[2]);
}

const ::std::string&
Demo::EchoSample::ice_id(const ::Ice::Current&) const
{
    return __Demo__EchoSample_ids[0];
}

const ::std::string&
Demo::EchoSample::ice_staticId()
{
    return __Demo__EchoSample_ids[0];
}

::IceInternal::DispatchStatus
Demo::EchoSample::___echo(::IceInternal::Incoming&__inS, const ::Ice::Current& __current)
{
    __checkMode(::Ice::Normal, __current.mode);
    ::IceInternal::BasicStream* __is = __inS.is();
    ::std::string msg;
    __is->read(msg);
    echo(msg, __current);
    return ::IceInternal::DispatchOK;
}

static ::std::string __Demo__EchoSample_all[] =
{
    "echo",
    "ice_id",
    "ice_ids",
    "ice_isA",
    "ice_ping"
};

::IceInternal::DispatchStatus
Demo::EchoSample::__dispatch(::IceInternal::Incoming& in, const ::Ice::Current& current)
{
    ::std::pair< ::std::string*, ::std::string*> r = ::std::equal_range(__Demo__EchoSample_all, __Demo__EchoSample_all + 5, current.operation);
    if(r.first == r.second)
    {
        return ::IceInternal::DispatchOperationNotExist;
    }

    switch(r.first - __Demo__EchoSample_all)
    {
        case 0:
        {
            return ___echo(in, current);
        }
        case 1:
        {
            return ___ice_id(in, current);
        }
        case 2:
        {
            return ___ice_ids(in, current);
        }
        case 3:
        {
            return ___ice_isA(in, current);
        }
        case 4:
        {
            return ___ice_ping(in, current);
        }
    }

    assert(false);
    return ::IceInternal::DispatchOperationNotExist;
}

void
Demo::EchoSample::__write(::IceInternal::BasicStream* __os) const
{
    __os->writeTypeId(ice_staticId());
    __os->startWriteSlice();
    __os->endWriteSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__write(__os);
#else
    ::Ice::Object::__write(__os);
#endif
}

void
Demo::EchoSample::__read(::IceInternal::BasicStream* __is, bool __rid)
{
    if(__rid)
    {
        ::std::string myId;
        __is->readTypeId(myId);
    }
    __is->startReadSlice();
    __is->endReadSlice();
#if defined(_MSC_VER) && (_MSC_VER < 1300) // VC++ 6 compiler bug
    Object::__read(__is, true);
#else
    ::Ice::Object::__read(__is, true);
#endif
}

void
Demo::EchoSample::__write(const ::Ice::OutputStreamPtr&) const
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Demo::EchoSample was not generated with stream support";
    throw ex;
}

void
Demo::EchoSample::__read(const ::Ice::InputStreamPtr&, bool)
{
    Ice::MarshalException ex(__FILE__, __LINE__);
    ex.reason = "type Demo::EchoSample was not generated with stream support";
    throw ex;
}

void 
Demo::__patch__EchoSamplePtr(void* __addr, ::Ice::ObjectPtr& v)
{
    ::Demo::EchoSamplePtr* p = static_cast< ::Demo::EchoSamplePtr*>(__addr);
    assert(p);
    *p = ::Demo::EchoSamplePtr::dynamicCast(v);
    if(v && !*p)
    {
        ::Ice::UnexpectedObjectException e(__FILE__, __LINE__);
        e.type = v->ice_id();
        e.expectedType = ::Demo::EchoSample::ice_staticId();
        throw e;
    }
}

bool
Demo::operator==(const ::Demo::EchoSample& l, const ::Demo::EchoSample& r)
{
    return static_cast<const ::Ice::Object&>(l) == static_cast<const ::Ice::Object&>(r);
}

bool
Demo::operator!=(const ::Demo::EchoSample& l, const ::Demo::EchoSample& r)
{
    return static_cast<const ::Ice::Object&>(l) != static_cast<const ::Ice::Object&>(r);
}

bool
Demo::operator<(const ::Demo::EchoSample& l, const ::Demo::EchoSample& r)
{
    return static_cast<const ::Ice::Object&>(l) < static_cast<const ::Ice::Object&>(r);
}

bool
Demo::operator<=(const ::Demo::EchoSample& l, const ::Demo::EchoSample& r)
{
    return l < r || l == r;
}

bool
Demo::operator>(const ::Demo::EchoSample& l, const ::Demo::EchoSample& r)
{
    return !(l < r) && !(l == r);
}

bool
Demo::operator>=(const ::Demo::EchoSample& l, const ::Demo::EchoSample& r)
{
    return !(l < r);
}
