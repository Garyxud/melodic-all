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

#ifndef __EchoSample_h__
#define __EchoSample_h__

#include <Ice/LocalObjectF.h>
#include <Ice/ProxyF.h>
#include <Ice/ObjectF.h>
#include <Ice/Exception.h>
#include <Ice/LocalObject.h>
#include <Ice/Proxy.h>
#include <Ice/Object.h>
#include <Ice/Outgoing.h>
#include <Ice/Incoming.h>
#include <Ice/Direct.h>
#include <Ice/StreamF.h>
#include <Ice/UndefSysMacros.h>

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

namespace IceProxy
{

namespace Demo
{

class EchoSample;
bool operator==(const EchoSample&, const EchoSample&);
bool operator!=(const EchoSample&, const EchoSample&);
bool operator<(const EchoSample&, const EchoSample&);
bool operator<=(const EchoSample&, const EchoSample&);
bool operator>(const EchoSample&, const EchoSample&);
bool operator>=(const EchoSample&, const EchoSample&);

}

}

namespace Demo
{

class EchoSample;
bool operator==(const EchoSample&, const EchoSample&);
bool operator!=(const EchoSample&, const EchoSample&);
bool operator<(const EchoSample&, const EchoSample&);
bool operator<=(const EchoSample&, const EchoSample&);
bool operator>(const EchoSample&, const EchoSample&);
bool operator>=(const EchoSample&, const EchoSample&);

}

namespace IceInternal
{

void incRef(::Demo::EchoSample*);
void decRef(::Demo::EchoSample*);

void incRef(::IceProxy::Demo::EchoSample*);
void decRef(::IceProxy::Demo::EchoSample*);

}

namespace Demo
{

typedef ::IceInternal::Handle< ::Demo::EchoSample> EchoSamplePtr;
typedef ::IceInternal::ProxyHandle< ::IceProxy::Demo::EchoSample> EchoSamplePrx;

void __write(::IceInternal::BasicStream*, const EchoSamplePrx&);
void __read(::IceInternal::BasicStream*, EchoSamplePrx&);
void __write(::IceInternal::BasicStream*, const EchoSamplePtr&);
void __patch__EchoSamplePtr(void*, ::Ice::ObjectPtr&);

void __addObject(const EchoSamplePtr&, ::IceInternal::GCCountMap&);
bool __usesClasses(const EchoSamplePtr&);
void __decRefUnsafe(const EchoSamplePtr&);
void __clearHandleUnsafe(EchoSamplePtr&);

}

namespace Demo
{

}

namespace IceProxy
{

namespace Demo
{

class EchoSample : virtual public ::IceProxy::Ice::Object
{
public:

    void echo(const ::std::string& msg)
    {
        echo(msg, 0);
    }
    void echo(const ::std::string& msg, const ::Ice::Context& __ctx)
    {
        echo(msg, &__ctx);
    }
    
private:

    void echo(const ::std::string&, const ::Ice::Context*);
    
public:
    
    static const ::std::string& ice_staticId();

private: 

    virtual ::IceInternal::Handle< ::IceDelegateM::Ice::Object> __createDelegateM();
    virtual ::IceInternal::Handle< ::IceDelegateD::Ice::Object> __createDelegateD();
};

}

}

namespace IceDelegate
{

namespace Demo
{

class EchoSample : virtual public ::IceDelegate::Ice::Object
{
public:

    virtual void echo(const ::std::string&, const ::Ice::Context*) = 0;
};

}

}

namespace IceDelegateM
{

namespace Demo
{

class EchoSample : virtual public ::IceDelegate::Demo::EchoSample,
                   virtual public ::IceDelegateM::Ice::Object
{
public:

    virtual void echo(const ::std::string&, const ::Ice::Context*);
};

}

}

namespace IceDelegateD
{

namespace Demo
{

class EchoSample : virtual public ::IceDelegate::Demo::EchoSample,
                   virtual public ::IceDelegateD::Ice::Object
{
public:

    virtual void echo(const ::std::string&, const ::Ice::Context*);
};

}

}

namespace Demo
{

class EchoSample : virtual public ::Ice::Object
{
public:

    typedef EchoSamplePrx ProxyType;
    typedef EchoSamplePtr PointerType;
    
    virtual ::Ice::ObjectPtr ice_clone() const;

    virtual bool ice_isA(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) const;
    virtual ::std::vector< ::std::string> ice_ids(const ::Ice::Current& = ::Ice::Current()) const;
    virtual const ::std::string& ice_id(const ::Ice::Current& = ::Ice::Current()) const;
    static const ::std::string& ice_staticId();

    virtual void echo(const ::std::string&, const ::Ice::Current& = ::Ice::Current()) = 0;
    ::IceInternal::DispatchStatus ___echo(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual ::IceInternal::DispatchStatus __dispatch(::IceInternal::Incoming&, const ::Ice::Current&);

    virtual void __write(::IceInternal::BasicStream*) const;
    virtual void __read(::IceInternal::BasicStream*, bool);
    virtual void __write(const ::Ice::OutputStreamPtr&) const;
    virtual void __read(const ::Ice::InputStreamPtr&, bool);
};

void __patch__EchoSamplePtr(void*, ::Ice::ObjectPtr&);

}

#endif
