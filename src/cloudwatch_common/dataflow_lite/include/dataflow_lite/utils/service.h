/*
 * Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */

#pragma once

#include <atomic>
#include <condition_variable>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <typeinfo>

#include <dataflow_lite/utils/observable_object.h>

enum ServiceState {
    CREATED,  // created and ready to start
    STARTED,  // started and ready to use
    SHUTDOWN,  // clean shutdown, possibly restartable
};

/**
 * Map used to pretty print the ServiceState enum.
 */
static std::map<ServiceState, std::string> SERVICE_STATE_NAME_MAP = {{CREATED, "CREATED"}, {STARTED, "STARTED"},
                                                                     {SHUTDOWN,"SHUTDOWN"}};

/**
 * Interface that defines init, start, and shutdown methods for an implementing class to define.
 */
class Service {
public:

    Service() : state_(CREATED) {};
    virtual ~Service() = default;

    /**
     * Called to start doing work. The format overriding classes should use is the following:
     *
     *   virtual bool start() {
     *       // do specific start logic here
     *
     *       // ensure the service state has been set to started
     *       bool b = Service::start();
     *
     *       // return the result of Service::start() or something else if desired
     *       return b;
     *   }
     *
     * @return
     */
    virtual bool start() {
      state_.setValue(STARTED);
      return true;
    }

    /**
     * Cleanup. Should be called before destruction. The format overriding classes should use is the following:
     *
     *   virtual bool shutdown() {
     *       //  immediately set the shutdown state
     *       bool b = Service::shutdown();
     *
     *       // do specific shutdown logic here
     *
     *       // return the result of Service::shutdown() or something else if desired
     *       return b;
     *   }
     * @return
     */
    virtual bool shutdown() {
      state_.setValue(SHUTDOWN);
      return true;
    }

    /**
     * Return a descriptive string describing the service and it's state.
     * @return
     */
    virtual std::string getStatusString() {
      // a more descriptive name (tag supplied on construction) would be ideal
      return typeid(this).name() + std::string(", state=") + SERVICE_STATE_NAME_MAP[getState()];
    }

    /**
     * Return the current ServiceState if this service.
     * @return ServiceState
     */
    ServiceState getState() {
      return state_.getValue();
    }

protected:
    /**
     * Set the current state of the service. To be used by overriding classes.
     *
     * @param new_state
     */
    void setState(ServiceState new_state) {
      state_.setValue(new_state);
    }

private:
    /**
     * The current state of this service.
     */
    ObservableObject<ServiceState> state_;
};

/**
 * Interface that implements basic Service methods and provides a mechanism to start a thread. Any interesting work
 * in the implemented class should be implemented in the "work" method. Note: start and shutdown methods should be
 * override if the implementing class requires extra steps in either scenarios.
 */
//  consider extending the waiter interface
class RunnableService : public Service
{
public:
    RunnableService() {
      should_run_.store(false);
    }
    ~RunnableService() override = default;

    /**
     * Starts the worker thread. Should be overridden if other actions are necessary to start.
     * @return
     */
    bool start() override {
      bool started = startWorkerThread();
      started &= Service::start();
      return started;
    }

    /**
     * Stops the worker thread. Should be overridden if other actions are necessary to stop.
     * @return
     */
    bool shutdown() override {
      bool is_shutdown = Service::shutdown();
      is_shutdown &= stopWorkerThread();
      return is_shutdown;
    }

    /**
     * Return if the RunnableService work thread is active
     * @return true if the work thread is active / running, false otherwise
     */
    virtual bool isRunning() {
      return Service::getState() == ServiceState::STARTED && should_run_.load();
    }

    /**
     * Wait for the work thread shutdown
     */
    void waitForShutdown() {
      std::unique_lock <std::mutex> lck(this->mtx_);
      if (runnable_thread_.joinable()) {
        cv_.wait(lck); // todo guard against spurious wakeup, or could do while
      }
    }

    /**
     * Wait for the work thread shutdown
     * @param millis time to wait
     */
    void waitForShutdown(std::chrono::milliseconds millis) {
      std::unique_lock <std::mutex> lck(this->mtx_);
      if (runnable_thread_.joinable()) {
        cv_.wait_for(lck, millis); // todo guard against spurious wakeup
      }
    }

    /**
     * Join the running thread if available.
     */
    void join() {
      if (runnable_thread_.joinable()) {
        runnable_thread_.join();
      }
    }

    /**
     * Return a descriptive string describing the service and it's state.
     * @return
     */
    std::string getStatusString() override {
      return Service::getStatusString() + std::string(", isRunning=") + (isRunning()
        ? std::string("True") : std::string("False"));
    }

protected:

  /**
   * Start the worker thread if not already running.
   * @return true if the worker thread was started, false if already running
   */
  virtual bool startWorkerThread() {
    if(!runnable_thread_.joinable()) {
      should_run_.store(true);
      runnable_thread_ = std::thread(&RunnableService::run, this);
      return true;
    }
    return false;
  }

  /**
   * Set the should_run_ flag to false for the worker thread to exit
   * @return
   */
  virtual bool stopWorkerThread() {
    if(should_run_.load()) {
      should_run_.store(false);
      return true;
    }
    return false;
  }

  /**
   * Calls the abstract work method and notifies when shutdown was called.
   */
  virtual void run() {
    while(should_run_.load() && ServiceState::STARTED == Service::getState()) {
      work();
    }
    // done, notify anyone waiting
    std::unique_lock <std::mutex> lck(this->mtx_);
    this->cv_.notify_all();
  }

  /**
   * Implement this method to do work. Note: this method is assumed to NOT block, otherwise shutdown does nothing.
   */
  virtual void work() = 0;

private:
  std::thread runnable_thread_;
  std::atomic<bool> should_run_{};
  std::condition_variable cv_;
  mutable std::mutex mtx_;
};
