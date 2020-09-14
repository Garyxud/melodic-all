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
#include <chrono>
#include <list>
#include <functional>
#include <iostream>
#include <mutex>

#include <dataflow_lite/utils/observable_object.h>
#include <dataflow_lite/utils/service.h>
#include <dataflow_lite/task/task.h>

/**
 * State of the publisher representing the last publisher attempt status.
 */
enum PublisherState {
    UNKNOWN, // constructed
    CONNECTED, //configured, ready to receive data
    NOT_CONNECTED, //unable to send data
};

/**
 * Base class implementing the Aws::FileManagement::IPublisher interface.
 *
 * Overriding classes should provide the mechanisms to initialize / configure their respective components.
 *
 * @tparam T the type to publish
 */
template <typename T>
class Publisher : public Aws::DataFlow::IPublisher<T>, public Service
{

public:

    Publisher() : publisher_state_(UNKNOWN) {
      publish_successes_.store(0);
      publish_attempts_.store(0);
      last_publish_duration_.store(std::chrono::milliseconds(0));
    }

    ~Publisher() override = default;

    /**
     * Return the current state of the publisher.
     *
     * @return
     */
    PublisherState getPublisherState() {
      return publisher_state_.getValue();
    }

    /**
     * Attempt to publish data to CloudWatch if this service is in the started state.
     *
     * @param data the data to publish
     * @return the resulting Aws::DataFlow::UploadStatus from the publish attempt. Returns FAIL if not in the started
     * state.
     */
    Aws::DataFlow::UploadStatus attemptPublish(T &data) override {

      // don't attempt to publish if not in the started state
      if(ServiceState::STARTED != this->getState()) {
        return Aws::DataFlow::UploadStatus::FAIL;
      }

      publish_attempts_++;
      auto published_status = Aws::DataFlow::UploadStatus::FAIL;

      // acquire lock to publish
      std::lock_guard<std::mutex> lck (publish_mutex_);
      {
        auto start = std::chrono::high_resolution_clock::now();
        published_status = publishData(data); // always at least try
        last_publish_duration_.store(std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::high_resolution_clock::now() - start));
      }

      if (Aws::DataFlow::UploadStatus::SUCCESS == published_status) {
        publish_successes_++;
        publisher_state_.setValue(CONNECTED);
      } else {
        publisher_state_.setValue(NOT_CONNECTED);
      }

      return published_status;
    }

    /**
     * Shutdown the publisher. This waits for attemptPublish to finish before returning.
     * @return the result of shutdown
     */
    bool shutdown() override {
      bool b = Service::shutdown();  // set shutdown state to try and fast fail any publish calls

      std::lock_guard<std::mutex> lck (publish_mutex_);
      //acquire the lock to ensure attemptPublish has finished
      publisher_state_.setValue(UNKNOWN);
      return b;
    }

    /**
     * Return true if this publisher can send data to CloudWatch, false otherwise.
     * @return
     */
    bool canPublish() {
      auto current_state = publisher_state_.getValue();
      return (current_state == UNKNOWN || current_state == CONNECTED) && ServiceState::STARTED == this->getState();
    }

    /**
     * Return the number of publish successes
     * @return
     */
    int getPublishSuccesses() {
      return publish_successes_.load();
    }

    /**
     * Return the number of attempts made to publish
     * @return
     */
    int getPublishAttempts() {
      return publish_attempts_.load();
    }

    /**
     * Return the time taken for the last publish attempt
     *
     * @return std::chrono::milliseconds the last publish attempt duration
     */
    std::chrono::milliseconds getLastPublishDuration() {
      return last_publish_duration_.load();
    }

    /**
     * Calculate and return the success rate of this publisher.
     *
     * @return the number of successes divided by the number of attempts. If zero attempts have been made
     * then return 0.
     */
    float getPublishSuccessPercentage() {
      int attempts = publish_attempts_.load();
      if (attempts == 0) {
        return 0;
      }
      int successes = publish_successes_.load();
      return static_cast<float>(successes) / static_cast<float>(attempts) * 100.0f;
    }

    /**
     * Provide the registration mechanism for ObservableObject (in this case PublisherState) changes.
     *
     * @param listener the PublisherState listener
     * @return true if the listener was added, false otherwise
     */
    virtual void addPublisherStateListener(const std::function<void(const PublisherState&)> & listener) {
      publisher_state_.addListener(listener);
    }

protected:

    /**
     * Actual publishing mechanism implemented by the agent.
     *
     * @param data
     * @return the Aws::DataFlow::UploadStatus resulting from the implemented attempt
     */
    virtual Aws::DataFlow::UploadStatus publishData(T &data) = 0;

private:
    /**
     * Track the publish state in a thread safe container which can provide events to registered listeners.
     */
    ObservableObject<PublisherState> publisher_state_;
    /**
     * Number of publish successes
     */
    std::atomic<int> publish_successes_{};
    /**
     * Number of publish attempts
     */
    std::atomic<int> publish_attempts_{};
    /**
     * The amount of time taken for the last publish action
     */
    std::atomic<std::chrono::milliseconds> last_publish_duration_{};
    /**
     * Mutex used in publish and shutdown
     */
    mutable std::mutex publish_mutex_;

};