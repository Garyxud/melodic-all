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

#include <chrono>
#include <mutex>

/**
 * Class used to provide easy mechanism to wait and notify.
 */
class Waiter
{
public:

    Waiter() {}
    ~Waiter() = default;

    /**
     * Wait until notified (blocking).
     */
    void wait() {
      std::unique_lock<std::mutex> lck(this->mtx);
      cv.wait(lck);
    }

    /**
     * Wait until notified or timed out.
     * @param millis milliseconds
     */
    void wait_for_millis(std::chrono::milliseconds millis) {
      std::unique_lock<std::mutex> lck(this->mtx);
      cv.wait_for(lck, millis);
    }

    /**
     * Wait until notified or timed out.
     * @param seconds seconds
     */
    void wait_for(std::chrono::seconds seconds) {
      std::unique_lock<std::mutex> lck(this->mtx);
      cv.wait_for(lck, seconds);
    }

    /**
     * Notify all waiters.
     */
    void notify() {
      std::unique_lock<std::mutex> lck(this->mtx);
      this->cv.notify_all(); //don't leave anyone blocking
    }

private:
    std::condition_variable cv;
    mutable std::mutex mtx;
    // todo need a count to guard against spurious wakeups
};
