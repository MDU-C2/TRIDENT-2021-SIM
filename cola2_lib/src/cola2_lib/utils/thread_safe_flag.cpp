/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

#include <cola2_lib/utils/thread_safe_flag.h>
#include <chrono>

namespace cola2
{
namespace utils
{
ThreadSafeFlag::ThreadSafeFlag() : state_(false)
{
}

ThreadSafeFlag::ThreadSafeFlag(const bool state) : state_(state)
{
}

void ThreadSafeFlag::setState(const bool state)
{
  {
    std::unique_lock<std::mutex> lck(mtx_);
    state_ = state;
  }
  cv_.notify_all();
}

bool ThreadSafeFlag::getState()
{
  std::unique_lock<std::mutex> lck(mtx_);
  return state_;
}

void ThreadSafeFlag::blockingWaitFor(const bool desired_state)
{
  std::unique_lock<std::mutex> lck(mtx_);
  while (state_ != desired_state)
  {
    // The program flow is stopped in wait. The wait method releases the mutex, and it takes the mutex back
    // when it returns
    cv_.wait(lck);
  }
}

void ThreadSafeFlag::timedBlockingWaitFor(const bool desired_state, const double timeout_sec)
{
  const auto start = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration<double>(timeout_sec);
  std::unique_lock<std::mutex> lck(mtx_);
  while (state_ != desired_state)
  {
    cv_.wait_for(lck, duration);

    const auto now = std::chrono::steady_clock::now();
    const double remaining_sec =
        timeout_sec - std::chrono::duration_cast<std::chrono::duration<double> >(now - start).count();
    if (remaining_sec <= 0.0)
    {
      break;
    }
    else
    {
      duration = std::chrono::duration<double>(remaining_sec);
    }
  }
}
}  // namespace utils
}  // namespace cola2
