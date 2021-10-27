/*
 * Copyright (c) 2020 Iqua Robotics SL - All Rights Reserved
 *
 * This file is subject to the terms and conditions defined in file
 * 'LICENSE.txt', which is part of this source code package.
 */

/**
 * @file
 * @brief This file contains a class that implements a thread safe flag.
 */

#pragma once

#include <condition_variable>
#include <mutex>

namespace cola2
{
namespace utils
{
/**
 * @addtogroup utils
 * @{
 */

/**
 * @brief This class implements a thread safe flag that can be used to synchronize different threads.
 */
class ThreadSafeFlag
{
protected:
  std::mutex mtx_;
  std::condition_variable cv_;
  bool state_;

public:
  /**
   * @brief Default constructor. Sets the state to false.
   */
  ThreadSafeFlag();

  /**
   * @brief Constructor with the initial state as a parameter.
   *
   * @param state Initial state.
   */
  explicit ThreadSafeFlag(const bool state);

  /**
   * @brief Set the state to the given value.
   *
   * @param state State.
   */
  void setState(const bool state);

  /**
   * @brief Returns the state.
   *
   * @return State.
   */
  bool getState();

  /**
   * @brief This method blocks until the flag is set to the given state.
   *
   * @param state Desired state.
   */
  void blockingWaitFor(const bool state);

  /**
   * @brief This method blocks until the flag is set to the given state or the timeout expires.
   *
   * @param state Desired state.
   * @param timeout Timeout in seconds.
   */
  void timedBlockingWaitFor(const bool state, const double timeout);
};

/** @} */
}  // namespace utils
}  // namespace cola2
