#ifndef TASK_H
#define TASK_H

/**
 * @file Task.h
 * @brief Abstract base interface for cooperative firmware tasks.
 * @details
 *   The marshmallow cooker firmware is organized as a set of cooperative
 *   tasks. Each task owns one subsystem and exposes a small common interface
 *   for periodic execution and high-level health reporting. The main loop calls
 *   each task's run() method frequently; tasks return quickly rather than
 *   blocking so sensor reads, command processing, and motor updates can share
 *   processor time.
 */

#include "main.h" // For print_str, and Error_Handler
#include "stm32f4xx_hal.h" // For HAL_GetTick

/**
 * @brief Shared formatted-print buffer used by task status messages.
 * @details
 *   The buffer is allocated in main.cpp and used by several tasks before
 *   passing the resulting null-terminated text to print_str().
 */
extern char print_buf[100];

/**
 * @class Task
 * @brief Common interface for all cooperative state-machine tasks.
 * @details
 *   Derived classes implement run() for non-blocking periodic work and
 *   getStatus() for coarse health reporting. More detailed task-specific state
 *   is provided by each child class through its own state enum.
 */
class Task {
public:
  /**
   * @enum Status
   * @brief Generic task health states used by the top-level scheduler.
   */
  enum class Status {
    Uninitialized, /**< Task has not completed its initialization sequence. */
    Running, /**< Task is initialized and not faulted. */
    Fault /**< Task detected a condition that prevents normal operation. */
  };

  /**
   * @brief Execute one non-blocking service pass for the task.
   * @details
   *   Implementations should return quickly and avoid long blocking waits so
   *   the main loop can continue servicing all other tasks.
   */
  virtual void run() = 0;

  /**
   * @brief Get the generic health status of the task.
   * @return Current generic status for scheduler-level monitoring.
   */
  virtual Status getStatus() const = 0;

  /**
   * @brief Virtual destructor for safe cleanup through a Task pointer.
   */
  virtual ~Task() = default;
};

#endif /* TASK_H */
