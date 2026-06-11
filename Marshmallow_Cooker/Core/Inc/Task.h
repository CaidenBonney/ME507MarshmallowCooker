#ifndef TASK_H
#define TASK_H

/**
 * @file Task.h
 * @brief Abstract base class for cooperative application tasks.
 */

#include "main.h"           ///< Project-wide declarations, including print_str and Error_Handler.
#include "stm32f4xx_hal.h"  ///< HAL timing and peripheral definitions.

/** @brief Shared printf-style buffer used by task status messages. */
extern char print_buf[100];

/**
 * @brief Common interface for all cooperative firmware tasks.
 *
 * Each task owns a small state machine and is called repeatedly from the main
 * loop. The interface keeps high-level task scheduling uniform without forcing
 * unrelated tasks to share implementation details.
 */
class Task {
public:
  /** @brief Generic task health state used by the task supervisor. */
  enum class Status {
    Uninitialized,  ///< Task has not completed startup initialization.
    Running,        ///< Task is initialized and not faulted.
    Fault           ///< Task has detected a fault condition.
  };

  /**
   * @brief Run one non-blocking iteration of the task state machine.
   */
  virtual void run() = 0;

  /**
   * @brief Get the task-level health status.
   * @return Generic task status.
   */
  virtual Status getStatus() const = 0;

  /** @brief Virtual destructor for safe base-class destruction. */
  virtual ~Task() = default;
};

#endif /* TASK_H */
