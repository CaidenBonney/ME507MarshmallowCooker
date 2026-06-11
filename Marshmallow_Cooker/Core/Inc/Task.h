#ifndef TASK_H
#define TASK_H

#include "main.h" // For print_str, and Error_Handler
#include "stm32f4xx_hal.h" // For HAL_GetTick

// Externs
extern char print_buf[100];

class Task {
public:
  enum class Status {
    Uninitialized,
    Running,
    Fault
  };

  virtual void run() = 0;
  virtual Status getStatus() const = 0;

  virtual ~Task() = default;
};

#endif /* TASK_H */
