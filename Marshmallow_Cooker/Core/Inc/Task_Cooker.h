#ifndef TASK_COOKER_H
#define TASK_COOKER_H

/**
 * @file Task_Cooker.h
 * @brief Top-level cooking sequence state machine.
 * @details
 *   TaskCooker coordinates the user interface, temperature task, rotisserie
 *   motor task, and vertical Z-axis task. It implements the high-level user
 *   commands such as home, start, stop, emergency stop, reset, and status. The
 *   class intentionally contains the cooking policy while lower-level tasks own
 *   individual motor and sensor details.
 */

#include "Task.h"
#include "Task_R_Motor.h"
#include "Task_Temps.h"
#include "Task_UI.h"
#include "Task_Z_Motor.h"

#include <cstdlib>

/**
 * @class TaskCooker
 * @brief Supervises the marshmallow cooker operating sequence.
 * @details
 *   The cooker task consumes parsed UI commands and translates them into safe
 *   actions on the R and Z motor tasks. It prevents cooking before homing,
 *   starts temperature control, monitors motor faults, declares completion when
 *   the IR done condition is met, and routes the system to a safe removal
 *   height or fault state as needed.
 */
class TaskCooker : public Task {
public:
  /**
   * @enum State
   * @brief Detailed top-level cooker state.
   */
  enum class State {
    Uninitialized,        /**< References are available but the task has not printed its startup prompt. */
    WaitingForHomeCommand,/**< System is idle and requires homing before cooking can start. */
    HomingZ,              /**< Z axis is homing and the R axis may be returning to its zero orientation. */
    ReadyToCook,          /**< Setup is complete and a start command will begin cooking. */
    Cooking,              /**< Active cooking sequence is running. */
    MovingToRemovalHeight,/**< Stop or done sequence is moving mechanisms to removal position. */
    Done,                 /**< Cook cycle is complete and reset is required before the next cook. */
    Fault                 /**< A fault or emergency stop has halted operation. */
  };

  /**
   * @brief Construct the top-level cooker task.
   * @param task_ui Reference to the UART command task.
   * @param task_temps Reference to the temperature acquisition task.
   * @param task_r_motor Reference to the rotisserie motor task.
   * @param task_z_motor Reference to the vertical Z-axis motor task.
   */
  TaskCooker(TaskUI& task_ui, TaskTemps& task_temps, TaskRMotor& task_r_motor, TaskZMotor& task_z_motor);

  /**
   * @brief Execute one non-blocking update of the cooker state machine.
   * @details
   *   Consumes at most one pending UI command and then services the active
   *   cooker state. This method does not directly generate motor step pulses;
   *   it issues high-level commands to the motor tasks.
   */
  void run() override;

  /**
   * @brief Get the generic task health status.
   * @return Task::Status::Fault when faulted, otherwise Running after initialization.
   */
  Status getStatus() const override;

  /**
   * @brief Get the detailed cooker state.
   * @return Current TaskCooker::State value.
   */
  State getState() const;

private:
  /** @brief Thermocouple flame-temperature target in degrees F x100. */
  static constexpr int16_t kTargetFlameTempFx100 = 35000; // TODO: tune. 350.00 F.

  /** @brief IR marshmallow done threshold in degrees F x100. */
  static constexpr int16_t kDoneMarshmallowTempFx100 = 16000; // TODO: tune. 160.00 F.

  /** @brief Current top-level state of the cooker task. */
  State state_ = State::Uninitialized;

  /** @brief UART command and status interface. */
  TaskUI& task_ui_;

  /** @brief Thermocouple and IR sensor task. */
  TaskTemps& task_temps_;

  /** @brief Rotisserie motor task. */
  TaskRMotor& task_r_motor_;

  /** @brief Vertical Z-axis motor task. */
  TaskZMotor& task_z_motor_;

  /**
   * @brief Dispatch a parsed UI command.
   * @param command Command value consumed from TaskUI.
   */
  void handleCommand(TaskUI::Command command);

  /**
   * @brief Enter the cooker fault state and stop motion.
   * @param reason Null-terminated text describing the cause of the fault.
   */
  void enterFault(const char* reason);

  /**
   * @brief Print a one-line snapshot of cooker, motor, and temperature status.
   */
  void printStatus() const;
};

#endif /* TASK_COOKER_H */
