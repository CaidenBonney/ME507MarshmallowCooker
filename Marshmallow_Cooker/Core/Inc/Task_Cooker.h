#ifndef TASK_COOKER_H
#define TASK_COOKER_H

/**
 * @file Task_Cooker.h
 * @brief Top-level marshmallow cooker state machine.
 */

// Parent class include
#include "Task.h"

// User created includes
#include "Task_R_Motor.h"
#include "Task_Temps.h"
#include "Task_UI.h"
#include "Task_Z_Motor.h"

// Additional includes
#include <cstdint>

// Externs
/** @brief UART print helper provided by main.cpp. */
extern void print_str(const char* str);

/** @brief Shared formatted-print buffer provided by main.cpp. */
extern char print_buf[100];

/**
 * @brief Coordinates UI commands, temperature sensing, and both motor tasks.
 *
 * TaskCooker owns the user-facing cooking sequence: home/setup, start cooking,
 * monitor done temperature, handle normal stops, and enter faults on unsafe
 * conditions.
 */
class TaskCooker : public Task {
public:
  /** @brief High-level cooker state. */
  enum class State {
    Uninitialized, ///< Cooker has not completed startup.
    WaitingForHomeCommand, ///< Waiting for the user to send home.
    HomingZ, ///< Returning R to zero and homing Z.
    ReadyToCook, ///< Setup complete and ready to start.
    ManualRotating, ///< R motor is manually rotating without PID cooking control.
    Cooking, ///< Active cooking cycle.
    MovingToRemovalHeight, ///< Normal stop or done move is in progress.
    Done, ///< Marshmallow is ready to remove.
    Fault ///< Faulted state requiring reset.
  };

  /**
   * @brief Construct the cooker task around existing child tasks.
   * @param task_ui User interface task reference.
   * @param task_temps Temperature task reference.
   * @param task_r_motor R motor task reference.
   * @param task_z_motor Z motor task reference.
   */
  TaskCooker(TaskUI& task_ui, TaskTemps& task_temps, TaskRMotor& task_r_motor, TaskZMotor& task_z_motor);

  /** @copydoc Task::run */
  void run() override;

  /** @copydoc Task::getStatus */
  Status getStatus() const override;

  /** @brief Get the current high-level cooker state. */
  State getState() const;

private:
  static constexpr int32_t kTargetFlameTempFx100 = 24000; ///< Target flame temperature, 215.00 F.
  static constexpr int32_t kDoneMarshmallowTempFx100 = 25000; ///< Done IR object temperature, 200.00 F.
  static constexpr uint32_t kDoneTempHoldTimeMs = 2000; ///< Required continuous time above done IR temperature.
  static constexpr uint32_t kStatusStreamPeriodMs = 500; ///< Status stream update period.

  State state_ = State::Uninitialized;

  TaskUI& task_ui_;
  TaskTemps& task_temps_;
  TaskRMotor& task_r_motor_;
  TaskZMotor& task_z_motor_;

  bool r_started_for_current_cook_ = false;
  bool manual_rotation_returning_ = false;
  bool done_temp_timer_active_ = false;
  uint32_t done_temp_start_ms_ = 0;

  bool status_stream_active_ = false;
  uint32_t status_stream_start_ms_ = 0;
  uint32_t status_stream_duration_ms_ = 0;
  uint32_t last_status_stream_ms_ = 0;

  /** @brief Handle one command consumed from the UI task. */
  void handleCommand(TaskUI::Command command);

  /** @brief Enter cooker fault state and emergency-stop child tasks. */
  void enterFault(const char* reason);

  /** @brief Print one status line over UART. */
  void printStatus() const;

  /** @brief Start periodic status streaming for a fixed duration. */
  void startStatusStream(uint32_t duration_ms);

  /** @brief Update the active status stream and stop it when complete. */
  void updateStatusStream();

  /** @brief Stop periodic status streaming. */
  void stopStatusStream();

  /** @brief Begin a normal stop sequence with R return and Z removal move. */
  void beginNormalStop(const char* message);

  /** @brief Clear the sustained IR done-temperature hold timer. */
  void resetDoneTempHoldTimer();
};

#endif /* TASK_COOKER_H */
