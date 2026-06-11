#ifndef TASK_COOKER_H
#define TASK_COOKER_H

// Parent class include
#include "Task.h"

// User created includes
#include "Task_R_Motor.h"
#include "Task_Temps.h"
#include "Task_UI.h"
#include "Task_Z_Motor.h"

// Additional includes
#include <cstdint>
#include <cstdlib>

// Externs

class TaskCooker : public Task {
public:
  enum class State {
    Uninitialized,
    WaitingForHomeCommand,
    HomingZ,
    ReadyToCook,
    Cooking,
    MovingToRemovalHeight,
    Done,
    Fault
  };

  TaskCooker(TaskUI& task_ui, TaskTemps& task_temps, TaskRMotor& task_r_motor, TaskZMotor& task_z_motor);

  void run() override;
  Status getStatus() const override;

  State getState() const;

private:
  static constexpr int16_t kTargetFlameTempFx100 = 35000; // TODO: tune. 350.00 F.
  static constexpr int16_t kDoneMarshmallowTempFx100 = 16000; // TODO: tune. 160.00 F.
  static constexpr uint32_t kStatusStreamPeriodMs = 500;

  State state_ = State::Uninitialized;

  TaskUI& task_ui_;
  TaskTemps& task_temps_;
  TaskRMotor& task_r_motor_;
  TaskZMotor& task_z_motor_;

  bool status_stream_active_ = false;
  uint32_t status_stream_start_ms_ = 0;
  uint32_t status_stream_duration_ms_ = 0;
  uint32_t last_status_stream_ms_ = 0;

  void handleCommand(TaskUI::Command command);
  void enterFault(const char* reason);
  void printStatus() const;
  void startStatusStream(uint32_t duration_ms);
  void updateStatusStream();
  void stopStatusStream();
};

#endif /* TASK_COOKER_H */
