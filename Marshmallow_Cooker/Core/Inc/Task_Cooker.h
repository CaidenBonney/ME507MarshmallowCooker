#ifndef TASK_COOKER_H
#define TASK_COOKER_H

#include "Task.h"
#include "Task_R_Motor.h"
#include "Task_Temps.h"
#include "Task_UI.h"
#include "Task_Z_Motor.h"
#include "stm32f4xx_hal.h"
#include <cstdlib>

extern void print_str(const char* str);
extern char print_buf[100];

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

  State state_ = State::Uninitialized;

  TaskUI& task_ui_;
  TaskTemps& task_temps_;
  TaskRMotor& task_r_motor_;
  TaskZMotor& task_z_motor_;

  void handleCommand(TaskUI::Command command);
  void enterFault(const char* reason);
  void printStatus() const;
};

#endif /* TASK_COOKER_H */
