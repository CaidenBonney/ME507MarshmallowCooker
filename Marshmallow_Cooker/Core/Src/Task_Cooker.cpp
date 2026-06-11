#include "Task_Cooker.h"

TaskCooker::TaskCooker(TaskUI& task_ui, TaskTemps& task_temps, TaskRMotor& task_r_motor, TaskZMotor& task_z_motor)
    : task_ui_(task_ui),
      task_temps_(task_temps),
      task_r_motor_(task_r_motor),
      task_z_motor_(task_z_motor) {
}

void TaskCooker::run() {
  if (state_ == State::Uninitialized) {
    print_str("Cooker initialized. Send 'home' to begin setup.\r\n");
    state_ = State::WaitingForHomeCommand;
  }

  const TaskUI::Command command = task_ui_.consumeCommand();
  handleCommand(command);
  updateStatusStream();

  if (task_temps_.hasValidThermocoupleReading()) {
    task_z_motor_.setMeasuredFlameTempFx100(task_temps_.getThermocoupleHotFx100());
  }

  switch (state_) {
    case State::Uninitialized:
      break;

    case State::WaitingForHomeCommand:
      break;

    case State::HomingZ:
      if (task_z_motor_.isFaulted()) {
        enterFault("Z fault during homing");
      } else if (task_z_motor_.isHomed() && !task_z_motor_.isBusy()) {
        print_str("Setup complete. Send 'start' to cook.\r\n");
        state_ = State::ReadyToCook;
      }
      break;

    case State::ReadyToCook:
      break;

    case State::Cooking:
      if (task_r_motor_.isFaulted() || task_z_motor_.isFaulted()) {
        enterFault("motor fault during cooking");
        break;
      }

      if (task_temps_.hasValidIrReading() && task_temps_.getIrObjectFx100() >= kDoneMarshmallowTempFx100) {
        print_str("Marshmallow done. Moving to removal height.\r\n");
        task_r_motor_.stopCookingRotation();
        task_z_motor_.stopMotion();
        task_z_motor_.moveToRemovalHeight();
        state_ = State::MovingToRemovalHeight;
      }
      break;

    case State::MovingToRemovalHeight:
      if (task_z_motor_.isFaulted()) {
        enterFault("Z fault moving to removal height");
      } else if (!task_z_motor_.isBusy()) {
        print_str("Remove marshmallow. Send 'reset' before next cook.\r\n");
        state_ = State::Done;
      }
      break;

    case State::Done:
      break;

    case State::Fault:
      break;
  }
}

Task::Status TaskCooker::getStatus() const {
  if (state_ == State::Uninitialized) {
    return Task::Status::Uninitialized;
  }

  if (state_ == State::Fault) {
    return Task::Status::Fault;
  }

  return Task::Status::Running;
}

TaskCooker::State TaskCooker::getState() const {
  return state_;
}

void TaskCooker::handleCommand(TaskUI::Command command) {
  switch (command) {
    case TaskUI::Command::None:
      break;

    case TaskUI::Command::Home:
      if (state_ == State::WaitingForHomeCommand || state_ == State::ReadyToCook || state_ == State::Done) {
        print_str("Starting Z home.\r\n");
        task_z_motor_.startHoming();
        state_ = State::HomingZ;
      } else {
        print_str("Home command ignored in current state.\r\n");
      }
      break;

    case TaskUI::Command::Start:
      if (state_ == State::ReadyToCook && task_z_motor_.isHomed()) {
        print_str("Cooking started.\r\n");
        task_r_motor_.startCookingRotation();
        task_z_motor_.startTemperatureControl(kTargetFlameTempFx100);
        state_ = State::Cooking;
      } else {
        print_str("Start rejected. Home Z first.\r\n");
      }
      break;

    case TaskUI::Command::Stop:
      if (state_ == State::Cooking || state_ == State::ReadyToCook || state_ == State::Done) {
        print_str("Normal stop. Moving to removal height.\r\n");
        task_r_motor_.stopCookingRotation();
        task_z_motor_.stopMotion();
        task_z_motor_.moveToRemovalHeight();
        state_ = State::MovingToRemovalHeight;
      } else {
        print_str("Stop command ignored in current state.\r\n");
      }
      break;

    case TaskUI::Command::EmergencyStop:
      enterFault("emergency stop");
      break;

    case TaskUI::Command::Reset:
      if (state_ == State::Fault || state_ == State::Done) {
        print_str("Software reset. Send 'home' before cooking.\r\n");
        task_r_motor_.resetFault();
        task_z_motor_.resetFault();
        stopStatusStream();
        state_ = State::WaitingForHomeCommand;
      } else {
        print_str("Reset ignored. Reset is only accepted from Fault or Done.\r\n");
      }
      break;

    case TaskUI::Command::Status: {
      const uint32_t status_duration_ms = task_ui_.consumeStatusDurationMs();

      if (status_duration_ms > 0U) {
        startStatusStream(status_duration_ms);
      } else {
        printStatus();
      }
      break;
    }

    case TaskUI::Command::Unknown:
      print_str("Unknown command. Use: home, start, stop, estop, reset, status\r\n");
      break;
  }
}

void TaskCooker::enterFault(const char* reason) {
  if (state_ != State::Fault) {
    print_str("Cooker fault: ");
    print_str(reason);
    print_str("\r\n");
  }

  task_r_motor_.emergencyStop();
  task_z_motor_.emergencyStop();
  state_ = State::Fault;
}

void TaskCooker::printStatus() const {
  sprintf(print_buf,
          "Status: cooker=%d z=%d r=%d zpos=%ld tc=%d.%02dF ir=%d.%02dF\r\n",
          static_cast<int>(state_),
          static_cast<int>(task_z_motor_.getState()),
          static_cast<int>(task_r_motor_.getState()),
          static_cast<long>(task_z_motor_.getPositionSteps()),
          task_temps_.getThermocoupleHotFx100() / 100,
          abs(task_temps_.getThermocoupleHotFx100() % 100),
          task_temps_.getIrObjectFx100() / 100,
          abs(task_temps_.getIrObjectFx100() % 100));
  print_str(print_buf);
}

void TaskCooker::startStatusStream(uint32_t duration_ms) {
  status_stream_active_ = true;
  status_stream_start_ms_ = HAL_GetTick();
  status_stream_duration_ms_ = duration_ms;
  last_status_stream_ms_ = status_stream_start_ms_ - kStatusStreamPeriodMs;

  sprintf(print_buf, "Streaming status for %lu ms.\r\n", static_cast<unsigned long>(duration_ms));
  print_str(print_buf);
}

void TaskCooker::updateStatusStream() {
  if (!status_stream_active_) {
    return;
  }

  const uint32_t now_ms = HAL_GetTick();

  if ((now_ms - status_stream_start_ms_) >= status_stream_duration_ms_) {
    status_stream_active_ = false;
    print_str("Status stream complete.\r\n");
    return;
  }

  if ((now_ms - last_status_stream_ms_) >= kStatusStreamPeriodMs) {
    last_status_stream_ms_ = now_ms;
    printStatus();
  }
}

void TaskCooker::stopStatusStream() {
  status_stream_active_ = false;
}
