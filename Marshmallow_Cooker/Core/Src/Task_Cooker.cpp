#include "Task_Cooker.h"

#include <cstdio>

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

  if (task_temps_.hasValidThermocoupleReading()) {
    task_z_motor_.setMeasuredFlameTempFx100(task_temps_.getThermocoupleHotFx100());
  }

  updateStatusStream();

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
      if (task_r_motor_.isFaulted()) {
        enterFault("R motor fault during cooking");
        break;
      }

      if (task_z_motor_.bottomLimitPressed()) {
        beginNormalStop("Bottom limit reached. Performing normal stop.\r\n");
        break;
      }

      if (task_z_motor_.isFaulted()) {
        enterFault("Z motor fault during cooking");
        break;
      }

      if (!r_started_for_current_cook_ && task_z_motor_.getState() == TaskZMotor::State::ControllingFlameTemp) {
        task_r_motor_.startCookingRotation();
        r_started_for_current_cook_ = true;
        print_str("R cooking rotation started after Z reached cook position.\r\n");
      }

      if (task_temps_.hasValidIrReading() && task_temps_.getIrObjectFx100() >= kDoneMarshmallowTempFx100) {
        beginNormalStop("Marshmallow done. Moving to removal height and returning R to initial rotation.\r\n");
      }
      break;

    case State::MovingToRemovalHeight:
      if (task_r_motor_.isFaulted() || task_z_motor_.isFaulted()) {
        enterFault("motor fault while moving to removal height");
      } else if (!task_z_motor_.isBusy() && !task_r_motor_.isBusy()) {
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
        r_started_for_current_cook_ = false;
        task_z_motor_.startHoming();
        state_ = State::HomingZ;
      } else {
        print_str("Home command ignored in current state.\r\n");
      }
      break;

    case TaskUI::Command::Start:
      if (state_ == State::ReadyToCook && task_z_motor_.isHomed()) {
        r_started_for_current_cook_ = false;
        task_z_motor_.startTemperatureControl(kTargetFlameTempFx100);
        state_ = State::Cooking;
        print_str("Cooking started. Moving Z to initial cook position.\r\n");
      } else {
        print_str("Start rejected. Home Z first.\r\n");
      }
      break;

    case TaskUI::Command::Stop:
      if (state_ == State::Cooking || state_ == State::ReadyToCook || state_ == State::Done) {
        beginNormalStop("Normal stop. Moving to removal height and returning R to initial rotation.\r\n");
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
        stopStatusStream();
        r_started_for_current_cook_ = false;
        task_r_motor_.resetFault();
        task_z_motor_.resetFault();
        state_ = State::WaitingForHomeCommand;
      } else {
        print_str("Reset ignored. Reset is only accepted from Fault or Done.\r\n");
      }
      break;

    case TaskUI::Command::Status: {
      const uint32_t duration_ms = task_ui_.consumeStatusDurationMs();

      if (duration_ms > 0U) {
        startStatusStream(duration_ms);
      } else {
        printStatus();
      }
      break;
    }

    case TaskUI::Command::PidDebugOn:
      task_z_motor_.setPidDebugEnabled(true);
      break;

    case TaskUI::Command::PidDebugOff:
      task_z_motor_.setPidDebugEnabled(false);
      break;

    case TaskUI::Command::ZJogDown: {
      if (state_ == State::HomingZ || state_ == State::Cooking || state_ == State::MovingToRemovalHeight) {
        print_str("Z jog rejected: cooker is busy.\r\n");
        break;
      }

      const uint32_t jog_steps = task_ui_.consumeJogSteps();
      task_z_motor_.jogRelativeSteps(-static_cast<int32_t>(jog_steps));
      break;
    }

    case TaskUI::Command::ZJogUp: {
      if (state_ == State::HomingZ || state_ == State::Cooking || state_ == State::MovingToRemovalHeight) {
        print_str("Z jog rejected: cooker is busy.\r\n");
        break;
      }

      const uint32_t jog_steps = task_ui_.consumeJogSteps();
      task_z_motor_.jogRelativeSteps(static_cast<int32_t>(jog_steps));
      break;
    }

    case TaskUI::Command::Unknown:
      print_str("Unknown command. Use: home, start, stop, estop, reset, status, status <ms>, piddebug on, "
                "piddebug off, -, - <steps>, =, = <steps>\r\n");
      break;
  }
}

void TaskCooker::beginNormalStop(const char* message) {
  print_str(message);

  r_started_for_current_cook_ = false;
  task_r_motor_.returnToInitialRotation();
  task_z_motor_.stopMotion();
  task_z_motor_.moveToRemovalHeight();

  state_ = State::MovingToRemovalHeight;
}

void TaskCooker::enterFault(const char* reason) {
  if (state_ != State::Fault) {
    print_str("Cooker fault: ");
    print_str(reason);
    print_str("\r\n");
  }

  stopStatusStream();
  r_started_for_current_cook_ = false;
  task_r_motor_.emergencyStop();
  task_z_motor_.emergencyStop();
  state_ = State::Fault;
}

void TaskCooker::printStatus() const {
  const int32_t tc_hot = task_temps_.getThermocoupleHotFx100();
  const int32_t ir_object = task_temps_.getIrObjectFx100();

  std::snprintf(print_buf,
                100,
                "Status: cooker=%d z=%d r=%d zpos=%ld top=%d bottom=%d tc=%ld.%02ldF ir=%ld.%02ldF\r\n",
                static_cast<int>(state_),
                static_cast<int>(task_z_motor_.getState()),
                static_cast<int>(task_r_motor_.getState()),
                static_cast<long>(task_z_motor_.getPositionSteps()),
                static_cast<int>(task_z_motor_.topLimitPressed()),
                static_cast<int>(task_z_motor_.bottomLimitPressed()),
                static_cast<long>(tc_hot / 100),
                static_cast<long>(tc_hot >= 0 ? tc_hot % 100 : -(tc_hot % 100)),
                static_cast<long>(ir_object / 100),
                static_cast<long>(ir_object >= 0 ? ir_object % 100 : -(ir_object % 100)));
  print_str(print_buf);
}

void TaskCooker::startStatusStream(uint32_t duration_ms) {
  status_stream_active_ = true;
  status_stream_duration_ms_ = duration_ms;
  status_stream_start_ms_ = HAL_GetTick();
  last_status_stream_ms_ = status_stream_start_ms_;
  printStatus();
}

void TaskCooker::updateStatusStream() {
  if (!status_stream_active_) {
    return;
  }

  const uint32_t now_ms = HAL_GetTick();

  if ((now_ms - status_stream_start_ms_) >= status_stream_duration_ms_) {
    status_stream_active_ = false;
    return;
  }

  if (last_status_stream_ms_ == 0U || (now_ms - last_status_stream_ms_) >= kStatusStreamPeriodMs) {
    last_status_stream_ms_ = now_ms;
    printStatus();
  }
}

void TaskCooker::stopStatusStream() {
  status_stream_active_ = false;
  status_stream_duration_ms_ = 0;
  status_stream_start_ms_ = 0;
  last_status_stream_ms_ = 0;
}
