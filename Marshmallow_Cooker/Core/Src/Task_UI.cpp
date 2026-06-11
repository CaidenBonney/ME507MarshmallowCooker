#include "Task_UI.h"

TaskUI::TaskUI() {
}

void TaskUI::run() {
  switch (state_) {
    case State::Uninitialized:
      print_str("UI initialized\r\n");
      state_ = State::Idle;
      break;

    case State::Idle:
      // TODO: Add UART command parsing here.
      // Expected commands: home, start, stop, estop, reset, status.
      break;

    case State::Fault:
      break;
  }
}

Task::Status TaskUI::getStatus() const {
  if (state_ == State::Uninitialized) {
    return Task::Status::Uninitialized;
  }

  if (state_ == State::Fault) {
    return Task::Status::Fault;
  }

  return Task::Status::Running;
}

TaskUI::State TaskUI::getState() const {
  return state_;
}

TaskUI::Command TaskUI::consumeCommand() {
  const Command command = pending_command_;
  pending_command_ = Command::None;
  return command;
}

void TaskUI::injectCommand(Command command) {
  pending_command_ = command;
}
