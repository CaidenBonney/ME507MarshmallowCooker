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
      /*
       * Later, read buttons or user input here.
       */
      break;

    case State::Fault:
      break;
  }
}

TaskUI::State TaskUI::getState() const {
  return state_;
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