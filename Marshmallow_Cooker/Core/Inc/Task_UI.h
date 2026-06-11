#ifndef TASK_UI_H
#define TASK_UI_H

// Parent class include
#include "Task.h"

// User created includes

// Additional includes

// Externs

class TaskUI : public Task {
public:
  enum class State {
    Uninitialized,
    Idle,
    Fault
  };

  enum class Command {
    None,
    Home,
    Start,
    Stop,
    EmergencyStop,
    Reset,
    Status
  };

  TaskUI();

  void run() override;
  Status getStatus() const override;

  State getState() const;

  Command consumeCommand();

  // Temporary helper for testing commands before UART parsing is wired in.
  void injectCommand(Command command);

private:
  State state_ = State::Uninitialized;
  Command pending_command_ = Command::None;
};

#endif /* TASK_UI_H */
