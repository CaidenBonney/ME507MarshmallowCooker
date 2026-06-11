#ifndef TASK_UI_H
#define TASK_UI_H

#include "Task.h"

extern void print_str(const char* str);

class TaskUI : public Task {
public:
  enum class State {
    Uninitialized,
    Idle,
    Fault
  };

  TaskUI();

  void run() override;

  Status getStatus() const override;
  State getState() const;

private:
  State state_ = State::Uninitialized;
};

#endif /* TASK_UI_H */