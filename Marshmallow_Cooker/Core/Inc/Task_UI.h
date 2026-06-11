#ifndef TASK_UI_H
#define TASK_UI_H

#include "Task.h"

extern void print_str(const char* str);

class TaskUI : public Task {
public:
  TaskUI();

  void run();

private:
  // TODO: add enum for init then run state
};

#endif /* TASK_UI_H */