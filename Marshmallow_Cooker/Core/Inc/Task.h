#ifndef TASK_H
#define TASK_H

class Task {
public:
  enum class Status {
    Uninitialized,
    Running,
    Fault
  };

  virtual void run() = 0;
  virtual Status getStatus() const = 0;

  virtual ~Task() = default;
};

#endif /* TASK_H */