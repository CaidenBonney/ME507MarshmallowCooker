#ifndef TASK_UI_H
#define TASK_UI_H

#include "Task.h"
#include "main.h"
#include <cstddef>
#include <cstdint>

extern UART_HandleTypeDef huart2;
extern void print_str(const char* str);
extern char print_buf[100];

class TaskUI : public Task {
public:
  enum class State {
    Uninitialized,
    Idle,
    CommandReady,
    Fault
  };

  enum class Command {
    None,
    Home,
    Start,
    Stop,
    EmergencyStop,
    Reset,
    Status,
    Unknown
  };

  TaskUI();

  void run() override;
  Status getStatus() const override;

  State getState() const;
  Command consumeCommand();
  void onUartReceiveComplete(UART_HandleTypeDef* huart);

private:
  static constexpr size_t kCommandBufferSize = 32;

  State state_ = State::Uninitialized;
  Command pending_command_ = Command::None;

  char command_buffer_[kCommandBufferSize] = {};
  size_t command_length_ = 0;

  uint8_t rx_byte_ = 0;
  bool rx_armed_ = false;
  bool overflowed_ = false;

  void armReceive();
  void handleReceivedChar(char c);
  void handleCompletedLine();
  void clearCommandBuffer();
  void echoChar(char c);
  void echoString(const char* str);
  Command parseCommand(const char* command) const;
  static char toLower(char c);
  static bool stringsEqual(const char* a, const char* b);
};

#endif /* TASK_UI_H */
