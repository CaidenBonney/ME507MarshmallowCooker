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
  uint32_t consumeStatusDurationMs();

  void onUartReceiveComplete(UART_HandleTypeDef* huart);
  void onUartError(UART_HandleTypeDef* huart);

private:
  static constexpr size_t kCommandBufferSize = 32;
  static constexpr size_t kRxQueueSize = 64;

  State state_ = State::Uninitialized;
  Command pending_command_ = Command::None;
  uint32_t pending_status_duration_ms_ = 0;

  char command_buffer_[kCommandBufferSize] = {};
  size_t command_length_ = 0;

  uint8_t rx_byte_ = 0;
  volatile bool rx_armed_ = false;
  bool overflowed_ = false;
  bool last_was_cr_ = false;

  uint8_t rx_queue_[kRxQueueSize] = {};
  volatile size_t rx_queue_head_ = 0;
  volatile size_t rx_queue_tail_ = 0;
  volatile bool rx_queue_overflowed_ = false;

  void armReceive();
  void processReceivedCharacters();
  bool popReceivedChar(char& c);
  void pushReceivedCharFromIsr(uint8_t c);

  void handleReceivedChar(char c);
  void handleCompletedLine();
  void clearCommandBuffer();
  void echoChar(char c);
  void echoString(const char* str);
  Command parseCommandLine(const char* command, uint32_t& status_duration_ms) const;
  static char toLower(char c);
  static bool stringsEqual(const char* a, const char* b);
  static bool isSpace(char c);
  static bool isDigit(char c);
};

#endif /* TASK_UI_H */
