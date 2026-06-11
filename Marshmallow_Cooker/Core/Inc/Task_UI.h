#ifndef TASK_UI_H
#define TASK_UI_H

/**
 * @file Task_UI.h
 * @brief UART command-line user interface task.
 */

// Parent class include
#include "Task.h"

// User created includes
#include "main.h"

// Additional includes
#include <cstddef>
#include <cstdint>

// Externs
/** @brief UART handle used by the command-line interface. */
extern UART_HandleTypeDef huart2;

/**
 * @brief Parses UART commands and exposes them to the cooker task.
 *
 * The UI task receives bytes through HAL UART interrupts, buffers them in a
 * small queue, echoes printable characters, and converts completed command
 * lines into a single pending command for TaskCooker to consume.
 */
class TaskUI : public Task {
public:
  /** @brief Internal state of the UI command parser. */
  enum class State {
    Uninitialized, ///< UART receive has not been armed yet.
    Idle, ///< Waiting for a completed command line.
    CommandReady, ///< A parsed command is ready to be consumed.
    Fault ///< UI task has entered a fault state.
  };

  /** @brief Commands supported by the UART terminal. */
  enum class Command {
    None, ///< No command is pending.
    Home, ///< Home/setup the cooker.
    Start, ///< Begin a cooking cycle.
    Stop, ///< Perform a normal stop.
    EmergencyStop, ///< Immediately fault and stop all motion.
    Reset, ///< Reset recoverable task faults and require homing.
    Status, ///< Print one status line or start a status stream.
    PidDebugOn, ///< Enable Z PID debug printing.
    PidDebugOff, ///< Disable Z PID debug printing.
    ZJogDown, ///< Jog Z downward by a step amount.
    ZJogUp, ///< Jog Z upward by a step amount.
    Unknown ///< Command text did not match a supported command.
  };

  /** @brief Construct the UI task. */
  TaskUI();

  /** @copydoc Task::run */
  void run() override;

  /** @copydoc Task::getStatus */
  Status getStatus() const override;

  /**
   * @brief Get the UI parser state.
   * @return Current UI state.
   */
  State getState() const;

  /**
   * @brief Consume and clear the pending parsed command.
   * @return Pending command, or Command::None when no command is ready.
   */
  Command consumeCommand();

  /**
   * @brief Consume the parsed duration for a status stream command.
   * @return Status stream duration in milliseconds, or 0 for one-shot status.
   */
  uint32_t consumeStatusDurationMs();

  /**
   * @brief Consume the parsed Z jog distance.
   * @return Requested jog distance in steps.
   */
  uint32_t consumeJogSteps();

  /**
   * @brief HAL UART RX complete callback entry point.
   * @param huart UART handle that completed the receive.
   */
  void onUartReceiveComplete(UART_HandleTypeDef* huart);

  /**
   * @brief HAL UART error callback entry point.
   * @param huart UART handle that reported an error.
   */
  void onUartError(UART_HandleTypeDef* huart);

private:
  static constexpr size_t kCommandBufferSize = 32; ///< Maximum command line length, including null terminator.
  static constexpr size_t kRxQueueSize = 64; ///< ISR-to-task receive queue size.
  static constexpr uint32_t kDefaultJogSteps = 100U; ///< Default Z jog distance when no argument is supplied.

  State state_ = State::Uninitialized;
  Command pending_command_ = Command::None;
  uint32_t pending_status_duration_ms_ = 0;
  uint32_t pending_jog_steps_ = 0;

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

  /** @brief Arm the next interrupt-driven UART byte receive. */
  void armReceive();

  /** @brief Process all bytes currently queued by the UART ISR. */
  void processReceivedCharacters();

  /**
   * @brief Pop one received character from the ISR queue.
   * @param[out] c Character popped from the queue.
   * @return true if a character was available, false if the queue was empty.
   */
  bool popReceivedChar(char& c);

  /**
   * @brief Push a received byte into the queue from interrupt context.
   * @param c Received byte.
   */
  void pushReceivedCharFromIsr(uint8_t c);

  /**
   * @brief Handle one received character, including echo and line editing.
   * @param c Received character.
   */
  void handleReceivedChar(char c);

  /** @brief Parse and dispatch the current completed command line. */
  void handleCompletedLine();

  /** @brief Clear the command buffer and overflow flag. */
  void clearCommandBuffer();

  /**
   * @brief Echo one character to the terminal.
   * @param c Character to echo.
   */
  void echoChar(char c);

  /**
   * @brief Echo a null-terminated string to the terminal.
   * @param str String to transmit.
   */
  void echoString(const char* str);

  /**
   * @brief Parse a completed command line.
   * @param command Null-terminated command line.
   * @param[out] command_value Numeric argument associated with status or jog commands.
   * @return Parsed command code.
   */
  Command parseCommandLine(const char* command, uint32_t& command_value) const;

  /** @brief Convert an ASCII character to lowercase. */
  static char toLower(char c);

  /** @brief Compare two null-terminated strings for exact equality. */
  static bool stringsEqual(const char* a, const char* b);

  /** @brief Test for command-parser whitespace. */
  static bool isSpace(char c);

  /** @brief Test for an ASCII decimal digit. */
  static bool isDigit(char c);
};

#endif /* TASK_UI_H */
