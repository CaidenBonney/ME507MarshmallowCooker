/**
 * @file Task_UI.cpp
 * @brief Implementation of the UART command-line user interface task.
 */

#include "Task_UI.h"
#include <cstring>

extern TaskUI task_ui;

TaskUI::TaskUI() {
}

void TaskUI::run() {
  if (state_ == State::Uninitialized) {
    print_str("UI initialized. Commands: home, start, stop, estop, reset, status, status <ms>, piddebug on, "
              "piddebug off, -, - <steps>, =, = <steps>\r\n");
    state_ = State::Idle;
    armReceive();
  }

  if (!rx_armed_) {
    armReceive();
  }

  if (rx_queue_overflowed_) {
    rx_queue_overflowed_ = false;
    print_str("\r\nUART receive queue overflow. Command line cleared.\r\n> ");
    clearCommandBuffer();
  }

  processReceivedCharacters();
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
  Command command = pending_command_;
  pending_command_ = Command::None;

  if (command != Command::Status) {
    pending_status_duration_ms_ = 0;
  }

  if (command != Command::ZJogDown && command != Command::ZJogUp) {
    pending_jog_steps_ = 0;
  }

  if (state_ == State::CommandReady) {
    state_ = State::Idle;
  }

  return command;
}

uint32_t TaskUI::consumeStatusDurationMs() {
  const uint32_t duration_ms = pending_status_duration_ms_;
  pending_status_duration_ms_ = 0;
  return duration_ms;
}

uint32_t TaskUI::consumeJogSteps() {
  const uint32_t jog_steps = pending_jog_steps_;
  pending_jog_steps_ = 0;
  return jog_steps;
}

void TaskUI::onUartReceiveComplete(UART_HandleTypeDef* huart) {
  if (huart != &huart2) {
    return;
  }

  rx_armed_ = false;
  pushReceivedCharFromIsr(rx_byte_);
  armReceive();
}

void TaskUI::armReceive() {
  if (rx_armed_) {
    return;
  }

  HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);

  if (status == HAL_OK) {
    rx_armed_ = true;
  } else if (status == HAL_BUSY) {
    print_str("UART RX arm failed: BUSY\r\n");
  } else {
    print_str("UART RX arm failed: ERROR\r\n");
  }
}

void TaskUI::processReceivedCharacters() {
  char c = '\0';

  while (popReceivedChar(c)) {
    handleReceivedChar(c);
  }
}

bool TaskUI::popReceivedChar(char& c) {
  if (rx_queue_tail_ == rx_queue_head_) {
    return false;
  }

  c = static_cast<char>(rx_queue_[rx_queue_tail_]);
  rx_queue_tail_ = (rx_queue_tail_ + 1U) % kRxQueueSize;
  return true;
}

void TaskUI::pushReceivedCharFromIsr(uint8_t c) {
  const size_t next_head = (rx_queue_head_ + 1U) % kRxQueueSize;

  if (next_head == rx_queue_tail_) {
    rx_queue_overflowed_ = true;
    return;
  }

  rx_queue_[rx_queue_head_] = c;
  rx_queue_head_ = next_head;
}

void TaskUI::handleReceivedChar(char c) {
  if (c == '\n' && last_was_cr_) {
    last_was_cr_ = false;
    return;
  }

  if (c == '\r' || c == '\n') {
    last_was_cr_ = (c == '\r');
    echoString("\r\n");
    handleCompletedLine();
    echoString("> ");
    return;
  }

  last_was_cr_ = false;

  if (c == '\b' || c == 0x7F) {
    if (command_length_ > 0) {
      command_length_--;
      command_buffer_[command_length_] = '\0';
      echoString("\b \b");
    }
    return;
  }

  if (c < 32 || c > 126) {
    return;
  }

  if (command_length_ < (kCommandBufferSize - 1U)) {
    command_buffer_[command_length_] = c;
    command_length_++;
    command_buffer_[command_length_] = '\0';
    echoChar(c);
  } else {
    overflowed_ = true;
  }
}

void TaskUI::handleCompletedLine() {
  if (overflowed_) {
    print_str("Command too long. Try again.\r\n");
    clearCommandBuffer();
    return;
  }

  if (command_length_ == 0) {
    clearCommandBuffer();
    return;
  }

  uint32_t command_value = 0;
  Command parsed_command = parseCommandLine(command_buffer_, command_value);

  if (parsed_command == Command::Unknown) {
    print_str("Unknown command. Use: home, start, stop, estop, reset, status, status <ms>, piddebug on, piddebug off, "
              "-, - <steps>, =, = <steps>\r\n");
  } else if (parsed_command != Command::None) {
    pending_command_ = parsed_command;

    if (parsed_command == Command::Status) {
      pending_status_duration_ms_ = command_value;
    } else if (parsed_command == Command::ZJogDown || parsed_command == Command::ZJogUp) {
      pending_jog_steps_ = command_value;
    }

    state_ = State::CommandReady;
  }

  clearCommandBuffer();
}

void TaskUI::clearCommandBuffer() {
  command_length_ = 0;
  overflowed_ = false;
  command_buffer_[0] = '\0';
}

void TaskUI::echoChar(char c) {
  char out[2] = {c, '\0'};
  echoString(out);
}

void TaskUI::echoString(const char* str) {
  HAL_UART_Transmit(&huart2,
                    reinterpret_cast<uint8_t*>(const_cast<char*>(str)),
                    static_cast<uint16_t>(std::strlen(str)),
                    100);
}

TaskUI::Command TaskUI::parseCommandLine(const char* command, uint32_t& command_value) const {
  command_value = 0;

  char normalized[kCommandBufferSize] = {};
  size_t write_index = 0;

  while (isSpace(*command)) {
    command++;
  }

  while (*command != '\0' && !isSpace(*command) && write_index < (kCommandBufferSize - 1U)) {
    normalized[write_index] = toLower(*command);
    write_index++;
    command++;
  }

  normalized[write_index] = '\0';

  while (isSpace(*command)) {
    command++;
  }

  const char* argument = command;

  if (stringsEqual(normalized, "home")) {
    return (*argument == '\0') ? Command::Home : Command::Unknown;
  }

  if (stringsEqual(normalized, "start")) {
    return (*argument == '\0') ? Command::Start : Command::Unknown;
  }

  if (stringsEqual(normalized, "stop")) {
    return (*argument == '\0') ? Command::Stop : Command::Unknown;
  }

  if (stringsEqual(normalized, "estop") || stringsEqual(normalized, "e-stop") ||
      stringsEqual(normalized, "emergency") || stringsEqual(normalized, "emergencystop")) {
    return (*argument == '\0') ? Command::EmergencyStop : Command::Unknown;
  }

  if (stringsEqual(normalized, "reset")) {
    return (*argument == '\0') ? Command::Reset : Command::Unknown;
  }

  if (stringsEqual(normalized, "status")) {
    if (*argument == '\0') {
      command_value = 0;
      return Command::Status;
    }

    uint32_t value_ms = 0;
    bool has_digit = false;

    while (isDigit(*argument)) {
      has_digit = true;
      const uint32_t digit = static_cast<uint32_t>(*argument - '0');

      if (value_ms <= ((UINT32_MAX - digit) / 10U)) {
        value_ms = (value_ms * 10U) + digit;
      } else {
        return Command::Unknown;
      }

      argument++;
    }

    while (isSpace(*argument)) {
      argument++;
    }

    if (!has_digit || *argument != '\0') {
      return Command::Unknown;
    }

    command_value = value_ms;
    return Command::Status;
  }

  if (stringsEqual(normalized, "piddebug")) {
    char debug_argument[kCommandBufferSize] = {};
    size_t debug_write_index = 0;

    while (*argument != '\0' && !isSpace(*argument) && debug_write_index < (kCommandBufferSize - 1U)) {
      debug_argument[debug_write_index] = toLower(*argument);
      debug_write_index++;
      argument++;
    }

    debug_argument[debug_write_index] = '\0';

    while (isSpace(*argument)) {
      argument++;
    }

    if (*argument != '\0') {
      return Command::Unknown;
    }

    if (stringsEqual(debug_argument, "on")) {
      return Command::PidDebugOn;
    }

    if (stringsEqual(debug_argument, "off")) {
      return Command::PidDebugOff;
    }

    return Command::Unknown;
  }

  if (stringsEqual(normalized, "-")) {
    if (*argument == '\0') {
      command_value = kDefaultJogSteps;
      return Command::ZJogDown;
    }

    uint32_t jog_steps = 0;
    bool has_digit = false;

    while (isDigit(*argument)) {
      has_digit = true;
      const uint32_t digit = static_cast<uint32_t>(*argument - '0');

      if (jog_steps <= ((UINT32_MAX - digit) / 10U)) {
        jog_steps = (jog_steps * 10U) + digit;
      } else {
        return Command::Unknown;
      }

      argument++;
    }

    while (isSpace(*argument)) {
      argument++;
    }

    if (!has_digit || *argument != '\0' || jog_steps == 0U) {
      return Command::Unknown;
    }

    command_value = jog_steps;
    return Command::ZJogDown;
  }

  if (stringsEqual(normalized, "=")) {
    if (*argument == '\0') {
      command_value = kDefaultJogSteps;
      return Command::ZJogUp;
    }

    uint32_t jog_steps = 0;
    bool has_digit = false;

    while (isDigit(*argument)) {
      has_digit = true;
      const uint32_t digit = static_cast<uint32_t>(*argument - '0');

      if (jog_steps <= ((UINT32_MAX - digit) / 10U)) {
        jog_steps = (jog_steps * 10U) + digit;
      } else {
        return Command::Unknown;
      }

      argument++;
    }

    while (isSpace(*argument)) {
      argument++;
    }

    if (!has_digit || *argument != '\0' || jog_steps == 0U) {
      return Command::Unknown;
    }

    command_value = jog_steps;
    return Command::ZJogUp;
  }

  return Command::Unknown;
}

char TaskUI::toLower(char c) {
  if (c >= 'A' && c <= 'Z') {
    return static_cast<char>(c - 'A' + 'a');
  }

  return c;
}

bool TaskUI::stringsEqual(const char* a, const char* b) {
  while (*a != '\0' && *b != '\0') {
    if (*a != *b) {
      return false;
    }

    a++;
    b++;
  }

  return *a == '\0' && *b == '\0';
}

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  task_ui.onUartReceiveComplete(huart);
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  task_ui.onUartError(huart);
}

bool TaskUI::isSpace(char c) {
  return c == ' ' || c == '\t';
}

bool TaskUI::isDigit(char c) {
  return c >= '0' && c <= '9';
}

void TaskUI::onUartError(UART_HandleTypeDef* huart) {
  if (huart != &huart2) {
    return;
  }

  rx_armed_ = false;
  armReceive();
}
