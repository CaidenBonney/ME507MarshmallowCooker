#include "Task_UI.h"
#include <cstring>

extern TaskUI task_ui;

TaskUI::TaskUI() {
}

void TaskUI::run() {
  if (state_ == State::Uninitialized) {
    print_str("UI initialized. Commands: home, start, stop, estop, reset, status\r\n> ");
    state_ = State::Idle;
    armReceive();
  }

  if (!rx_armed_) {
    armReceive();
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
  Command command = pending_command_;
  pending_command_ = Command::None;

  if (state_ == State::CommandReady) {
    state_ = State::Idle;
  }

  return command;
}

void TaskUI::onUartReceiveComplete(UART_HandleTypeDef* huart) {
  if (huart != &huart2) {
    return;
  }

  rx_armed_ = false;
  handleReceivedChar(static_cast<char>(rx_byte_));
  armReceive();
}

void TaskUI::armReceive() {
  if (rx_armed_) {
    return;
  }

  if (HAL_UART_Receive_IT(&huart2, &rx_byte_, 1) == HAL_OK) {
    rx_armed_ = true;
  } else {
    state_ = State::Fault;
  }
}

void TaskUI::handleReceivedChar(char c) {
  if (c == '\r' || c == '\n') {
    echoString("\r\n");
    handleCompletedLine();
    echoString("> ");
    return;
  }

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

  if (command_length_ < (kCommandBufferSize - 1)) {
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

  Command parsed_command = parseCommand(command_buffer_);

  if (parsed_command == Command::None) {
    print_str("Unknown command. Use: home, start, stop, estop, reset, status\r\n");
  } else {
    pending_command_ = parsed_command;
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

TaskUI::Command TaskUI::parseCommand(const char* command) const {
  char normalized[kCommandBufferSize] = {};
  size_t write_index = 0;

  while (*command == ' ' || *command == '\t') {
    command++;
  }

  while (*command != '\0' && write_index < (kCommandBufferSize - 1)) {
    if (*command == ' ' || *command == '\t') {
      break;
    }

    normalized[write_index] = toLower(*command);
    write_index++;
    command++;
  }

  normalized[write_index] = '\0';

  if (stringsEqual(normalized, "home")) {
    return Command::Home;
  }

  if (stringsEqual(normalized, "start")) {
    return Command::Start;
  }

  if (stringsEqual(normalized, "stop")) {
    return Command::Stop;
  }

  if (stringsEqual(normalized, "estop") || stringsEqual(normalized, "e-stop") ||
      stringsEqual(normalized, "emergency") || stringsEqual(normalized, "emergencystop")) {
    return Command::EmergencyStop;
  }

  if (stringsEqual(normalized, "reset")) {
    return Command::Reset;
  }

  if (stringsEqual(normalized, "status")) {
    return Command::Status;
  }

  return Command::None;
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
