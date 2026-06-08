#include "z_motor_driver.h"

ZMotorDriver::ZMotorDriver()
    : position_steps_(0),
      target_steps_(0),
      speed_steps_per_second_(kDefaultSpeedStepsPerSecond),
      step_interval_us_(1000000UL / kDefaultSpeedStepsPerSecond),
      last_step_time_us_(0),
      microsteps_(kDefaultMicrosteps),
      enabled_(false),
      homing_down_(false),
      limits_active_low_(true),
      current_direction_(Direction::Up),
      state_(State::Disabled) {
}

void ZMotorDriver::begin() {
  configureGpioPins();

  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_RESET);

  disable();
  zeroPosition();
}

void ZMotorDriver::configureGpioPins() {
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  GPIO_InitStruct.Pin = Z_STEP_Pin | Z_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = Z_ENN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Z_ENN_GPIO_Port, &GPIO_InitStruct);
}

void ZMotorDriver::enable() {
  HAL_GPIO_WritePin(Z_ENN_GPIO_Port, Z_ENN_Pin, GPIO_PIN_RESET);
  enabled_ = true;
  state_ = State::Idle;
}

void ZMotorDriver::disable() {
  HAL_GPIO_WritePin(Z_ENN_GPIO_Port, Z_ENN_Pin, GPIO_PIN_SET);
  enabled_ = false;
  homing_down_ = false;
  state_ = State::Disabled;
}

void ZMotorDriver::update() {
  if (!enabled_) {
    state_ = State::Disabled;
    return;
  }

  if (driverFaultActive()) {
    stop();
    state_ = State::Fault;
    return;
  }

  if (homing_down_) {
    if (bottomLimitPressed()) {
      stop();
      zeroPosition();
      state_ = State::HitBottomLimit;
      return;
    }

    const uint32_t now_us = micros();
    if ((uint32_t)(now_us - last_step_time_us_) >= step_interval_us_) {
      last_step_time_us_ = now_us;
      setDirection(Direction::Down);
      stepOnce();
    }

    state_ = State::Moving;
    return;
  }

  if (position_steps_ == target_steps_) {
    state_ = State::Idle;
    return;
  }

  const Direction desired_direction = (target_steps_ > position_steps_) ? Direction::Up : Direction::Down;

  if (limitBlocksDirection(desired_direction)) {
    state_ = (desired_direction == Direction::Up) ? State::HitTopLimit : State::HitBottomLimit;
    target_steps_ = position_steps_;
    return;
  }

  const uint32_t now_us = micros();
  if ((uint32_t)(now_us - last_step_time_us_) < step_interval_us_) {
    state_ = State::Moving;
    return;
  }

  last_step_time_us_ = now_us;
  setDirection(desired_direction);
  stepOnce();

  state_ = State::Moving;
}

void ZMotorDriver::moveSteps(int32_t steps) {
  if (steps == 0) {
    return;
  }

  homing_down_ = false;
  target_steps_ = position_steps_ + steps;

  if (!enabled_) {
    enable();
  }
}

void ZMotorDriver::moveTo(int32_t target_position_steps) {
  homing_down_ = false;
  target_steps_ = target_position_steps;

  if (!enabled_) {
    enable();
  }
}

void ZMotorDriver::jog(Direction direction, uint32_t speed_steps_per_second) {
  setSpeed(speed_steps_per_second);

  if (direction == Direction::Up) {
    moveTo(INT32_MAX);
  } else {
    moveTo(INT32_MIN);
  }
}

void ZMotorDriver::stop() {
  target_steps_ = position_steps_;
  homing_down_ = false;

  if (enabled_) {
    state_ = State::Idle;
  }
}

void ZMotorDriver::homeDown(uint32_t speed_steps_per_second) {
  setSpeed(speed_steps_per_second);
  homing_down_ = true;
  target_steps_ = position_steps_;

  if (!enabled_) {
    enable();
  }
}

void ZMotorDriver::zeroPosition() {
  position_steps_ = 0;
  target_steps_ = 0;
}

void ZMotorDriver::setSpeed(uint32_t steps_per_second) {
  if (steps_per_second == 0) {
    steps_per_second = 1;
  }

  speed_steps_per_second_ = steps_per_second;
  step_interval_us_ = 1000000UL / speed_steps_per_second_;

  if (step_interval_us_ < 2 * kMinStepPulseUs) {
    step_interval_us_ = 2 * kMinStepPulseUs;
  }
}

void ZMotorDriver::setMicrosteps(uint16_t microsteps) {
  if (microsteps == 0) {
    microsteps = 1;
  }

  microsteps_ = microsteps;
}

void ZMotorDriver::setLimitsActiveLow(bool active_low) {
  limits_active_low_ = active_low;
}

int32_t ZMotorDriver::getPositionSteps() const {
  return position_steps_;
}

int32_t ZMotorDriver::getTargetSteps() const {
  return target_steps_;
}

ZMotorDriver::State ZMotorDriver::getState() const {
  return state_;
}

bool ZMotorDriver::topLimitPressed() const {
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(Z_TOP_GPIO_Port, Z_TOP_Pin);
  return limits_active_low_ ? (pin_state == GPIO_PIN_RESET) : (pin_state == GPIO_PIN_SET);
}

bool ZMotorDriver::bottomLimitPressed() const {
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(Z_BOT_GPIO_Port, Z_BOT_Pin);
  return limits_active_low_ ? (pin_state == GPIO_PIN_RESET) : (pin_state == GPIO_PIN_SET);
}

bool ZMotorDriver::driverFaultActive() const {
  return HAL_GPIO_ReadPin(Z_DIAG_GPIO_Port, Z_DIAG_Pin) == GPIO_PIN_SET;
}

void ZMotorDriver::setDirection(Direction direction) {
  current_direction_ = direction;

  if (direction == Direction::Up) {
    HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_SET);
  } else {
    HAL_GPIO_WritePin(Z_DIR_GPIO_Port, Z_DIR_Pin, GPIO_PIN_RESET);
  }
}

void ZMotorDriver::stepOnce() {
  HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_SET);

  const uint32_t start_us = micros();
  while ((uint32_t)(micros() - start_us) < kMinStepPulseUs) {
  }

  HAL_GPIO_WritePin(Z_STEP_GPIO_Port, Z_STEP_Pin, GPIO_PIN_RESET);

  if (current_direction_ == Direction::Up) {
    position_steps_++;
  } else {
    position_steps_--;
  }
}

bool ZMotorDriver::limitBlocksDirection(Direction direction) const {
  if (direction == Direction::Up && topLimitPressed()) {
    return true;
  }

  if (direction == Direction::Down && bottomLimitPressed()) {
    return true;
  }

  return false;
}

uint32_t ZMotorDriver::micros() const {
  return DWT->CYCCNT / (HAL_RCC_GetHCLKFreq() / 1000000UL);
}