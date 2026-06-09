#include "z_motor_driver.h"

#include <climits>

ZMotorDriver::ZMotorDriver()
    : tmc_(Z_STEP_GPIO_Port,
           Z_STEP_Pin,
           Z_DIR_GPIO_Port,
           Z_DIR_Pin,
           Z_ENN_GPIO_Port,
           Z_ENN_Pin,
           Z_DIAG_GPIO_Port,
           Z_DIAG_Pin),
      position_steps_(0),
      target_steps_(0),
      speed_steps_per_second_(kDefaultSpeedStepsPerSecond),
      microsteps_(kDefaultMicrosteps),
      homing_down_(false),
      limits_active_low_(true),
      current_direction_(Direction::Up),
      state_(State::Disabled) {
}

void ZMotorDriver::begin() {
  tmc_.begin();
  tmc_.setStepRate(speed_steps_per_second_);
  zeroPosition();
  state_ = State::Disabled;
}

void ZMotorDriver::enable() {
  tmc_.enable();
  state_ = State::Idle;
}

void ZMotorDriver::disable() {
  tmc_.disable();
  homing_down_ = false;
  state_ = State::Disabled;
}

void ZMotorDriver::update() {
  if (!tmc_.isEnabled()) {
    state_ = State::Disabled;
    return;
  }

  // DIAG is currently informational only. It is logged by the TMC2209 driver,
  // but it does not stop Z motion until testing proves it is reliable.
  tmc_.updateDiagLog();

  if (homing_down_) {
    if (bottomLimitPressed()) {
      stop();
      zeroPosition();
      state_ = State::HitBottomLimit;
      return;
    }

    if (tryStep(Direction::Down)) {
      state_ = State::Moving;
    }

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

  if (tryStep(desired_direction)) {
    state_ = State::Moving;
  } else {
    state_ = State::Moving;
  }
}

void ZMotorDriver::moveSteps(int32_t steps) {
  if (steps == 0) {
    return;
  }

  homing_down_ = false;
  target_steps_ = position_steps_ + steps;

  if (!tmc_.isEnabled()) {
    enable();
  }

  state_ = State::Moving;
}

void ZMotorDriver::moveTo(int32_t target_position_steps) {
  homing_down_ = false;
  target_steps_ = target_position_steps;

  if (!tmc_.isEnabled()) {
    enable();
  }

  if (position_steps_ != target_steps_) {
    state_ = State::Moving;
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

  if (tmc_.isEnabled()) {
    state_ = State::Idle;
  }
}

void ZMotorDriver::homeDown(uint32_t speed_steps_per_second) {
  setSpeed(speed_steps_per_second);
  homing_down_ = true;
  target_steps_ = position_steps_;

  if (!tmc_.isEnabled()) {
    enable();
  }

  state_ = State::Moving;
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
  tmc_.setStepRate(speed_steps_per_second_);
}

void ZMotorDriver::setSpeedStepsPerSecond(uint32_t steps_per_second) {
  setSpeed(steps_per_second);
}

void ZMotorDriver::setMicrosteps(uint16_t microsteps) {
  if (microsteps == 0) {
    microsteps = 1;
  }

  // With the present PCB wiring, microstepping is controlled by MS1/MS2 pins.
  // This value is stored for calculations/reporting, but it cannot change the
  // TMC2209 setting.
  microsteps_ = microsteps;
}

void ZMotorDriver::setLimitsActiveLow(bool active_low) {
  limits_active_low_ = active_low;
}

void ZMotorDriver::setDirectionInverted(bool inverted) {
  tmc_.setDirectionInverted(inverted);
  setDirection(current_direction_);
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

bool ZMotorDriver::isBusy() const {
  return state_ == State::Moving;
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
  return tmc_.diagActive();
}

void ZMotorDriver::setDirection(Direction direction) {
  current_direction_ = direction;

  if (direction == Direction::Up) {
    tmc_.setDirection(TMC2209::Direction::Forward);
  } else {
    tmc_.setDirection(TMC2209::Direction::Reverse);
  }
}

bool ZMotorDriver::tryStep(Direction direction) {
  setDirection(direction);

  if (!tmc_.stepIfDue()) {
    return false;
  }

  if (direction == Direction::Up) {
    position_steps_++;
  } else {
    position_steps_--;
  }

  return true;
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