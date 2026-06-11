/**
 * @file z_motor_driver.cpp
 * @brief Implementation of the non-blocking Z-axis stepper driver.
 */

#include "z_motor_driver.h"

#include "main.h"
#include <climits>

/** @copydoc ZMotorDriver::ZMotorDriver */
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
      homing_(false),
      homing_direction_(Direction::Up),
      limits_active_low_(true),
      current_direction_(Direction::Up),
      state_(State::Disabled) {
}

/** @copydoc ZMotorDriver::begin */
void ZMotorDriver::begin() {
  tmc_.begin();
  tmc_.setStepRate(speed_steps_per_second_);
  zeroPosition();
  state_ = State::Disabled;
}

/** @copydoc ZMotorDriver::enable */
void ZMotorDriver::enable() {
  tmc_.enable();
  state_ = State::Idle;
}

/** @copydoc ZMotorDriver::disable */
void ZMotorDriver::disable() {
  tmc_.disable();
  homing_ = false;
  state_ = State::Disabled;
}

/** @copydoc ZMotorDriver::update */
void ZMotorDriver::update() {
  if (!tmc_.isEnabled()) {
    state_ = State::Disabled;
    return;
  }

  // DIAG is currently informational only. It is logged by the TMC2209 driver,
  // but it does not stop Z motion until testing proves it is reliable.
  tmc_.updateDiagLog();

  // Hard software limit protection. If either limit switch is pressed while
  // the current motion direction would push farther into that switch, stop
  // before any more steps are issued.
  if (motionCommandActive() && topLimitPressed() && current_direction_ == Direction::Up) {
    stop();

    if (homing_direction_ == Direction::Up) {
      zeroPosition();
    }

    state_ = State::HitTopLimit;
    return;
  }

  if (motionCommandActive() && bottomLimitPressed() && current_direction_ == Direction::Down) {
    stop();

    if (homing_direction_ == Direction::Down) {
      zeroPosition();
    }

    state_ = State::HitBottomLimit;
    return;
  }

  if (homing_) {
    if (limitBlocksDirection(homing_direction_)) {
      stop();

      // For this project, homing upward defines top as Z = 0. Keeping the
      // zero here also preserves legacy homeDown behavior if it is used.
      zeroPosition();

      state_ = (homing_direction_ == Direction::Up) ? State::HitTopLimit : State::HitBottomLimit;
      return;
    }

    setDirection(homing_direction_);

    if (tryStep(homing_direction_)) {
      state_ = State::Moving;
    } else {
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
    stop();
    state_ = (desired_direction == Direction::Up) ? State::HitTopLimit : State::HitBottomLimit;
    return;
  }

  if (tryStep(desired_direction)) {
    state_ = State::Moving;
  } else {
    state_ = State::Moving;
  }
}

/** @copydoc ZMotorDriver::moveSteps */
void ZMotorDriver::moveSteps(int32_t steps) {
  if (steps == 0) {
    return;
  }

  homing_ = false;
  target_steps_ = position_steps_ + steps;

  if (!tmc_.isEnabled()) {
    enable();
  }

  const Direction desired_direction = (target_steps_ > position_steps_) ? Direction::Up : Direction::Down;
  setDirection(desired_direction);

  if (limitBlocksDirection(desired_direction)) {
    stop();
    state_ = (desired_direction == Direction::Up) ? State::HitTopLimit : State::HitBottomLimit;
    return;
  }

  state_ = State::Moving;
}

/** @copydoc ZMotorDriver::moveTo */
void ZMotorDriver::moveTo(int32_t target_position_steps) {
  homing_ = false;
  target_steps_ = target_position_steps;

  if (!tmc_.isEnabled()) {
    enable();
  }

  if (position_steps_ == target_steps_) {
    state_ = State::Idle;
    return;
  }

  const Direction desired_direction = (target_steps_ > position_steps_) ? Direction::Up : Direction::Down;
  setDirection(desired_direction);

  if (limitBlocksDirection(desired_direction)) {
    stop();
    state_ = (desired_direction == Direction::Up) ? State::HitTopLimit : State::HitBottomLimit;
    return;
  }

  state_ = State::Moving;
}

/** @copydoc ZMotorDriver::jog */
void ZMotorDriver::jog(Direction direction, uint32_t speed_steps_per_second) {
  setSpeed(speed_steps_per_second);

  if (direction == Direction::Up) {
    moveTo(INT32_MAX);
  } else {
    moveTo(INT32_MIN);
  }
}

/** @copydoc ZMotorDriver::stop */
void ZMotorDriver::stop() {
  target_steps_ = position_steps_;
  homing_ = false;

  if (tmc_.isEnabled()) {
    state_ = State::Idle;
  }
}

/** @copydoc ZMotorDriver::home */
void ZMotorDriver::home(Direction direction, uint32_t speed_steps_per_second) {
  setSpeed(speed_steps_per_second);
  homing_direction_ = direction;
  homing_ = true;
  target_steps_ = position_steps_;

  if (!tmc_.isEnabled()) {
    enable();
  }

  setDirection(homing_direction_);

  if (limitBlocksDirection(homing_direction_)) {
    stop();
    zeroPosition();
    state_ = (homing_direction_ == Direction::Up) ? State::HitTopLimit : State::HitBottomLimit;
    return;
  }

  state_ = State::Moving;
}

/** @copydoc ZMotorDriver::homeUp */
void ZMotorDriver::homeUp(uint32_t speed_steps_per_second) {
  home(Direction::Up, speed_steps_per_second);
}

/** @copydoc ZMotorDriver::homeDown */
void ZMotorDriver::homeDown(uint32_t speed_steps_per_second) {
  home(Direction::Down, speed_steps_per_second);
}

/** @copydoc ZMotorDriver::zeroPosition */
void ZMotorDriver::zeroPosition() {
  position_steps_ = 0;
  target_steps_ = 0;
}

/** @copydoc ZMotorDriver::setSpeed */
void ZMotorDriver::setSpeed(uint32_t steps_per_second) {
  if (steps_per_second == 0) {
    steps_per_second = 1;
  }

  speed_steps_per_second_ = steps_per_second;
  tmc_.setStepRate(speed_steps_per_second_);
}

/** @copydoc ZMotorDriver::setSpeedStepsPerSecond */
void ZMotorDriver::setSpeedStepsPerSecond(uint32_t steps_per_second) {
  setSpeed(steps_per_second);
}

/** @copydoc ZMotorDriver::setMicrosteps */
void ZMotorDriver::setMicrosteps(uint16_t microsteps) {
  if (microsteps == 0) {
    microsteps = 1;
  }

  // With the present PCB wiring, microstepping is controlled by MS1/MS2 pins.
  // This value is stored for calculations/reporting, but it cannot change the
  // TMC2209 setting.
  microsteps_ = microsteps;
}

/** @copydoc ZMotorDriver::setLimitsActiveLow */
void ZMotorDriver::setLimitsActiveLow(bool active_low) {
  limits_active_low_ = active_low;
}

/** @copydoc ZMotorDriver::setDirectionInverted */
void ZMotorDriver::setDirectionInverted(bool inverted) {
  tmc_.setDirectionInverted(inverted);
  setDirection(current_direction_);
}

/** @copydoc ZMotorDriver::getPositionSteps */
int32_t ZMotorDriver::getPositionSteps() const {
  return position_steps_;
}

/** @copydoc ZMotorDriver::getTargetSteps */
int32_t ZMotorDriver::getTargetSteps() const {
  return target_steps_;
}

/** @copydoc ZMotorDriver::getState */
ZMotorDriver::State ZMotorDriver::getState() const {
  return state_;
}

/** @copydoc ZMotorDriver::isBusy */
bool ZMotorDriver::isBusy() const {
  return state_ == State::Moving;
}

/** @copydoc ZMotorDriver::topLimitPressed */
bool ZMotorDriver::topLimitPressed() const {
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(Z_TOP_GPIO_Port, Z_TOP_Pin);
  return limits_active_low_ ? (pin_state == GPIO_PIN_RESET) : (pin_state == GPIO_PIN_SET);
}

/** @copydoc ZMotorDriver::bottomLimitPressed */
bool ZMotorDriver::bottomLimitPressed() const {
  GPIO_PinState pin_state = HAL_GPIO_ReadPin(Z_BOT_GPIO_Port, Z_BOT_Pin);
  return limits_active_low_ ? (pin_state == GPIO_PIN_RESET) : (pin_state == GPIO_PIN_SET);
}

/** @copydoc ZMotorDriver::driverFaultActive */
bool ZMotorDriver::driverFaultActive() const {
  return tmc_.diagActive();
}

/** @copydoc ZMotorDriver::setDirection */
void ZMotorDriver::setDirection(Direction direction) {
  current_direction_ = direction;

  if (direction == Direction::Up) {
    tmc_.setDirection(TMC2209::Direction::Forward);
  } else {
    tmc_.setDirection(TMC2209::Direction::Reverse);
  }
}

/** @copydoc ZMotorDriver::tryStep */
bool ZMotorDriver::tryStep(Direction direction) {
  setDirection(direction);

  if (limitBlocksDirection(direction)) {
    stop();
    state_ = (direction == Direction::Up) ? State::HitTopLimit : State::HitBottomLimit;
    return false;
  }

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

/** @copydoc ZMotorDriver::limitBlocksDirection */
bool ZMotorDriver::limitBlocksDirection(Direction direction) const {
  if (direction == Direction::Up && topLimitPressed()) {
    return true;
  }

  if (direction == Direction::Down && bottomLimitPressed()) {
    return true;
  }

  return false;
}

/** @copydoc ZMotorDriver::motionCommandActive */
bool ZMotorDriver::motionCommandActive() const {
  return homing_ || (state_ == State::Moving && position_steps_ != target_steps_);
}
