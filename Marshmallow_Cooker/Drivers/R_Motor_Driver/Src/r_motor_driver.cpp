#include "r_motor_driver.h"

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;

RMotorDriver::RMotorDriver()
    : driver_(&htim1, TIM_CHANNEL_2, TIM_CHANNEL_3),
      encoder_(&htim3) {
}

HAL_StatusTypeDef RMotorDriver::begin() {
  encoder_.reset();

  HAL_StatusTypeDef status = driver_.begin();

  if (status == HAL_OK) {
    state_ = State::Idle;
  } else {
    state_ = State::Fault;
  }

  return status;
}

void RMotorDriver::update() {
  encoder_.update();

  switch (state_) {
    case State::Disabled:
      break;

    case State::Idle:
      break;

    case State::MovingToPosition: {
      const uint32_t now_ms = HAL_GetTick();

      if ((now_ms - move_start_ms_) >= move_timeout_ms_) {
        brake();
        state_ = State::Fault;
        return;
      }

      const int32_t current_counts = getPosition();
      const int32_t error_counts = target_counts_ - current_counts;

      if (error_counts <= kPositionToleranceCounts && error_counts >= -kPositionToleranceCounts) {
        brake();
        state_ = State::Idle;
        return;
      }

      const int32_t direction = error_counts >= 0 ? 1 : -1;
      setDuty(commanded_duty_ * direction);

      break;
    }

    case State::RotatingContinuous:
      // Nothing else needed here yet.
      // Encoder update still happens at the top.
      break;

    case State::Fault:
      brake();
      break;
  }
}

void RMotorDriver::setDuty(int16_t duty) {
  driver_.setDuty(clampPower(duty));
}

void RMotorDriver::brake() {
  driver_.brake();
}

void RMotorDriver::coast() {
  driver_.coast();
}

int16_t RMotorDriver::getVelocity() const {
  return encoder_.getVelocity();
}

int32_t RMotorDriver::getPosition() const {
  return encoder_.getPosition();
}

int32_t RMotorDriver::getPositionDegrees() const {
  return countsToDegrees(encoder_.getPosition());
}

void RMotorDriver::resetEncoder() {
  encoder_.reset();
}

int32_t RMotorDriver::degreesToCounts(int32_t degrees) const {
  return (degrees * kCountsPerOutputRev) / 360;
}

int32_t RMotorDriver::countsToDegrees(int32_t counts) const {
  return (counts * 360) / kCountsPerOutputRev;
}

int16_t RMotorDriver::clampPower(int32_t power) const {
  if (power > kMaxPower) {
    return kMaxPower;
  }

  if (power < -kMaxPower) {
    return -kMaxPower;
  }

  return static_cast<int16_t>(power);
}

void RMotorDriver::moveDegreesBlocking(int32_t degrees, int16_t duty, uint32_t timeout_ms) {

  if (degrees == 0 || duty == 0) {
    brake();
    return;
  }

  resetEncoder();

  const int32_t target_counts = degreesToCounts(degrees);
  const int32_t direction = target_counts >= 0 ? 1 : -1;
  const int32_t abs_target_counts = target_counts >= 0 ? target_counts : -target_counts;

  duty = duty >= 0 ? duty : -duty;
  setDuty(duty * direction);
  const uint32_t start_ms = HAL_GetTick();

  while ((HAL_GetTick() - start_ms) < timeout_ms) {
    update();
    int32_t position = getPosition() * direction;
    if (position >= abs_target_counts) {
      break;
    }

    HAL_Delay(1);
  }

  brake();
}

void RMotorDriver::stop() {
  brake();
  state_ = State::Idle;
}

void RMotorDriver::moveDegrees(int32_t degrees, int16_t duty, uint32_t timeout_ms) {
  if (degrees == 0 || duty == 0) {
    stop();
    return;
  }

  update();

  move_start_counts_ = getPosition();
  target_counts_ = move_start_counts_ + degreesToCounts(degrees);

  commanded_duty_ = duty >= 0 ? duty : -duty;
  commanded_duty_ = clampPower(commanded_duty_);

  move_start_ms_ = HAL_GetTick();
  move_timeout_ms_ = timeout_ms;

  state_ = State::MovingToPosition;
}

void RMotorDriver::moveToDegrees(int32_t target_degrees, int16_t duty, uint32_t timeout_ms) {
  target_counts_ = degreesToCounts(target_degrees);

  commanded_duty_ = duty >= 0 ? duty : -duty;
  commanded_duty_ = clampPower(commanded_duty_);

  move_start_counts_ = getPosition();
  move_start_ms_ = HAL_GetTick();
  move_timeout_ms_ = timeout_ms;

  state_ = State::MovingToPosition;
}

void RMotorDriver::rotateContinuous(int16_t duty) {
  if (duty == 0) {
    stop();
    return;
  }

  commanded_duty_ = clampPower(duty);
  setDuty(commanded_duty_);

  state_ = State::RotatingContinuous;
}

bool RMotorDriver::isBusy() const {
  return state_ == State::MovingToPosition || state_ == State::RotatingContinuous;
}

bool RMotorDriver::isFaulted() const {
  return state_ == State::Fault;
}

RMotorDriver::State RMotorDriver::getState() const {
  return state_;
}