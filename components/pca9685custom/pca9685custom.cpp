#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "esphome/core/hal.h"
#include "pca9685custom.h"

namespace esphome {
namespace pca9685custom {

static const char *const TAG = "pca9685custom";

static const uint16_t PCA9685custom_COMMAND_NONE = 0;
static const uint16_t PCA9685custom_COMMAND_TYPE_READ = 1;
static const uint16_t PCA9685custom_COMMAND_DOSE_VOLUME = 8;
static const uint16_t PCA9685custom_COMMAND_READ_DOSING = 3;
static const uint16_t PCA9685custom_COMMAND_READ_SINGLE_REPORT = 5;
static const std::string DOSING_MODE_NONE = "None";
static const std::string DOSING_MODE_VOLUME = "Volume";

const uint8_t PCA9685custom_MODE_INVERTED = 0x10;
const uint8_t PCA9685custom_MODE_OUTPUT_ONACK = 0x08;
const uint8_t PCA9685custom_MODE_OUTPUT_TOTEM_POLE = 0x04;
const uint8_t PCA9685custom_MODE_OUTNE_HIGHZ = 0x02;
const uint8_t PCA9685custom_MODE_OUTNE_LOW = 0x01;

static const uint8_t PCA9685custom_REGISTER_SOFTWARE_RESET = 0x06;
static const uint8_t PCA9685custom_REGISTER_MODE1 = 0x00;
static const uint8_t PCA9685custom_REGISTER_MODE2 = 0x01;
static const uint8_t PCA9685custom_REGISTER_LED0 = 0x06;
static const uint8_t PCA9685custom_REGISTER_PRE_SCALE = 0xFE;

static const uint8_t PCA9685custom_MODE1_RESTART = 0b10000000;
static const uint8_t PCA9685custom_MODE1_EXTCLK = 0b01000000;
static const uint8_t PCA9685custom_MODE1_AUTOINC = 0b00100000;
static const uint8_t PCA9685custom_MODE1_SLEEP = 0b00010000;

void PCA9685customOutput::setup() {
  ESP_LOGCONFIG(TAG, "Setting up PCA9685customOutputComponent...");

  ESP_LOGV(TAG, "  Resetting devices...");
  uint8_t address_tmp = this->address_;
  this->set_i2c_address(0x00);
  if (!this->write_bytes(PCA9685custom_REGISTER_SOFTWARE_RESET, nullptr, 0)) {
    this->mark_failed();
    return;
  }
  this->set_i2c_address(address_tmp);

  if (!this->write_byte(PCA9685custom_REGISTER_MODE1, PCA9685custom_MODE1_RESTART | PCA9685custom_MODE1_AUTOINC)) {
    this->mark_failed();
    return;
  }
  if (!this->write_byte(PCA9685custom_REGISTER_MODE2, this->mode_)) {
    this->mark_failed();
    return;
  }

  uint8_t mode1;
  if (!this->read_byte(PCA9685custom_REGISTER_MODE1, &mode1)) {
    this->mark_failed();
    return;
  }
  mode1 = (mode1 & ~PCA9685custom_MODE1_RESTART) | PCA9685custom_MODE1_SLEEP;
  if (!this->write_byte(PCA9685custom_REGISTER_MODE1, mode1)) {
    this->mark_failed();
    return;
  }

  int pre_scaler = 3;
  if (this->extclk_) {
    mode1 = mode1 | PCA9685custom_MODE1_EXTCLK;
    if (!this->write_byte(PCA9685custom_REGISTER_MODE1, mode1)) {
      this->mark_failed();
      return;
    }
  } else {
    pre_scaler = static_cast<int>((25000000 / (4096 * this->frequency_)) - 1);
    pre_scaler = clamp(pre_scaler, 3, 255);

    ESP_LOGV(TAG, "  -> Prescaler: %d", pre_scaler);
  }
  if (!this->write_byte(PCA9685custom_REGISTER_PRE_SCALE, pre_scaler)) {
    this->mark_failed();
    return;
  }

  mode1 = (mode1 & ~PCA9685custom_MODE1_SLEEP) | PCA9685custom_MODE1_RESTART;
  if (!this->write_byte(PCA9685custom_REGISTER_MODE1, mode1)) {
    this->mark_failed();
    return;
  }
  delayMicroseconds(500);

  this->loop();
}

void PCA9685customOutput::dump_config() {
  ESP_LOGCONFIG(TAG, "PCA9685custom:");
  ESP_LOGCONFIG(TAG, "  Mode: 0x%02X", this->mode_);
  if (this->extclk_) {
    ESP_LOGCONFIG(TAG, "  EXTCLK: enabled");
  } else {
    ESP_LOGCONFIG(TAG, "  EXTCLK: disabled");
    ESP_LOGCONFIG(TAG, "  Frequency: %.0f Hz", this->frequency_);
  }
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Setting up PCA9685custom failed!");
  }
}

void PCA9685customOutput::loop() {
  if (this->min_channel_ == 0xFF || !this->update_)
    return;

  const uint16_t num_channels = this->max_channel_ - this->min_channel_ + 1;
  for (uint8_t channel = this->min_channel_; channel <= this->max_channel_; channel++) {
    uint16_t phase_begin = uint16_t(channel - this->min_channel_) / num_channels * 4096;
    uint16_t phase_end;
    uint16_t amount = this->pwm_amounts_[channel];
    if (amount == 0) {
      phase_end = 4096;
    } else if (amount >= 4096) {
      phase_begin = 4096;
      phase_end = 0;
    } else {
      phase_end = phase_begin + amount;
      if (phase_end >= 4096)
        phase_end -= 4096;
    }

    ESP_LOGVV(TAG, "Channel %02u: amount=%04u phase_begin=%04u phase_end=%04u", channel, amount, phase_begin,
              phase_end);

    uint8_t data[4];
    data[0] = phase_begin & 0xFF;
    data[1] = (phase_begin >> 8) & 0xFF;
    data[2] = phase_end & 0xFF;
    data[3] = (phase_end >> 8) & 0xFF;

    uint8_t reg = PCA9685custom_REGISTER_LED0 + 4 * channel;
    if (!this->write_bytes(reg, data, 4)) {
      this->status_set_warning();
      return;
    }
  }

  this->status_clear_warning();
  this->update_ = false;
}

void PCA9685customOutput::register_channel(PCA9685customChannel *channel) {
  auto c = channel->channel_;
  this->min_channel_ = std::min(this->min_channel_, c);
  this->max_channel_ = std::max(this->max_channel_, c);
  channel->set_parent(this);
}

void PCA9685customChannel::write_state(float state) {
  const uint16_t max_duty = 4096;
  const float duty_rounded = roundf(state * max_duty);
  auto duty = static_cast<uint16_t>(duty_rounded);
  this->parent_->set_channel_value_(this->channel_, duty);
}

void PCA9685customOutput::queue_command_(uint16_t command, double volume, int duration, bool should_schedule) {
  if (!should_schedule) {
    return;
  }

  if (this->next_command_queue_length_ >= 10) {
    ESP_LOGE(TAG, "Tried to queue command '%d' but queue is full", command);
    return;
  }

  this->next_command_queue_[this->next_command_queue_last_] = command;
  this->next_command_volume_queue_[this->next_command_queue_last_] = volume;
  this->next_command_duration_queue_[this->next_command_queue_last_] = duration;

  ESP_LOGV(TAG, "Queue command '%d' in position '%d'", command, next_command_queue_last_);

  // Move positions
  next_command_queue_last_++;
  if (next_command_queue_last_ >= 10) {
    next_command_queue_last_ = 0;
  }

  next_command_queue_length_++;
}

void PCA9685customOutput::read_command_result_() {
  uint8_t response_buffer[21] = {'\0'};

  response_buffer[0] = 0;
  if (!this->read_bytes_raw(response_buffer, 20)) {
    ESP_LOGE(TAG, "read error");
    this->clear_current_command_();
    return;
  }

  switch (response_buffer[0]) {
    case 254:
      return;  // keep waiting
    case 1:
      break;
    case 2:
      ESP_LOGE(TAG, "device returned a syntax error");
      this->clear_current_command_();
      return;
    case 255:
      ESP_LOGE(TAG, "device returned no data");
      this->clear_current_command_();
      return;
    default:
      ESP_LOGE(TAG, "device returned an unknown response: %d", response_buffer[0]);
      this->clear_current_command_();
      return;
  }

  char first_parameter_buffer[10] = {'\0'};
  char second_parameter_buffer[10] = {'\0'};
  char third_parameter_buffer[10] = {'\0'};

  first_parameter_buffer[0] = '\0';
  second_parameter_buffer[0] = '\0';
  third_parameter_buffer[0] = '\0';

  int current_parameter = 1;

  size_t position_in_parameter_buffer = 0;
  // some sensors return multiple comma-separated values, terminate string after first one
  for (size_t i = 1; i < sizeof(response_buffer) - 1; i++) {
    char current_char = response_buffer[i];

    if (current_char == '\0') {
      ESP_LOGV(TAG, "Read Response from device: %s", (char *) response_buffer);
      ESP_LOGV(TAG, "First Component: %s", (char *) first_parameter_buffer);
      ESP_LOGV(TAG, "Second Component: %s", (char *) second_parameter_buffer);
      ESP_LOGV(TAG, "Third Component: %s", (char *) third_parameter_buffer);

      break;
    }

    if (current_char == ',') {
      current_parameter++;
      position_in_parameter_buffer = 0;
      continue;
    }

    switch (current_parameter) {
      case 1:
        first_parameter_buffer[position_in_parameter_buffer] = current_char;
        first_parameter_buffer[position_in_parameter_buffer + 1] = '\0';
        break;
      case 2:
        second_parameter_buffer[position_in_parameter_buffer] = current_char;
        second_parameter_buffer[position_in_parameter_buffer + 1] = '\0';
        break;
      case 3:
        third_parameter_buffer[position_in_parameter_buffer] = current_char;
        third_parameter_buffer[position_in_parameter_buffer + 1] = '\0';
        break;
    }

    position_in_parameter_buffer++;
  }

  auto parsed_first_parameter = parse_number<float>(first_parameter_buffer);
  auto parsed_second_parameter = parse_number<float>(second_parameter_buffer);
  auto parsed_third_parameter = parse_number<float>(third_parameter_buffer);

  switch (this->current_command_) {
    // Read Commands
    case PCA9685custom_COMMAND_DOSE_VOLUME:  // Volume Dispensing (page 55)
      if (this->dosing_mode_ && this->dosing_mode_->state != DOSING_MODE_VOLUME)
        this->dosing_mode_->publish_state(DOSING_MODE_VOLUME);
      break;
    default:
      ESP_LOGE(TAG, "Unsupported command received: %d", this->current_command_);
      return;
  }

  this->clear_current_command_();
}

void PCA9685customOutput::send_next_command_() {
  int wait_time_for_command = 500;  // milliseconds
  uint8_t command_buffer[21];
  int command_buffer_length = 0;

  this->pop_next_command_();  // this->next_command will be updated.
  switch (this->next_command_) {
    // Read Commands
    case PCA9685custom_COMMAND_DOSE_VOLUME:  // Volume Dispensing (page 55)
      command_buffer_length = sprintf((char *) command_buffer, "D,%0.1f", this->next_command_volume_);
      break;
    default:
      ESP_LOGE(TAG, "Unsupported command received: %d", this->next_command_);
      return;
  }
  // Send command
  ESP_LOGV(TAG, "Sending command to device: %s", (char *) command_buffer);
  this->write(command_buffer, command_buffer_length);

  this->current_command_ = this->next_command_;
  this->next_command_ = PCA9685custom_COMMAND_NONE;
  this->is_waiting_ = true;
  this->start_time_ = millis();
  this->wait_time_ = wait_time_for_command;
}
	
void PCA9685customOutput::dose_volume(double volume) {
  this->queue_command_(PCA9685custom_COMMAND_DOSE_VOLUME, volume, 0, true);
  this->queue_command_(PCA9685custom_COMMAND_READ_DOSING, 0, 0, true);
  this->queue_command_(PCA9685custom_COMMAND_READ_SINGLE_REPORT, 0, 0, (bool) this->current_volume_dosed_);
}

}  // namespace pca9685custom
}  // namespace esphome