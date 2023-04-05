#pragma once

#include "esphome/core/defines.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"
#include "esphome/components/output/float_output.h"
#include "esphome/components/i2c/i2c.h"

#ifdef USE_BINARY_SENSOR
#include "esphome/components/binary_sensor/binary_sensor.h"
#endif

#ifdef USE_SENSOR
#include "esphome/components/sensor/sensor.h"
#endif

#ifdef USE_TEXT_SENSOR
#include "esphome/components/text_sensor/text_sensor.h"
#endif

namespace esphome {
namespace pca9685custom {

/// Inverts polarity of channel output signal
extern const uint8_t PCA9685custom_MODE_INVERTED;
/// Channel update happens upon ACK (post-set) rather than on STOP (endTransmission)
extern const uint8_t PCA9685custom_MODE_OUTPUT_ONACK;
/// Use a totem-pole (push-pull) style output rather than an open-drain structure.
extern const uint8_t PCA9685custom_MODE_OUTPUT_TOTEM_POLE;
/// For active low output enable, sets channel output to high-impedance state
extern const uint8_t PCA9685custom_MODE_OUTNE_HIGHZ;
/// Similarly, sets channel output to high if in totem-pole mode, otherwise
extern const uint8_t PCA9685custom_MODE_OUTNE_LOW;

class PCA9685customOutput;

class PCA9685customChannel : public output::FloatOutput {
 public:
  void set_channel(uint8_t channel) { channel_ = channel; }
  void set_parent(PCA9685customOutput *parent) { parent_ = parent; }

 protected:
  friend class PCA9685customOutput;

  void write_state(float state) override;

  uint8_t channel_;
  PCA9685customOutput *parent_;
};

/// PCA9685 float output component.
class PCA9685customOutput : public Component, public i2c::I2CDevice {
 public:

double gain = 1.0;
double offset = 0.0;

  PCA9685customOutput(uint8_t mode = PCA9685custom_MODE_OUTPUT_ONACK | PCA9685custom_MODE_OUTPUT_TOTEM_POLE) : mode_(mode) {}

  void register_channel(PCA9685customChannel *channel);

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::HARDWARE; }
  void loop() override;
  void set_extclk(bool extclk) { this->extclk_ = extclk; }
  void set_frequency(float frequency) { this->frequency_ = frequency; }

#ifdef USE_SENSOR
  void set_current_volume_dosed(sensor::Sensor *current_volume_dosed) { current_volume_dosed_ = current_volume_dosed; }
#endif

  void dose_volume(double volume);
  void pause_dosing();
  void stop_dosing();

 protected:

  void send_next_command_();
  void read_command_result_();
  void queue_command_(uint16_t command, double volume, int duration, bool should_schedule);

#ifdef USE_SENSOR
  sensor::Sensor *current_volume_dosed_{nullptr};
#endif

  friend PCA9685customChannel;

template<typename... Ts> class PCA9685customOutputDoseVolumeAction : public Action<Ts...> {
 public:
  PCA9685customOutputDoseVolumeAction(PCA9685customOutput *pca9685custom) : pca9685custom_(pca9685custom) {}

  void play(Ts... x) override { this->pca9685custom_->dose_volume(this->volume_.value(x...)); }
  TEMPLATABLE_VALUE(double, volume)

 protected:
  PCA9685customOutput *pca9685custom_;
};

  void set_channel_value_(uint8_t channel, double value2) {
    if (this->pwm_amounts_[channel] != (int)value2)
      this->update_ = true;
    this->pwm_amounts_[channel] = value2*gain+offset;

  }

  float frequency_;
  uint8_t mode_;
  bool extclk_ = false;

  uint8_t min_channel_{0xFF};
  uint8_t max_channel_{0x00};
  uint16_t pwm_amounts_[16] = {
      0,
  };
  bool update_{true};
};

}  // namespace pca9685custom
}  // namespace esphome