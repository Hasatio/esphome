#include "bme680_bsec.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include <string>

namespace esphome {
namespace bme680_bsec {
#ifdef USE_BSEC
static const char *const TAG = "bme680_bsec.sensor";

#ifdef BME680_BSEC_CONFIGURATION
static const uint8_t bsec_configuration[] = BME680_BSEC_CONFIGURATION;
#endif

static const std::string IAQ_ACCURACY_STATES[4] = {"Stabilizing", "Uncertain", "Calibrating", "Calibrated"};

BME680BSECComponent *BME680BSECComponent::instance;  // NOLINT(cppcoreguidelines-avoid-non-const-global-variables)

void BME680BSECComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up BME680 via BSEC...");
  BME680BSECComponent::instance = this;

  this->bsec_status_ = bsec_init();
  if (this->bsec_status_ != BSEC_OK) {
    this->mark_failed();
    return;
  }

  this->bme680_.intf = BME68X_I2C_INTF;
  this->bme680_.read = BME680BSECComponent::read_bytes_wrapper;
  this->bme680_.write = BME680BSECComponent::write_bytes_wrapper;
  this->bme680_.delay_us = BME680BSECComponent::delay_us;
  this->bme680_.amb_temp = 25;

  this->bme680_status_ = bme68x_init(&this->bme680_);
  if (this->bme680_status_ != BME68X_OK) {
    this->mark_failed();
    return;
  }
  #ifdef BME680_BSEC_CONFIGURATION
  this->set_config_(bsec_configuration, sizeof(bsec_configuration));
  if (this->bsec_status_ != BSEC_OK) {
    this->mark_failed();
    return;
  }

  #endif

  this->update_subscription_();
  if (this->bsec_status_ != BSEC_OK) {
    this->mark_failed();
    return;
  }

  this->load_state_();
}

void BME680BSECComponent::set_config_(const uint8_t *config, uint32_t len) {
  if (len>BSEC_MAX_PROPERTY_BLOB_SIZE) {
    ESP_LOGE(TAG, "Configuration is too larger than BSEC_MAX_PROPERTY_BLOB_SIZE");
    this->mark_failed();
    return;
  }
  uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE];
  this->bsec_status_ = bsec_set_configuration(config, len, work_buffer, sizeof(work_buffer));
}

float BME680BSECComponent::calc_sensor_sample_rate_(SampleRate sample_rate) {
  if (sample_rate == SAMPLE_RATE_DEFAULT) {
    sample_rate = this->sample_rate_;
  }
  return sample_rate == SAMPLE_RATE_ULP ? BSEC_SAMPLE_RATE_ULP : BSEC_SAMPLE_RATE_LP;
}

void BME680BSECComponent::update_subscription_() {
  bsec_sensor_configuration_t virtual_sensors[BSEC_NUMBER_OUTPUTS];
  int num_virtual_sensors = 0;

  if (this->iaq_sensor_) {
    virtual_sensors[num_virtual_sensors].sensor_id = BSEC_OUTPUT_IAQ;
    virtual_sensors[num_virtual_sensors].sample_rate = this->calc_sensor_sample_rate_(SAMPLE_RATE_DEFAULT);
    num_virtual_sensors++;
  }

  if (this->iaq_static_sensor_) {
    virtual_sensors[num_virtual_sensors].sensor_id = BSEC_OUTPUT_STATIC_IAQ;
    virtual_sensors[num_virtual_sensors].sample_rate = this->calc_sensor_sample_rate_(SAMPLE_RATE_DEFAULT);
    num_virtual_sensors++;
  }

  if (this->co2_equivalent_sensor_) {
    virtual_sensors[num_virtual_sensors].sensor_id = BSEC_OUTPUT_CO2_EQUIVALENT;
    virtual_sensors[num_virtual_sensors].sample_rate = this->calc_sensor_sample_rate_(SAMPLE_RATE_DEFAULT);
    num_virtual_sensors++;
  }

  if (this->breath_voc_equivalent_sensor_) {
    virtual_sensors[num_virtual_sensors].sensor_id = BSEC_OUTPUT_BREATH_VOC_EQUIVALENT;
    virtual_sensors[num_virtual_sensors].sample_rate = this->calc_sensor_sample_rate_(SAMPLE_RATE_DEFAULT);
    num_virtual_sensors++;
  }

  if (this->pressure_sensor_) {
    virtual_sensors[num_virtual_sensors].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    virtual_sensors[num_virtual_sensors].sample_rate = this->calc_sensor_sample_rate_(this->pressure_sample_rate_);
    num_virtual_sensors++;
  }

  if (this->gas_resistance_sensor_) {
    virtual_sensors[num_virtual_sensors].sensor_id = BSEC_OUTPUT_RAW_GAS;
    virtual_sensors[num_virtual_sensors].sample_rate = this->calc_sensor_sample_rate_(SAMPLE_RATE_DEFAULT);
    num_virtual_sensors++;
  }

  if (this->temperature_sensor_) {
    virtual_sensors[num_virtual_sensors].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    virtual_sensors[num_virtual_sensors].sample_rate = this->calc_sensor_sample_rate_(this->temperature_sample_rate_);
    num_virtual_sensors++;
  }

  if (this->humidity_sensor_) {
    virtual_sensors[num_virtual_sensors].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    virtual_sensors[num_virtual_sensors].sample_rate = this->calc_sensor_sample_rate_(this->humidity_sample_rate_);
    num_virtual_sensors++;
  }

  bsec_sensor_configuration_t sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
  uint8_t num_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
  this->bsec_status_ =
      bsec_update_subscription(virtual_sensors, num_virtual_sensors, sensor_settings, &num_sensor_settings);
}

void BME680BSECComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "BME680 via BSEC:");

  bsec_version_t version;
  bsec_get_version(&version);
  ESP_LOGCONFIG(TAG, "  BSEC Version: %d.%d.%d.%d", version.major, version.minor, version.major_bugfix,
                version.minor_bugfix);

  LOG_I2C_DEVICE(this);

  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication failed (BSEC Status: %d, BME680 Status: %d)", this->bsec_status_,
             this->bme680_status_);
  }

  ESP_LOGCONFIG(TAG, "  Temperature Offset: %.2f", this->temperature_offset_);
  ESP_LOGCONFIG(TAG, "  Sample Rate: %s", BME680_BSEC_SAMPLE_RATE_LOG(this->sample_rate_));
  ESP_LOGCONFIG(TAG, "  State Save Interval: %ims", this->state_save_interval_ms_);

  LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  ESP_LOGCONFIG(TAG, "    Sample Rate: %s", BME680_BSEC_SAMPLE_RATE_LOG(this->temperature_sample_rate_));
  LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
  ESP_LOGCONFIG(TAG, "    Sample Rate: %s", BME680_BSEC_SAMPLE_RATE_LOG(this->pressure_sample_rate_));
  LOG_SENSOR("  ", "Humidity", this->humidity_sensor_);
  ESP_LOGCONFIG(TAG, "    Sample Rate: %s", BME680_BSEC_SAMPLE_RATE_LOG(this->humidity_sample_rate_));
  LOG_SENSOR("  ", "Gas Resistance", this->gas_resistance_sensor_);
  LOG_SENSOR("  ", "IAQ", this->iaq_sensor_);
  LOG_SENSOR("  ", "Numeric IAQ Accuracy", this->iaq_accuracy_sensor_);
  LOG_TEXT_SENSOR("  ", "IAQ Accuracy", this->iaq_accuracy_text_sensor_);
  LOG_SENSOR("  ", "CO2 Equivalent", this->co2_equivalent_sensor_);
  LOG_SENSOR("  ", "Breath VOC Equivalent", this->breath_voc_equivalent_sensor_);
}

float BME680BSECComponent::get_setup_priority() const { return setup_priority::DATA; }

void BME680BSECComponent::loop() {
  this->run_();

  if (this->bsec_status_ < BSEC_OK || this->bme680_status_ < BME68X_OK) {
    this->status_set_error();
  } else {
    this->status_clear_error();
  }
  if (this->bsec_status_ > BSEC_OK || this->bme680_status_ > BME68X_OK) {
    this->status_set_warning();
  } else {
    this->status_clear_warning();
  }
}

void BME680BSECComponent::run_() {
  int64_t curr_time_ns = this->get_time_ns_();
  if (curr_time_ns < this->next_call_ns_) {
    return;
  }
  this->op_mode = this->bsec_settings.op_mode;
  uint8_t status;

  ESP_LOGV(TAG, "Performing sensor run");

  struct bme68x_conf bme680_conf;
  this->bsec_status_ = bsec_sensor_control(curr_time_ns, &bsec_settings);
  if (this->bsec_status_ < BSEC_OK) {
    ESP_LOGW(TAG, "Failed to fetch sensor control settings (BSEC Error Code %d)", this->bsec_status_);
    return;
  }
  this->next_call_ns_ = bsec_settings.next_call;

  if (bsec_settings.trigger_measurement) {

    bme68x_get_conf(&bme680_conf,  &this->bme680_);

    bme680_conf.os_hum = bsec_settings.humidity_oversampling;
    bme680_conf.os_temp = bsec_settings.temperature_oversampling;
    bme680_conf.os_pres = bsec_settings.pressure_oversampling;
    bme68x_set_conf(&bme680_conf, &this->bme680_);


    switch (bsec_settings.op_mode)
    {
    case BME68X_FORCED_MODE:
      this->bme680_heatr_conf.enable = BME68X_ENABLE;
      this->bme680_heatr_conf.heatr_temp = bsec_settings.heater_temperature;
      this->bme680_heatr_conf.heatr_dur = bsec_settings.heater_duration;

      status = bme68x_set_op_mode(bsec_settings.op_mode, &this->bme680_);
      status = bme68x_set_heatr_conf(BME68X_FORCED_MODE, &this->bme680_heatr_conf, &this->bme680_);
      status = bme68x_set_op_mode(BME68X_FORCED_MODE, &this->bme680_);
      this->op_mode = BME68X_FORCED_MODE;
      this->sleep_mode = false;
      ESP_LOGV(TAG, "Using forced mode");


      break;
    case BME68X_PARALLEL_MODE:
      if (this->op_mode != bsec_settings.op_mode)
      {
        this->bme680_heatr_conf.enable = BME68X_ENABLE;
        this->bme680_heatr_conf.heatr_temp_prof = bsec_settings.heater_temperature_profile;
        this->bme680_heatr_conf.heatr_dur_prof = bsec_settings.heater_duration_profile;
        this->bme680_heatr_conf.profile_len = bsec_settings.heater_profile_len;
        this->bme680_heatr_conf.shared_heatr_dur = BSEC_TOTAL_HEAT_DUR - (bme68x_get_meas_dur(BME68X_PARALLEL_MODE, &bme680_conf, &this->bme680_) / INT64_C(1000));;

        status = bme68x_set_heatr_conf(BME68X_PARALLEL_MODE, &this->bme680_heatr_conf, &this->bme680_);

        status = bme68x_set_op_mode(BME68X_PARALLEL_MODE, &this->bme680_);
        this->op_mode = BME68X_PARALLEL_MODE;
        this->sleep_mode = false;
        ESP_LOGV(TAG, "Using parallel mode");

      }
      break;
    case BME68X_SLEEP_MODE:
      if (!this->sleep_mode)
      {
          bme68x_set_op_mode(BME68X_SLEEP_MODE, &this->bme680_);
          //this->opMode = BME68X_SLEEP_MODE;
          this->sleep_mode = true;
          ESP_LOGV(TAG, "Using sleep mode");

      }
      break;

    }

    uint32_t meas_dur = 0;
    meas_dur = bme68x_get_meas_dur(this->op_mode, &bme680_conf, &this->bme680_);
    ESP_LOGV(TAG, "Queueing read in %uus", meas_dur);
    this->set_timeout("read", meas_dur/1000,
                      [this, curr_time_ns]() { this->read_(curr_time_ns); });
  } else {
    ESP_LOGV(TAG, "Measurement not required");
    this->read_(curr_time_ns);
  }
}

void BME680BSECComponent::read_(int64_t trigger_time_ns) {
  ESP_LOGV(TAG, "Reading data");

  if (bsec_settings.trigger_measurement) {
    uint8_t current_op_mode;
    this->bme680_status_ = bme68x_get_op_mode(&current_op_mode, &this->bme680_);

    if (current_op_mode == BME68X_SLEEP_MODE) {
      ESP_LOGV(TAG, "still in sleep mode, doing nothing");
      return;
    }
    //TODO: this loop looks dangerous to me, do we really need it?
    // while (current_op_mode != BME68X_SLEEP_MODE) { 
    //   this->bme680_status_ = bme68x_get_op_mode(&current_op_mode, &this->bme680_);
    //   if (this->bme680_status_ != BME68X_OK) {
    //     ESP_LOGW(TAG, "Failed to get sensor mode (BME680 Error Code %d)", this->bme680_status_);
    //   }
    // }
  }

  if (!bsec_settings.process_data) {
    ESP_LOGV(TAG, "Data processing not required");
    return;
  }

  struct bme68x_data data[3];
  uint8_t nFields = 0;
  this->bme680_status_ = bme68x_get_data(this->op_mode, &data[0], &nFields, &this->bme680_);

  if (this->bme680_status_ != BME68X_OK) {
    ESP_LOGW(TAG, "Failed to get sensor data (BME680 Error Code %d)", this->bme680_status_);
    return;
  }
  if (nFields < 1) {
    ESP_LOGD(TAG, "BME680 did not provide new data");
    return;
  }
  
  for (uint8_t i=0; i<nFields; i++) {
    bsec_input_t inputs[BSEC_MAX_PHYSICAL_SENSOR];  // Temperature, Pressure, Humidity & Gas Resistance
    uint8_t num_inputs = 0;

    if (BSEC_CHECK_INPUT(bsec_settings.process_data, BSEC_INPUT_TEMPERATURE)) {
      inputs[num_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
      inputs[num_inputs].signal = data[i].temperature;
      inputs[num_inputs].time_stamp = trigger_time_ns;
      num_inputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_settings.process_data, BSEC_INPUT_HEATSOURCE)) {
      inputs[num_inputs].sensor_id = BSEC_INPUT_HEATSOURCE;
      inputs[num_inputs].signal = this->temperature_offset_;
      inputs[num_inputs].time_stamp = trigger_time_ns;
      num_inputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_settings.process_data, BSEC_INPUT_HUMIDITY)) {
      inputs[num_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
      inputs[num_inputs].signal = data[i].humidity;
      inputs[num_inputs].time_stamp = trigger_time_ns;
      num_inputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_settings.process_data, BSEC_INPUT_PRESSURE)) {
      inputs[num_inputs].sensor_id = BSEC_INPUT_PRESSURE;
      inputs[num_inputs].signal = data[i].pressure;
      inputs[num_inputs].time_stamp = trigger_time_ns;
      num_inputs++;
    }
    if (BSEC_CHECK_INPUT(bsec_settings.process_data, BSEC_INPUT_GASRESISTOR)) {
      if (data[i].status & BME68X_GASM_VALID_MSK) {
        inputs[num_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
        inputs[num_inputs].signal = data[i].gas_resistance;
        inputs[num_inputs].time_stamp = trigger_time_ns;
        num_inputs++;
      } else {
        ESP_LOGD(TAG, "BME680 did not report gas data");
      }
    }
    if (BSEC_CHECK_INPUT(bsec_settings.process_data, BSEC_INPUT_PROFILE_PART) &&
            (data[i].status & BME68X_GASM_VALID_MSK))
    {
        inputs[num_inputs].sensor_id = BSEC_INPUT_PROFILE_PART;
        inputs[num_inputs].signal = (this->op_mode == BME68X_FORCED_MODE) ? 0 : data[i].gas_index;
        inputs[num_inputs].time_stamp = trigger_time_ns;
        num_inputs++;
    }

    if (num_inputs < 1) {
      ESP_LOGD(TAG, "No signal inputs available for BSEC");
      return;
    }

    bsec_output_t outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t num_outputs = BSEC_NUMBER_OUTPUTS;
    this->bsec_status_ = bsec_do_steps(inputs, num_inputs, outputs, &num_outputs);
    if (this->bsec_status_ != BSEC_OK) {
      ESP_LOGW(TAG, "BSEC failed to process signals (BSEC Error Code %d)", this->bsec_status_);
      return;
    }
    if (num_outputs < 1) {
      ESP_LOGD(TAG, "No signal outputs provided by BSEC");
      return;
    }

    this->publish_(outputs, num_outputs);
  }
}

void BME680BSECComponent::publish_(const bsec_output_t *outputs, uint8_t num_outputs) {
  ESP_LOGV(TAG, "Publishing sensor states");
  uint8_t accuracy = 0xff;
  for (uint8_t i = 0; i < num_outputs; i++) {
    switch (outputs[i].sensor_id) {
      case BSEC_OUTPUT_IAQ:
        accuracy = outputs[i].accuracy;
        this->publish_sensor_state_(this->iaq_sensor_, outputs[i].signal);
        break;
      case BSEC_OUTPUT_STATIC_IAQ:
        accuracy = outputs[i].accuracy;
        this->publish_sensor_state_(this->iaq_static_sensor_, outputs[i].signal);
        break;
      case BSEC_OUTPUT_CO2_EQUIVALENT:
        this->publish_sensor_state_(this->co2_equivalent_sensor_, outputs[i].signal);
        break;
      case BSEC_OUTPUT_BREATH_VOC_EQUIVALENT:
        this->publish_sensor_state_(this->breath_voc_equivalent_sensor_, outputs[i].signal);
        break;
      case BSEC_OUTPUT_RAW_PRESSURE:
        this->publish_sensor_state_(this->pressure_sensor_, outputs[i].signal / 100.0f);
        break;
      case BSEC_OUTPUT_RAW_GAS:
        this->publish_sensor_state_(this->gas_resistance_sensor_, outputs[i].signal);
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
        this->publish_sensor_state_(this->temperature_sensor_, outputs[i].signal);
        break;
      case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
        this->publish_sensor_state_(this->humidity_sensor_, outputs[i].signal);
        break;
    }
    if (accuracy != 0xff) {
        this->publish_sensor_state_(this->iaq_accuracy_text_sensor_, IAQ_ACCURACY_STATES[accuracy]);
        this->publish_sensor_state_(this->iaq_accuracy_sensor_, accuracy, true);
        // Queue up an opportunity to save state
        this->defer("save_state", [this, accuracy]() { this->save_state_(accuracy); });

    }
  }
}

int64_t BME680BSECComponent::get_time_ns_() {
  int64_t time_ms = millis();
  if (this->last_time_ms_ > time_ms) {
    this->millis_overflow_counter_++;
  }
  this->last_time_ms_ = time_ms;

  return (time_ms + ((int64_t) this->millis_overflow_counter_ << 32)) * INT64_C(1000000);
}

void BME680BSECComponent::publish_sensor_state_(sensor::Sensor *sensor, float value, bool change_only) {
  if (!sensor || (change_only && sensor->has_state() && sensor->state == value)) {
    return;
  }
  sensor->publish_state(value);
}

void BME680BSECComponent::publish_sensor_state_(text_sensor::TextSensor *sensor, const std::string &value) {
  if (!sensor || (sensor->has_state() && sensor->state == value)) {
    return;
  }
  sensor->publish_state(value);
}

int8_t BME680BSECComponent::read_bytes_wrapper(uint8_t a_register, uint8_t *data, uint32_t len, void *intfPtr) {
  return BME680BSECComponent::instance->read_bytes(a_register, data, len) ? 0 : -1;
}

int8_t BME680BSECComponent::write_bytes_wrapper(uint8_t a_register, const uint8_t *data, uint32_t len, void *intfPtr) {
  return BME680BSECComponent::instance->write_bytes(a_register, data, len) ? 0 : -1;
}

void BME680BSECComponent::delay_us(uint32_t period, void *intfPtr) {
  ESP_LOGV(TAG, "Delaying for %uus", period);
  delayMicroseconds(period);
}

void BME680BSECComponent::load_state_() {
  uint32_t hash = fnv1_hash("bme680_bsec_state_" + to_string(this->address_));
  this->bsec_state_ = global_preferences->make_preference<uint8_t[BSEC_MAX_STATE_BLOB_SIZE]>(hash, true);

  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
  if (this->bsec_state_.load(&state)) {
    ESP_LOGV(TAG, "Loading state");
    uint8_t work_buffer[BSEC_MAX_WORKBUFFER_SIZE];
    this->bsec_status_ = bsec_set_state(state, BSEC_MAX_STATE_BLOB_SIZE, work_buffer, sizeof(work_buffer));
    if (this->bsec_status_ != BSEC_OK) {
      ESP_LOGW(TAG, "Failed to load state (BSEC Error Code %d)", this->bsec_status_);
    }
    ESP_LOGI(TAG, "Loaded state");
  }
}

void BME680BSECComponent::save_state_(uint8_t accuracy) {
  if (accuracy < 3 || (millis() - this->last_state_save_ms_ < this->state_save_interval_ms_)) {
    return;
  }

  ESP_LOGV(TAG, "Saving state");

  uint8_t state[BSEC_MAX_STATE_BLOB_SIZE];
  uint8_t work_buffer[BSEC_MAX_STATE_BLOB_SIZE];
  uint32_t num_serialized_state = BSEC_MAX_STATE_BLOB_SIZE;

  this->bsec_status_ =
      bsec_get_state(0, state, BSEC_MAX_STATE_BLOB_SIZE, work_buffer, BSEC_MAX_STATE_BLOB_SIZE, &num_serialized_state);
  if (this->bsec_status_ != BSEC_OK) {
    ESP_LOGW(TAG, "Failed fetch state for save (BSEC Error Code %d)", this->bsec_status_);
    return;
  }

  if (!this->bsec_state_.save(&state)) {
    ESP_LOGW(TAG, "Failed to save state");
    return;
  }
  this->last_state_save_ms_ = millis();

  ESP_LOGI(TAG, "Saved state");
}
#endif
}  // namespace bme680_bsec
}  // namespace esphome
