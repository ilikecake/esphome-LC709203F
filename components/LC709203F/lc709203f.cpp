#include "lc709203f.h"
#include "esphome/core/log.h"

namespace esphome {
namespace lc709203f {

// MAX174043 is a 1-Cell Fuel Gauge with ModelGauge and Low-Battery Alert
// Consult the datasheet at https://www.analog.com/en/products/max17043.html

static const char *const TAG = "lc709203f";

static const uint8_t LC709203F_I2C_ADDR_DEFAULT         = 0x0B;

static const uint8_t LC709203F_BEFORE_RSOC              = 0x04;
static const uint8_t LC709203F_THERMISTOR_B             = 0x06;
static const uint8_t LC709203F_INITIAL_RSOC             = 0x07;
static const uint8_t LC709203F_CELL_TEMPERATURE         = 0x08;
static const uint8_t LC709203F_CELL_VOLTAGE             = 0x09;
static const uint8_t LC709203F_CURRENT_DIRECTION        = 0x0A;
static const uint8_t LC709203F_APA                      = 0x0B;
static const uint8_t LC709203F_APT                      = 0x0C;
static const uint8_t LC709203F_RSOC                     = 0x0D;
static const uint8_t LC709203F_ITE                      = 0x0F;
static const uint8_t LC709203F_IC_VERSION               = 0x11;
static const uint8_t LC709203F_CHANGE_OF_THE_PARAMETER  = 0x12;
static const uint8_t LC709203F_ALARM_LOW_RSOC           = 0x13;
static const uint8_t LC709203F_ALARM_LOW_CELL_VOLTAGE   = 0x14;
static const uint8_t LC709203F_IC_POWER_MODE            = 0x15;
static const uint8_t LC709203F_STATUS_BIT               = 0x16;
static const uint8_t LC709203F_NUMBER_OF_THE_PARAMETER  = 0x1A;

/*static const uint8_t MAX17043_VCELL = 0x02;
static const uint8_t MAX17043_SOC = 0x04;
static const uint8_t MAX17043_CONFIG = 0x0c;

static const uint16_t MAX17043_CONFIG_POWER_UP_DEFAULT = 0x971C;
static const uint16_t MAX17043_CONFIG_SAFE_MASK = 0xFF1F;  // mask out sleep bit (7), unused bit (6) and alert bit (4)
static const uint16_t MAX17043_CONFIG_SLEEP_MASK = 0x0080;
*/

void LC709203FComponent::update() {
  uint16_t raw_voltage, raw_percent;

  if (this->voltage_sensor_ != nullptr) {
    if (!this->read_byte_16(MAX17043_VCELL, &raw_voltage)) {
      this->status_set_warning("Unable to read MAX17043_VCELL");
    } else {
      float voltage = (1.25 * (float) (raw_voltage >> 4)) / 1000.0;
      this->voltage_sensor_->publish_state(voltage);
      this->status_clear_warning();
    }
  }
  if (this->battery_remaining_sensor_ != nullptr) {
    if (!this->read_byte_16(MAX17043_SOC, &raw_percent)) {
      this->status_set_warning("Unable to read MAX17043_SOC");
    } else {
      float percent = (float) ((raw_percent >> 8) + 0.003906f * (raw_percent & 0x00ff));
      this->battery_remaining_sensor_->publish_state(percent);
      this->status_clear_warning();
    }
  }
}

void LC709203FComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up LC709203F...");

  /*
  uint16_t config_reg;
  if (this->write(&MAX17043_CONFIG, 1) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }

  if (this->read(reinterpret_cast<uint8_t *>(&config_reg), 2) != i2c::ERROR_OK) {
    this->status_set_warning();
    return;
  }

  config_reg = i2c::i2ctohs(config_reg) & MAX17043_CONFIG_SAFE_MASK;
  ESP_LOGV(TAG, "MAX17043 CONFIG register reads 0x%X", config_reg);

  if (config_reg != MAX17043_CONFIG_POWER_UP_DEFAULT) {
    ESP_LOGE(TAG, "Device does not appear to be a MAX17043");
    this->status_set_error("unrecognised");
    this->mark_failed();
    return;
  }

  // need to write back to config register to reset the sleep bit
  if (!this->write_byte_16(MAX17043_CONFIG, MAX17043_CONFIG_POWER_UP_DEFAULT)) {
    this->status_set_error("sleep reset failed");
    this->mark_failed();
    return;
  }*/
}

void LC709203FComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "LC709203F:");
  /*LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with MAX17043 failed");
  }
  LOG_UPDATE_INTERVAL(this);
  LOG_SENSOR("  ", "Battery Voltage", this->voltage_sensor_);
  LOG_SENSOR("  ", "Battery Level", this->battery_remaining_sensor_);*/
}

//float LC709203FComponent::get_setup_priority() const { return setup_priority::DATA; }

void LC709203FComponent::sleep_mode() {
  /*if (!this->is_failed()) {
    if (!this->write_byte_16(MAX17043_CONFIG, MAX17043_CONFIG_POWER_UP_DEFAULT | MAX17043_CONFIG_SLEEP_MASK)) {
      ESP_LOGW(TAG, "Unable to write the sleep bit to config register");
      this->status_set_warning();
    }
  }*/
}

uint16_t LC709203FComponent::ReadRegister(RegisterToRead) {
    uint8_t ReadBuffer[6];
    
    ReadBuffer[0] = LC709203F_I2C_ADDR_DEFAULT;     //TODO: Can this change?
    ReadBuffer[1] = RegisterToRead;
    ReadBuffer[2] = ReadBuffer[0] | 0x01
    
    if (!this->read_bytes(RegisterToRead, &(ReadBuffer[3]), 3))
    {
        this->status_set_warning("Unable to read register");    //TODO: add retry here?
    }
    
    //TODO: Check CRC here.
}
    
uint8_t LC709203FComponent::CRC8(uint8_t *data, int len) {
    const uint8_t POLYNOMIAL(0x07);
    uint8_t crc(0x00);

    for (int j = len; j; --j) 
    {
        crc ^= *data++;

        for (int i = 8; i; --i) 
        {
            crc = (crc & 0x80) ? (crc << 1) ^ POLYNOMIAL : (crc << 1);
        }
    }
  return crc;
}

}  // namespace lc709203f
}  // namespace esphome
