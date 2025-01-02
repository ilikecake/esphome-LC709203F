#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/i2c/i2c.h"

namespace esphome {
namespace lc709203f {

class LC709203FComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;
  void update() override;
  void sleep_mode();

  void set_voltage_sensor(sensor::Sensor *voltage_sensor) { voltage_sensor_ = voltage_sensor; }
  void set_battery_remaining_sensor(sensor::Sensor *battery_remaining_sensor) {
    battery_remaining_sensor_ = battery_remaining_sensor;
  }
  
  uint16_t ReadRegister(RegisterToRead);
  uint8_t CRC8(uint8_t *data, int len);

 protected:
  sensor::Sensor *voltage_sensor_{nullptr};
  sensor::Sensor *battery_remaining_sensor_{nullptr};
};

}  // namespace lc709203f
}  // namespace esphome
