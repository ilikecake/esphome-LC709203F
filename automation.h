
#pragma once
#include "esphome/core/automation.h"
#include "lc709203f.h"

namespace esphome {
namespace lc709203f {

/*template<typename... Ts> class SleepAction : public Action<Ts...> {
 public:
  explicit SleepAction(MAX17043Component *max17043) : max17043_(max17043) {}

  void play(Ts... x) override { this->max17043_->sleep_mode(); }

 protected:
  MAX17043Component *max17043_;
};*/

}  // namespace lc709203f
}  // namespace esphome
