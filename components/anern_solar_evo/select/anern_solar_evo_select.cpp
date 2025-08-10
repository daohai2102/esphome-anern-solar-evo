#include "anern_solar_evo_select.h"
#include "esphome/core/log.h"

namespace esphome {
namespace anern_solar_evo {

static const char *const TAG = "anern_solar_evo.select";

void AnernSolarEvoSelect::control(const std::string &value) {
  for (auto const &[key, val] : this->options_) {
    if (key == value) {
      this->parent_->switch_command(this->command_ + val);
    }
  }
}

void AnernSolarEvoSelect::add_option(const std::string &key, const std::string &value) {
  this->options_[key] = value;
}

void AnernSolarEvoSelect::publish_state_from_value(int value) {
  if (this->reverse_mapping_) {
    this->publish_state(this->reverse_mapping_(value));
  }
}

void AnernSolarEvoSelect::dump_config() {
  LOG_SELECT("", "AnernSolarEvo Select", this);
}

}  // namespace anern_solar_evo
}  // namespace esphome
