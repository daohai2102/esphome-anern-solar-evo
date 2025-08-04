#include "anern_solar_evo_switch.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace anern_solar_evo {

  static const char *const TAG = "anern_solar_evo.switch";

void AnernSolarEvoSwitch::dump_config() { LOG_SWITCH("", "AnernSolarEvo Switch", this); }
void AnernSolarEvoSwitch::write_state(bool state) {
  if (state) {
    if (!this->on_command_.empty()) {
      this->parent_->switch_command(this->on_command_);
    }
  } else {
    if (!this->off_command_.empty()) {
      this->parent_->switch_command(this->off_command_);
    }
  }
}

}  // namespace anern_solar_evo
}  // namespace esphome
