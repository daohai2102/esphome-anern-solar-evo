#include "anern_solar_evo_number.h"
#include "esphome/core/log.h"

namespace esphome {
namespace anern_solar_evo {

static const char *const TAG = "anern_solar_evo.number";

void AnernSolarEvoNumber::dump_config() { LOG_NUMBER("", "AnernSolarEvo Number", this); }

void AnernSolarEvoNumber::control(float value) {
  char buffer[20];
  int len = snprintf(buffer, sizeof(buffer), this->set_command_.c_str(), value);
  if (len > 0) {
    this->parent_->switch_command(std::string(buffer), this->query_command_);
  }
  this->publish_state(value);
}

}  // namespace anern_solar_evo
}  // namespace esphome
