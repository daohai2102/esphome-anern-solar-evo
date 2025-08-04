#include "anern_solar_output_evo.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"

namespace esphome {
namespace anern_solar_evo {

  static const char *const TAG = "anern_solar_evo.output";

void AnernSolarEvoOutput::write_state(float state) {
  char tmp[10];
  sprintf(tmp, this->set_command_.c_str(), state);

  if (std::find(this->possible_values_.begin(), this->possible_values_.end(), state) != this->possible_values_.end()) {
    ESP_LOGD(TAG, "Will write: %s out of value %f / %02.0f", tmp, state, state);
    this->parent_->switch_command(std::string(tmp));
  } else {
    ESP_LOGD(TAG, "Will not write: %s as it is not in list of allowed values", tmp);
  }
}
}  // namespace anern_solar_evo
}  // namespace esphome
