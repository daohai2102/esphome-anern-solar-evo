#pragma once

#include "../anern_solar_evo.h"
#include "esphome/components/number/number.h"
#include "esphome/core/component.h"

namespace esphome {
namespace anern_solar_evo {

class AnernSolarEvo;

class AnernSolarEvoNumber : public number::Number, public Component {
 public:
  void set_parent(AnernSolarEvo *parent) { this->parent_ = parent; }
  void set_set_command(const std::string &command) { this->set_command_ = command; };
  void dump_config() override;

 protected:
  void control(float value) override;
  std::string set_command_;
  AnernSolarEvo *parent_;
};

}  // namespace anern_solar_evo
}  // namespace esphome
