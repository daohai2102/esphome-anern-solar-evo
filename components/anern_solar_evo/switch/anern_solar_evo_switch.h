#pragma once

#include "../anern_solar_evo.h"
#include "esphome/components/switch/switch.h"
#include "esphome/core/component.h"

namespace esphome {
namespace anern_solar_evo {
class AnernSolarEvo;
class AnernSolarEvoSwitch : public switch_::Switch, public Component {
 public:
  void set_parent(AnernSolarEvo *parent) { this->parent_ = parent; };
  void set_on_command(const std::string &command) { this->on_command_ = command; };
  void set_off_command(const std::string &command) { this->off_command_ = command; };
  void set_query_command(const std::string &command) { this->query_command_ = command; };
  void dump_config() override;

 protected:
  void write_state(bool state) override;
  std::string on_command_;
  std::string off_command_;
  std::string query_command_;
  AnernSolarEvo *parent_;
};

}  // namespace anern_solar_evo
}  // namespace esphome
