#pragma once

#include "../anern_solar_evo.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace anern_solar_evo {

class AnernSolarEvo;
class AnernSolarEvoSelect : public select::Select, public Component {
 public:
  void set_parent(AnernSolarEvo *parent) { this->parent_ = parent; };
  void set_command(const std::string &command) { this->command_ = command; };
  void add_option(const std::string &key, const std::string &value);
  void dump_config() override;

 protected:
  void control(const std::string &value) override;

  std::string command_;
  std::map<std::string, std::string> options_;
  AnernSolarEvo *parent_;
};

}  // namespace anern_solar_evo
}  // namespace esphome
