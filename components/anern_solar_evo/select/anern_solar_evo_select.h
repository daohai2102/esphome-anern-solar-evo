#pragma once

#include <map>
#include "../anern_solar_evo.h"
#include "esphome/components/select/select.h"

namespace esphome {
namespace anern_solar_evo {

class AnernSolarEvo;
class AnernSolarEvoSelect : public select::Select, public Component {
 public:
  void set_parent(AnernSolarEvo *parent) { this->parent_ = parent; };
  void set_command(const std::string &command) { this->command_ = command; };
  void set_query_command(const std::string &command) { this->query_command_ = command; };
  void add_option(const std::string &key, const std::string &value);
  void dump_config() override;
  void set_reverse_mapping(std::function<std::string(int)> f) { this->reverse_mapping_ = f; }
  void publish_state_from_value(int value);

 protected:
  void control(const std::string &value) override;

  std::string command_;
  std::string query_command_;
  std::map<std::string, std::string> options_;
  AnernSolarEvo *parent_;
  std::function<std::string(int)> reverse_mapping_{nullptr};
};

}  // namespace anern_solar_evo
}  // namespace esphome
