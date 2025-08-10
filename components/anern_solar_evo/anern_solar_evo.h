#pragma once

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/uart/uart.h"
#include "esphome/core/automation.h"
#include "esphome/core/component.h"

namespace esphome {
namespace anern_solar_evo {

enum ENUMPollingCommand {
  POLLING_QPIRI = 0,
  POLLING_QPIGS = 1,
  POLLING_QMOD = 2,
  POLLING_QFLAG = 3,
  POLLING_QPIWS = 4,
  POLLING_QT = 5,
  POLLING_QMN = 6,
  POLLING_QID = 7,
};
struct PollingCommand {
  uint8_t *command;
  uint8_t length = 0;
  uint8_t errors;
  ENUMPollingCommand identifier;
};

#define ANERN_SOLAR_EVO_VALUED_ENTITY_(type, name, polling_command, value_type) \
 protected: \
  value_type value_##name##_; \
  ANERN_SOLAR_EVO_ENTITY_(type, name, polling_command)

#define ANERN_SOLAR_EVO_ENTITY_(type, name, polling_command) \
 protected: \
  type *name##_{}; /* NOLINT */ \
\
 public: \
  void set_##name(type *name) { /* NOLINT */ \
    this->name##_ = name; \
    this->add_polling_command_(#polling_command, POLLING_##polling_command); \
  }

#define ANERN_SOLAR_EVO_SENSOR(name, polling_command, value_type) \
ANERN_SOLAR_EVO_VALUED_ENTITY_(sensor::Sensor, name, polling_command, value_type)
#define ANERN_SOLAR_EVO_SWITCH(name, polling_command) ANERN_SOLAR_EVO_ENTITY_(switch_::Switch, name, polling_command)
#define ANERN_SOLAR_EVO_BINARY_SENSOR(name, polling_command, value_type) \
ANERN_SOLAR_EVO_VALUED_ENTITY_(binary_sensor::BinarySensor, name, polling_command, value_type)
#define ANERN_SOLAR_EVO_VALUED_TEXT_SENSOR(name, polling_command, value_type) \
ANERN_SOLAR_EVO_VALUED_ENTITY_(text_sensor::TextSensor, name, polling_command, value_type)
#define ANERN_SOLAR_EVO_TEXT_SENSOR(name, polling_command) ANERN_SOLAR_EVO_ENTITY_(text_sensor::TextSensor, name, polling_command)

class AnernSolarEvo : public uart::UARTDevice, public PollingComponent {
  // QPIGS values
  ANERN_SOLAR_EVO_SENSOR(grid_voltage, QPIGS, float)
  ANERN_SOLAR_EVO_SENSOR(grid_frequency, QPIGS, float)
  ANERN_SOLAR_EVO_SENSOR(ac_output_voltage, QPIGS, float)
  ANERN_SOLAR_EVO_SENSOR(ac_output_frequency, QPIGS, float)
  ANERN_SOLAR_EVO_SENSOR(ac_output_apparent_power, QPIGS, int)
  ANERN_SOLAR_EVO_SENSOR(ac_output_active_power, QPIGS, int)
  ANERN_SOLAR_EVO_SENSOR(output_load_percent, QPIGS, int)
  ANERN_SOLAR_EVO_SENSOR(bus_voltage, QPIGS, int)
  ANERN_SOLAR_EVO_SENSOR(battery_voltage, QPIGS, float)
  ANERN_SOLAR_EVO_SENSOR(battery_charging_current, QPIGS, int)
  ANERN_SOLAR_EVO_SENSOR(battery_capacity_percent, QPIGS, int)
  ANERN_SOLAR_EVO_SENSOR(inverter_heat_sink_temperature, QPIGS, int)
  ANERN_SOLAR_EVO_SENSOR(pv_input_current_for_battery, QPIGS, float)
  ANERN_SOLAR_EVO_SENSOR(pv_input_voltage, QPIGS, float)
  ANERN_SOLAR_EVO_SENSOR(battery_voltage_scc, QPIGS, float)
  ANERN_SOLAR_EVO_SENSOR(battery_discharge_current, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(add_sbu_priority_version, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(configuration_status, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(scc_firmware_version, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(load_status, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(battery_voltage_to_steady_while_charging, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(charging_status, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(scc_charging_status, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(ac_charging_status, QPIGS, int)
  ANERN_SOLAR_EVO_SENSOR(battery_voltage_offset_for_fans_on, QPIGS, int)  //.1 scale
  ANERN_SOLAR_EVO_SENSOR(eeprom_version, QPIGS, int)
  ANERN_SOLAR_EVO_SENSOR(pv_charging_power, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(charging_to_floating_mode, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(switch_on, QPIGS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(dustproof_installed, QPIGS, int)

  // QPIRI values
  ANERN_SOLAR_EVO_SENSOR(grid_rating_voltage, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(grid_rating_current, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(ac_output_rating_voltage, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(ac_output_rating_frequency, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(ac_output_rating_current, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(ac_output_rating_apparent_power, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(ac_output_rating_active_power, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(battery_rating_voltage, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(battery_recharge_voltage, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(battery_under_voltage, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(battery_bulk_voltage, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(battery_float_voltage, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(battery_type, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(current_max_ac_charging_current, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(current_max_charging_current, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(input_voltage_range, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(output_source_priority, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(charger_source_priority, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(parallel_max_num, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(machine_type, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(topology, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(output_mode, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(battery_redischarge_voltage, QPIRI, float)
  ANERN_SOLAR_EVO_SENSOR(pv_ok_condition_for_parallel, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(pv_power_balance, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(voltage_point_back_to_utility, QPIRI, float) // need to check
  ANERN_SOLAR_EVO_SENSOR(grid_tie_current, QPIRI, int)
  ANERN_SOLAR_EVO_SENSOR(dual_output_functional_voltage_point, QPIRI, float)


  // QMOD values
  ANERN_SOLAR_EVO_VALUED_TEXT_SENSOR(device_mode, QMOD, char)
  // QID values
  ANERN_SOLAR_EVO_VALUED_TEXT_SENSOR(serial_number, QID, std::string)

  // QFLAG values
  ANERN_SOLAR_EVO_BINARY_SENSOR(silence_buzzer_open_buzzer, QFLAG, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(overload_bypass_function, QFLAG, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(lcd_escape_to_default, QFLAG, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(overload_restart_function, QFLAG, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(over_temperature_restart_function, QFLAG, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(backlight_on, QFLAG, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(alarm_on_when_primary_source_interrupt, QFLAG, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_code_record, QFLAG, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(power_saving, QFLAG, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(data_log_popup, QFLAG, int) // can be alerm control or led light pattern -> need to check
  ANERN_SOLAR_EVO_BINARY_SENSOR(battery_equalization, QFLAG, int) // can be equalization activated immediately -> need to check
  ANERN_SOLAR_EVO_BINARY_SENSOR(dual_output, QFLAG, int) // equalization activated immediately -> need to check

  // QPIWS values
  ANERN_SOLAR_EVO_BINARY_SENSOR(warnings_present, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(faults_present, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_power_loss, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_inverter_fault, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_bus_over, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_bus_under, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_bus_soft_fail, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_line_fail, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_opvshort, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_inverter_voltage_too_low, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_inverter_voltage_too_high, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_over_temperature, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_fan_lock, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_battery_voltage_high, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_battery_low_alarm, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_battery_under_shutdown, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_battery_derating, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_over_load, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_eeprom_failed, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_inverter_over_current, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_inverter_soft_failed, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_self_test_failed, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_op_dc_voltage_over, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_battery_open, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_current_sensor_failed, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_battery_short, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_power_limit, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_pv_voltage_high, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_mppt_overload, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_mppt_overload, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_battery_too_low_to_charge, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_dc_dc_over_current, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(fault_code, QPIWS, int)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warnung_low_pv_energy, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_high_ac_input_during_bus_soft_start, QPIWS, bool)
  ANERN_SOLAR_EVO_BINARY_SENSOR(warning_battery_equalization, QPIWS, bool)

  ANERN_SOLAR_EVO_TEXT_SENSOR(last_qpigs, QPIGS)
  ANERN_SOLAR_EVO_TEXT_SENSOR(last_qpiri, QPIRI)
  ANERN_SOLAR_EVO_TEXT_SENSOR(last_qmod, QMOD)
  ANERN_SOLAR_EVO_TEXT_SENSOR(last_qflag, QFLAG)
  ANERN_SOLAR_EVO_TEXT_SENSOR(last_qpiws, QPIWS)
  ANERN_SOLAR_EVO_TEXT_SENSOR(last_qt, QT)
  ANERN_SOLAR_EVO_TEXT_SENSOR(last_qmn, QMN)
  ANERN_SOLAR_EVO_TEXT_SENSOR(last_qid, QID)

  ANERN_SOLAR_EVO_SWITCH(output_source_priority_utility_switch, QPIRI)
  ANERN_SOLAR_EVO_SWITCH(output_source_priority_solar_switch, QPIRI)
  ANERN_SOLAR_EVO_SWITCH(output_source_priority_battery_switch, QPIRI)
  ANERN_SOLAR_EVO_SWITCH(output_source_priority_hybrid_switch, QPIRI)
  ANERN_SOLAR_EVO_SWITCH(input_voltage_range_switch, QPIRI)
  ANERN_SOLAR_EVO_SWITCH(pv_ok_condition_for_parallel_switch, QPIRI)
  ANERN_SOLAR_EVO_SWITCH(pv_power_balance_switch, QPIRI)

  void switch_command(const std::string &command);
  void setup() override;
  void loop() override;
  void dump_config() override;
  void update() override;

 protected:
  static const size_t ANERN_SOLAR_EVO_READ_BUFFER_LENGTH = 120;  // maximum supported answer length
  static const size_t COMMAND_QUEUE_LENGTH = 10;
  static const size_t COMMAND_TIMEOUT = 5000;
  uint32_t last_poll_ = 0;
  void add_polling_command_(const char *command, ENUMPollingCommand polling_command);
  void empty_uart_buffer_();
  uint8_t check_incoming_crc_();
  uint8_t check_incoming_length_(uint8_t length);
  uint16_t anern_solar_crc__evo(uint8_t *msg, uint8_t len);
  uint8_t send_next_command_();
  void send_next_poll_();
  void queue_command_(const char *command, uint8_t length);
  std::string command_queue_[COMMAND_QUEUE_LENGTH];
  uint8_t command_queue_position_ = 0;
  uint8_t read_buffer_[ANERN_SOLAR_EVO_READ_BUFFER_LENGTH];
  size_t read_pos_{0};

  uint32_t command_start_millis_ = 0;
  uint8_t state_;
  enum State {
    STATE_IDLE = 0,
    STATE_POLL = 1,
    STATE_COMMAND = 2,
    STATE_POLL_COMPLETE = 3,
    STATE_COMMAND_COMPLETE = 4,
    STATE_POLL_CHECKED = 5,
    STATE_POLL_DECODED = 6,
  };

  uint8_t last_polling_command_ = 0;
  PollingCommand used_polling_commands_[15];
};

}  // namespace anern_solar_evo
}  // namespace esphome
