#include "anern_solar_evo.h"
#include "esphome/core/helpers.h"
#include "esphome/core/log.h"
#include "select/anern_solar_evo_select.h"
#include <algorithm> // New include for std::find

namespace esphome {
namespace anern_solar_evo {

static const char *const TAG = "anern_solar_evo";

void AnernSolarEvo::setup() {
  this->state_ = STATE_IDLE;
  this->command_start_millis_ = 0;
}

void AnernSolarEvo::empty_uart_buffer_() {
  uint8_t byte;
  while (this->available()) {
    this->read_byte(&byte);
  }
}

// Start of new helper function
// This function runs once to separate used commands into normal and priority lists
void AnernSolarEvo::initialize_polling_logic_() {
  if (this->polling_logic_initialized_) {
    return;
  }

  // Separate the used commands into normal and priority lists based on YAML config
  for (uint8_t i = 0; i < 15; i++) {
    if (this->used_polling_commands_[i].length > 0) {
      ENUMPollingCommand current_id = this->used_polling_commands_[i].identifier;
      // Check if the current command's ID is in the priority list
      bool is_priority = std::find(this->priority_polling_commands_.begin(), this->priority_polling_commands_.end(),
                                    current_id) != this->priority_polling_commands_.end();
      if (is_priority) {
        this->priority_command_indices_.push_back(i);
      } else {
        this->normal_command_indices_.push_back(i);
      }
    }
  }

  // Edge case: if there are no normal commands, treat priority commands as normal to ensure polling happens
  if (this->normal_command_indices_.empty() && !this->priority_command_indices_.empty()) {
    this->normal_command_indices_ = this->priority_command_indices_;
    this->priority_command_indices_.clear();
  }

  // The first cycle should be a priority cycle if priority commands exist
  this->next_is_priority_cycle_ = !this->priority_command_indices_.empty();
  this->polling_logic_initialized_ = true;
}
// End of new helper function

void AnernSolarEvo::loop() {
  // --- State: STATE_IDLE ---
  // In idle state, we check if we need to send a command from the queue or a scheduled poll.
  if (this->state_ == STATE_IDLE) {
    this->empty_uart_buffer_();
    if (this->send_next_command_() == 0) { // If no command was sent from the queue
      // Time to send a poll command
      if (millis() - this->last_poll_ > this->update_interval_) {
        this->send_next_poll_();
        this->last_poll_ = millis();
      }
    }
    return; // Return and wait for the next loop iteration
  }

  // --- State: STATE_POLL or STATE_COMMAND ---
  // In these states, we are waiting for and reading a response from the inverter.
  if (this->state_ == STATE_POLL || this->state_ == STATE_COMMAND) {
    while (this->available()) {
      // 1. Check for buffer overflow BEFORE writing to the buffer.
      if (this->read_pos_ >= ANERN_SOLAR_EVO_READ_BUFFER_LENGTH) {
        ESP_LOGE(TAG, "Buffer overflow, discarding message. Buffer content: %s", this->read_buffer_);
        this->read_pos_ = 0;
        this->state_ = STATE_IDLE;
        return; // Exit and let the next loop() call handle the idle state
      }

      uint8_t byte;
      this->read_byte(&byte);

      this->read_buffer_[this->read_pos_] = byte;
      this->read_pos_++;

      // 2. Check for the end of the message.
      if (byte == 0x0D) {
        this->read_buffer_[this->read_pos_ - 1] = '\0'; // Null-terminate the string, replacing the \r

        if (this->state_ == STATE_POLL) {
          this->state_ = STATE_POLL_COMPLETE;
        } else { // STATE_COMMAND
          this->state_ = STATE_COMMAND_COMPLETE;
        }
        break; // Exit the while loop, we have a complete message.
      }
    }

    // 3. Timeout Check (if still waiting for a response)
    if (this->state_ == STATE_POLL || this->state_ == STATE_COMMAND) {
      if (millis() - this->command_start_millis_ > esphome::anern_solar_evo::AnernSolarEvo::COMMAND_TIMEOUT) {
        if (this->state_ == STATE_POLL) {
          ESP_LOGW(TAG, "Timeout waiting for poll response for: %s", (char *)this->used_polling_commands_[this->last_polling_command_].command);
        } else { // STATE_COMMAND
          ESP_LOGW(TAG, "Timeout waiting for command response for: %s", this->command_queue_[this->command_queue_position_].c_str());
          this->command_queue_[this->command_queue_position_] = "";
          this->command_queue_position_ = (this->command_queue_position_ + 1) % COMMAND_QUEUE_LENGTH;
        }
        this->state_ = STATE_IDLE;
      }
    }
  }

  // --- State: Process complete messages ---
  // The rest of your state machine for processing received data goes here.
  // This logic is separated from the reading loop.

  if (this->state_ == STATE_COMMAND_COMPLETE) {
      if (this->check_incoming_crc_()) {
        if (this->read_buffer_[1] == 'A' && this->read_buffer_[2] == 'C' && this->read_buffer_[3] == 'K') {
          ESP_LOGD(TAG, "Command successful: %s", this->read_buffer_);
        } else {
          ESP_LOGW(TAG, "Command failed (NAK): %s", this->read_buffer_);
        }
      } else {
        ESP_LOGW(TAG, "Received command response with invalid CRC.");
      }
      this->command_queue_[this->command_queue_position_] = "";
      this->command_queue_position_ = (this->command_queue_position_ + 1) % COMMAND_QUEUE_LENGTH;
      this->state_ = STATE_IDLE;
  }

  if (this->state_ == STATE_POLL_COMPLETE) {
      if (this->read_buffer_[0] == '(' && this->read_buffer_[1] == 'N' && this->read_buffer_[2] == 'A' && this->read_buffer_[3] == 'K') {
          ESP_LOGW(TAG, "Poll command failed (NAK)");
          this->state_ = STATE_IDLE;
          return;
      }
      if (this->check_incoming_crc_()) {
          this->state_ = STATE_POLL_CHECKED;
      } else {
          ESP_LOGW(TAG, "Received poll response with invalid CRC.");
          this->state_ = STATE_IDLE;
      }
  }

  if (this->state_ == STATE_POLL_DECODED) {
    std::string mode;
    switch (this->used_polling_commands_[this->last_polling_command_].identifier) {
      case POLLING_QPIRI:
        if (this->grid_rating_voltage_) {
          this->grid_rating_voltage_->publish_state(value_grid_rating_voltage_);
        }
        if (this->grid_rating_current_) {
          this->grid_rating_current_->publish_state(value_grid_rating_current_);
        }
        if (this->ac_output_rating_voltage_) {
          this->ac_output_rating_voltage_->publish_state(value_ac_output_rating_voltage_);
        }
        if (this->ac_output_rating_frequency_) {
          this->ac_output_rating_frequency_->publish_state(value_ac_output_rating_frequency_);
        }
        if (this->ac_output_rating_current_) {
          this->ac_output_rating_current_->publish_state(value_ac_output_rating_current_);
        }
        if (this->ac_output_rating_apparent_power_) {
          this->ac_output_rating_apparent_power_->publish_state(value_ac_output_rating_apparent_power_);
        }
        if (this->ac_output_rating_active_power_) {
          this->ac_output_rating_active_power_->publish_state(value_ac_output_rating_active_power_);
        }
        if (this->battery_rating_voltage_) {
          this->battery_rating_voltage_->publish_state(value_battery_rating_voltage_);
        }
        if (this->battery_recharge_voltage_) {
          this->battery_recharge_voltage_->publish_state(value_battery_recharge_voltage_);
        }
        if (this->battery_under_voltage_) {
          this->battery_under_voltage_->publish_state(value_battery_under_voltage_);
        }
        if (this->battery_bulk_voltage_) {
          this->battery_bulk_voltage_->publish_state(value_battery_bulk_voltage_);
        }
        if (this->battery_float_voltage_) {
          this->battery_float_voltage_->publish_state(value_battery_float_voltage_);
        }
        if (this->battery_type_) {
          static_cast<AnernSolarEvoSelect *>(this->battery_type_)->publish_state_from_value(value_battery_type_);
        }
        if (this->current_max_ac_charging_current_) {
          this->current_max_ac_charging_current_->publish_state(value_current_max_ac_charging_current_);
        }
        if (this->current_max_charging_current_) {
          this->current_max_charging_current_->publish_state(value_current_max_charging_current_);
        }
        if (this->input_voltage_range_) {
          static_cast<AnernSolarEvoSelect *>(this->input_voltage_range_)->publish_state_from_value(value_input_voltage_range_);
        }
        if (this->output_source_priority_) {
          static_cast<AnernSolarEvoSelect *>(this->output_source_priority_)->publish_state_from_value(value_output_source_priority_);
        }
        if (this->charger_source_priority_) {
          static_cast<AnernSolarEvoSelect *>(this->charger_source_priority_)->publish_state_from_value(value_charger_source_priority_);
        }
        if (this->parallel_max_num_) {
          this->parallel_max_num_->publish_state(value_parallel_max_num_);
        }
        if (this->machine_type_) {
          this->machine_type_->publish_state(value_machine_type_);
        }
        if (this->topology_) {
          this->topology_->publish_state(value_topology_);
        }
        if (this->output_mode_) {
          this->output_mode_->publish_state(value_output_mode_);
        }
        if (this->battery_redischarge_voltage_) {
          this->battery_redischarge_voltage_->publish_state(value_battery_redischarge_voltage_);
        }
        if (this->pv_ok_condition_for_parallel_) {
          static_cast<AnernSolarEvoSelect *>(this->pv_ok_condition_for_parallel_)->publish_state_from_value(value_pv_ok_condition_for_parallel_);
        }
        if (this->pv_power_balance_) {
          static_cast<AnernSolarEvoSelect *>(this->pv_power_balance_)->publish_state_from_value(value_pv_power_balance_);
        }
        if (this->voltage_point_back_to_utility_) {
          this->voltage_point_back_to_utility_->publish_state(value_voltage_point_back_to_utility_);
        }
        if (this->grid_tie_current_) {
          this->grid_tie_current_->publish_state(value_grid_tie_current_);
        }
        if (this->dual_output_functional_voltage_point_) {
          this->dual_output_functional_voltage_point_->publish_state(value_dual_output_functional_voltage_point_);
        }
        this->state_ = STATE_IDLE;
        break;
      case POLLING_QPIGS:
        if (this->grid_voltage_) {
          this->grid_voltage_->publish_state(value_grid_voltage_);
        }
        if (this->grid_frequency_) {
          this->grid_frequency_->publish_state(value_grid_frequency_);
        }
        if (this->ac_output_voltage_) {
          this->ac_output_voltage_->publish_state(value_ac_output_voltage_);
        }
        if (this->ac_output_frequency_) {
          this->ac_output_frequency_->publish_state(value_ac_output_frequency_);
        }
        if (this->ac_output_apparent_power_) {
          this->ac_output_apparent_power_->publish_state(value_ac_output_apparent_power_);
        }
        if (this->ac_output_active_power_) {
          this->ac_output_active_power_->publish_state(value_ac_output_active_power_);
        }
        if (this->output_load_percent_) {
          this->output_load_percent_->publish_state(value_output_load_percent_);
        }
        if (this->bus_voltage_) {
          this->bus_voltage_->publish_state(value_bus_voltage_);
        }
        if (this->battery_voltage_) {
          this->battery_voltage_->publish_state(value_battery_voltage_);
        }
        if (this->battery_charging_current_) {
          this->battery_charging_current_->publish_state(value_battery_charging_current_);
        }
        if (this->battery_capacity_percent_) {
          this->battery_capacity_percent_->publish_state(value_battery_capacity_percent_);
        }
        if (this->inverter_heat_sink_temperature_) {
          this->inverter_heat_sink_temperature_->publish_state(value_inverter_heat_sink_temperature_);
        }
        if (this->pv_input_current_for_battery_) {
          this->pv_input_current_for_battery_->publish_state(value_pv_input_current_for_battery_);
        }
        if (this->pv_input_voltage_) {
          this->pv_input_voltage_->publish_state(value_pv_input_voltage_);
        }
        if (this->battery_voltage_scc_) {
          this->battery_voltage_scc_->publish_state(value_battery_voltage_scc_);
        }
        if (this->battery_discharge_current_) {
          this->battery_discharge_current_->publish_state(value_battery_discharge_current_);
        }
        if (this->add_sbu_priority_version_) {
          this->add_sbu_priority_version_->publish_state(value_add_sbu_priority_version_);
        }
        if (this->configuration_status_) {
          this->configuration_status_->publish_state(value_configuration_status_);
        }
        if (this->scc_firmware_version_) {
          this->scc_firmware_version_->publish_state(value_scc_firmware_version_);
        }
        if (this->load_status_) {
          this->load_status_->publish_state(value_load_status_);
        }
        if (this->battery_voltage_to_steady_while_charging_) {
          this->battery_voltage_to_steady_while_charging_->publish_state(
              value_battery_voltage_to_steady_while_charging_);
        }
        if (this->charging_status_) {
          this->charging_status_->publish_state(value_charging_status_);
        }
        if (this->scc_charging_status_) {
          this->scc_charging_status_->publish_state(value_scc_charging_status_);
        }
        if (this->ac_charging_status_) {
          this->ac_charging_status_->publish_state(value_ac_charging_status_);
        }
        if (this->battery_voltage_offset_for_fans_on_) {
          this->battery_voltage_offset_for_fans_on_->publish_state(value_battery_voltage_offset_for_fans_on_ / 10.0f);
        }  //.1 scale
        if (this->eeprom_version_) {
          this->eeprom_version_->publish_state(value_eeprom_version_);
        }
        if (this->pv_charging_power_) {
          this->pv_charging_power_->publish_state(value_pv_charging_power_);
        }
        if (this->charging_to_floating_mode_) {
          this->charging_to_floating_mode_->publish_state(value_charging_to_floating_mode_);
        }
        if (this->switch_on_) {
          this->switch_on_->publish_state(value_switch_on_);
        }
        if (this->dustproof_installed_) {
          this->dustproof_installed_->publish_state(value_dustproof_installed_);
        }
        // Calculated sensors
        if (this->battery_charging_power_) {
          int battery_charging_power = value_battery_charging_current_ * value_battery_voltage_;
          this->battery_charging_power_->publish_state(battery_charging_power);
        }
        if (this->battery_discharge_power_) {
          int battery_discharge_power = value_battery_discharge_current_ * value_battery_voltage_;
          this->battery_discharge_power_->publish_state(battery_discharge_power);
        }
        if (this->grid_power_) {
          // Add 50 as the average power of the inverter itself
          int grid_power = 50 + value_ac_output_apparent_power_ + (value_battery_charging_current_ * value_battery_voltage_) - value_pv_charging_power_ - (value_battery_discharge_current_ * value_battery_voltage_);
          if (grid_power < 0) {
            grid_power = 0;
          }
          this->grid_power_->publish_state(grid_power);
        }
        this->state_ = STATE_IDLE;
        break;
      case POLLING_QMOD:
        if (this->device_mode_) {
          mode = value_device_mode_;
          this->device_mode_->publish_state(mode);
        }
        this->state_ = STATE_IDLE;
        break;
      case POLLING_QFLAG:
        if (this->silence_buzzer_open_buzzer_) {
          this->silence_buzzer_open_buzzer_->publish_state(value_silence_buzzer_open_buzzer_);
        }
        if (this->overload_bypass_function_) {
          this->overload_bypass_function_->publish_state(value_overload_bypass_function_);
        }
        if (this->lcd_escape_to_default_) {
          this->lcd_escape_to_default_->publish_state(value_lcd_escape_to_default_);
        }
        if (this->overload_restart_function_) {
          this->overload_restart_function_->publish_state(value_overload_restart_function_);
        }
        if (this->over_temperature_restart_function_) {
          this->over_temperature_restart_function_->publish_state(value_over_temperature_restart_function_);
        }
        if (this->backlight_on_) {
          this->backlight_on_->publish_state(value_backlight_on_);
        }
        if (this->alarm_on_when_primary_source_interrupt_) {
          this->alarm_on_when_primary_source_interrupt_->publish_state(value_alarm_on_when_primary_source_interrupt_);
        }
        if (this->fault_code_record_) {
          this->fault_code_record_->publish_state(value_fault_code_record_);
        }
        if (this->power_saving_) {
          this->power_saving_->publish_state(value_power_saving_);
        }
        this->state_ = STATE_IDLE;
        break;
      case POLLING_QPIWS:
        if (this->warnings_present_) {
          this->warnings_present_->publish_state(value_warnings_present_);
        }
        if (this->faults_present_) {
          this->faults_present_->publish_state(value_faults_present_);
        }
        if (this->warning_power_loss_) {
          this->warning_power_loss_->publish_state(value_warning_power_loss_);
        }
        if (this->fault_inverter_fault_) {
          this->fault_inverter_fault_->publish_state(value_fault_inverter_fault_);
        }
        if (this->fault_bus_over_) {
          this->fault_bus_over_->publish_state(value_fault_bus_over_);
        }
        if (this->fault_bus_under_) {
          this->fault_bus_under_->publish_state(value_fault_bus_under_);
        }
        if (this->fault_bus_soft_fail_) {
          this->fault_bus_soft_fail_->publish_state(value_fault_bus_soft_fail_);
        }
        if (this->warning_line_fail_) {
          this->warning_line_fail_->publish_state(value_warning_line_fail_);
        }
        if (this->fault_opvshort_) {
          this->fault_opvshort_->publish_state(value_fault_opvshort_);
        }
        if (this->fault_inverter_voltage_too_low_) {
          this->fault_inverter_voltage_too_low_->publish_state(value_fault_inverter_voltage_too_low_);
        }
        if (this->fault_inverter_voltage_too_high_) {
          this->fault_inverter_voltage_too_high_->publish_state(value_fault_inverter_voltage_too_high_);
        }
        if (this->warning_over_temperature_) {
          this->warning_over_temperature_->publish_state(value_warning_over_temperature_);
        }
        if (this->warning_fan_lock_) {
          this->warning_fan_lock_->publish_state(value_warning_fan_lock_);
        }
        if (this->warning_battery_voltage_high_) {
          this->warning_battery_voltage_high_->publish_state(value_warning_battery_voltage_high_);
        }
        if (this->warning_battery_low_alarm_) {
          this->warning_battery_low_alarm_->publish_state(value_warning_battery_low_alarm_);
        }
        if (this->warning_battery_under_shutdown_) {
          this->warning_battery_under_shutdown_->publish_state(value_warning_battery_under_shutdown_);
        }
        if (this->warning_battery_derating_) {
          this->warning_battery_derating_->publish_state(value_warning_battery_derating_);
        }
        if (this->warning_over_load_) {
          this->warning_over_load_->publish_state(value_warning_over_load_);
        }
        if (this->warning_eeprom_failed_) {
          this->warning_eeprom_failed_->publish_state(value_warning_eeprom_failed_);
        }
        if (this->fault_inverter_over_current_) {
          this->fault_inverter_over_current_->publish_state(value_fault_inverter_over_current_);
        }
        if (this->fault_inverter_soft_failed_) {
          this->fault_inverter_soft_failed_->publish_state(value_fault_inverter_soft_failed_);
        }
        if (this->fault_self_test_failed_) {
          this->fault_self_test_failed_->publish_state(value_fault_self_test_failed_);
        }
        if (this->fault_op_dc_voltage_over_) {
          this->fault_op_dc_voltage_over_->publish_state(value_fault_op_dc_voltage_over_);
        }
        if (this->fault_battery_open_) {
          this->fault_battery_open_->publish_state(value_fault_battery_open_);
        }
        if (this->fault_current_sensor_failed_) {
          this->fault_current_sensor_failed_->publish_state(value_fault_current_sensor_failed_);
        }
        if (this->fault_battery_short_) {
          this->fault_battery_short_->publish_state(value_fault_battery_short_);
        }
        if (this->warning_power_limit_) {
          this->warning_power_limit_->publish_state(value_warning_power_limit_);
        }
        if (this->warning_pv_voltage_high_) {
          this->warning_pv_voltage_high_->publish_state(value_warning_pv_voltage_high_);
        }
        if (this->fault_mppt_overload_) {
          this->fault_mppt_overload_->publish_state(value_fault_mppt_overload_);
        }
        if (this->warning_mppt_overload_) {
          this->warning_mppt_overload_->publish_state(value_warning_mppt_overload_);
        }
        if (this->warning_battery_too_low_to_charge_) {
          this->warning_battery_too_low_to_charge_->publish_state(value_warning_battery_too_low_to_charge_);
        }
        if (this->fault_dc_dc_over_current_) {
          this->fault_dc_dc_over_current_->publish_state(value_fault_dc_dc_over_current_);
        }
        if (this->fault_code_) {
          this->fault_code_->publish_state(value_fault_code_);
        }
        if (this->warning_low_pv_energy_) {
          this->warning_low_pv_energy_->publish_state(value_warning_low_pv_energy_);
        }
        if (this->warning_high_ac_input_during_bus_soft_start_) {
          this->warning_high_ac_input_during_bus_soft_start_->publish_state(
              value_warning_high_ac_input_during_bus_soft_start_);
        }
        if (this->warning_battery_equalization_) {
          this->warning_battery_equalization_->publish_state(value_warning_battery_equalization_);
        }
        this->state_ = STATE_IDLE;
        break;
      case POLLING_QT:
      case POLLING_QMN:
        this->state_ = STATE_IDLE;
        break;
      case POLLING_QID:
        if (this->serial_number_) {
          this->serial_number_->publish_state(this->value_serial_number_);
        }
        this->state_ = STATE_IDLE;
        break;
    }
  }

  if (this->state_ == STATE_POLL_CHECKED) {
    bool enabled = true;
    std::string fc;
    char tmp[ANERN_SOLAR_EVO_READ_BUFFER_LENGTH];
    sprintf(tmp, "%s", this->read_buffer_);
    switch (this->used_polling_commands_[this->last_polling_command_].identifier) {
      case POLLING_QPIRI:
        ESP_LOGD(TAG, "Decode QPIRI");
        sscanf(tmp, "(%f %f %f %f %f %d %d %f %f %f %f %f %d %d %d %d %d %d %d %d %d %d %f %d %d %f %d %f",          // NOLINT
               &value_grid_rating_voltage_, &value_grid_rating_current_, &value_ac_output_rating_voltage_,  // NOLINT
               &value_ac_output_rating_frequency_, &value_ac_output_rating_current_,                        // NOLINT
               &value_ac_output_rating_apparent_power_, &value_ac_output_rating_active_power_,              // NOLINT
               &value_battery_rating_voltage_, &value_battery_recharge_voltage_,                            // NOLINT
               &value_battery_under_voltage_, &value_battery_bulk_voltage_, &value_battery_float_voltage_,  // NOLINT
               &value_battery_type_, &value_current_max_ac_charging_current_,                               // NOLINT
               &value_current_max_charging_current_, &value_input_voltage_range_,                           // NOLINT
               &value_output_source_priority_, &value_charger_source_priority_, &value_parallel_max_num_,   // NOLINT
               &value_machine_type_, &value_topology_, &value_output_mode_,                                 // NOLINT
               &value_battery_redischarge_voltage_, &value_pv_ok_condition_for_parallel_,                   // NOLINT
               &value_pv_power_balance_, &value_voltage_point_back_to_utility_,
               &value_grid_tie_current_, &value_dual_output_functional_voltage_point_);                                                                   // NOLINT
        if (this->last_qpiri_) {
          this->last_qpiri_->publish_state(tmp);
        }
        this->state_ = STATE_POLL_DECODED;
        break;
      case POLLING_QPIGS:
        ESP_LOGD(TAG, "Decode QPIGS");
        sscanf(                                                                                              // NOLINT
            tmp,                                                                                             // NOLINT
            "(%f %f %f %f %d %d %d %d %f %d %d %d %f %f %f %d %1d%1d%1d%1d%1d%1d%1d%1d %d %d %d %1d%1d%1d",  // NOLINT
            &value_grid_voltage_, &value_grid_frequency_, &value_ac_output_voltage_,                         // NOLINT
            &value_ac_output_frequency_,                                                                     // NOLINT
            &value_ac_output_apparent_power_, &value_ac_output_active_power_, &value_output_load_percent_,   // NOLINT
            &value_bus_voltage_, &value_battery_voltage_, &value_battery_charging_current_,                  // NOLINT
            &value_battery_capacity_percent_, &value_inverter_heat_sink_temperature_,                        // NOLINT
            &value_pv_input_current_for_battery_, &value_pv_input_voltage_, &value_battery_voltage_scc_,     // NOLINT
            &value_battery_discharge_current_, &value_add_sbu_priority_version_,                             // NOLINT
            &value_configuration_status_, &value_scc_firmware_version_, &value_load_status_,                 // NOLINT
            &value_battery_voltage_to_steady_while_charging_, &value_charging_status_,                       // NOLINT
            &value_scc_charging_status_, &value_ac_charging_status_,                                         // NOLINT
            &value_battery_voltage_offset_for_fans_on_, &value_eeprom_version_, &value_pv_charging_power_,   // NOLINT
            &value_charging_to_floating_mode_, &value_switch_on_,                                            // NOLINT
            &value_dustproof_installed_);                                                                    // NOLINT
        if (this->last_qpigs_) {
          this->last_qpigs_->publish_state(tmp);
        }
        this->state_ = STATE_POLL_DECODED;
        break;
      case POLLING_QMOD:
        ESP_LOGD(TAG, "Decode QMOD");
        this->value_device_mode_ = char(this->read_buffer_[1]);
        if (this->last_qmod_) {
          this->last_qmod_->publish_state(tmp);
        }
        this->state_ = STATE_POLL_DECODED;
        break;
      case POLLING_QFLAG:
        ESP_LOGD(TAG, "Decode QFLAG");
        // result like:"(EbkuvxzDajy"
        // get through all char: ignore first "(" Enable flag on 'E', Disable on 'D') else set the corresponding value
        for (size_t i = 1; i < strlen(tmp); i++) {
          switch (tmp[i]) {
            case 'E':
              enabled = true;
              break;
            case 'D':
              enabled = false;
              break;
            case 'a':
              this->value_silence_buzzer_open_buzzer_ = enabled;
              break;
            case 'b':
              this->value_overload_bypass_function_ = enabled;
              break;
            case 'k':
              this->value_lcd_escape_to_default_ = enabled;
              break;
            case 'u':
              this->value_overload_restart_function_ = enabled;
              break;
            case 'v':
              this->value_over_temperature_restart_function_ = enabled;
              break;
            case 'x':
              this->value_backlight_on_ = enabled;
              break;
            case 'y':
              this->value_alarm_on_when_primary_source_interrupt_ = enabled;
              break;
            case 'z':
              this->value_fault_code_record_ = enabled;
              break;
            case 'j':
              this->value_power_saving_ = enabled;
              break;
          }
        }
        if (this->last_qflag_) {
          this->last_qflag_->publish_state(tmp);
        }
        this->state_ = STATE_POLL_DECODED;
        break;
      case POLLING_QPIWS:
        ESP_LOGD(TAG, "Decode QPIWS");
        // '(00000000000000000000000000000000'
        // iterate over all available flag (as not all models have all flags, but at least in the same order)
        this->value_warnings_present_ = false;
        this->value_faults_present_ = false;

        for (size_t i = 1; i < strlen(tmp); i++) {
          enabled = tmp[i] == '1';
          switch (i) {
            case 1:
              this->value_warning_power_loss_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 2:
              this->value_fault_inverter_fault_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 3:
              this->value_fault_bus_over_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 4:
              this->value_fault_bus_under_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 5:
              this->value_fault_bus_soft_fail_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 6:
              this->value_warning_line_fail_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 7:
              this->value_fault_opvshort_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 8:
              this->value_fault_inverter_voltage_too_low_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 9:
              this->value_fault_inverter_voltage_too_high_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 10:
              this->value_warning_over_temperature_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 11:
              this->value_warning_fan_lock_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 12:
              this->value_warning_battery_voltage_high_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 13:
              this->value_warning_battery_low_alarm_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 15:
              this->value_warning_battery_under_shutdown_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 16:
              this->value_warning_battery_derating_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 17:
              this->value_warning_over_load_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 18:
              this->value_warning_eeprom_failed_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 19:
              this->value_fault_inverter_over_current_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 20:
              this->value_fault_inverter_soft_failed_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 21:
              this->value_fault_self_test_failed_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 22:
              this->value_fault_op_dc_voltage_over_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 23:
              this->value_fault_battery_open_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 24:
              this->value_fault_current_sensor_failed_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 25:
              this->value_fault_battery_short_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 26:
              this->value_warning_power_limit_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 27:
              this->value_warning_pv_voltage_high_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 28:
              this->value_fault_mppt_overload_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 29:
              this->value_warning_mppt_overload_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 30:
              this->value_warning_battery_too_low_to_charge_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 31:
              this->value_fault_dc_dc_over_current_ = enabled;
              this->value_faults_present_ += enabled;
              break;
            case 32:
              fc = tmp[i];
              fc += tmp[i + 1];
              this->value_fault_code_ = parse_number<int>(fc).value_or(0);
              break;
            case 34:
              this->value_warning_low_pv_energy_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 35:
              this->value_warning_high_ac_input_during_bus_soft_start_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
            case 36:
              this->value_warning_battery_equalization_ = enabled;
              this->value_warnings_present_ += enabled;
              break;
          }
        }
        if (this->last_qpiws_) {
          this->last_qpiws_->publish_state(tmp);
        }
        this->state_ = STATE_POLL_DECODED;
        break;
      case POLLING_QT:
        ESP_LOGD(TAG, "Decode QT");
        if (this->last_qt_) {
          this->last_qt_->publish_state(tmp);
        }
        this->state_ = STATE_POLL_DECODED;
        break;
      case POLLING_QMN:
        ESP_LOGD(TAG, "Decode QMN");
        if (this->last_qmn_) {
          this->last_qmn_->publish_state(tmp);
        }
        this->state_ = STATE_POLL_DECODED;
        break;
      case POLLING_QID:
        ESP_LOGD(TAG, "Decode QID");

        // Create a std::string from the raw buffer, starting from the second character ('(' is at index 0)
        // and taking a number of characters equal to the total length minus 4 (1 for '(' and 3 for the end bytes).
        this->value_serial_number_ = std::string(tmp + 1, this->read_pos_ - 4);

        // For debugging, publish the full raw response if the last_qid sensor is configured.
        if (this->last_qid_) {
          this->last_qid_->publish_state(tmp);
        }
        this->state_ = STATE_POLL_DECODED;
        break;
      default:
        this->state_ = STATE_IDLE;
        break;
    }
    return;
  }

  if (this->state_ == STATE_COMMAND) {
    if (millis() - this->command_start_millis_ > esphome::anern_solar_evo::AnernSolarEvo::COMMAND_TIMEOUT) {
      // command timeout
      const char *command = this->command_queue_[this->command_queue_position_].c_str();
      this->command_start_millis_ = millis();
      ESP_LOGD(TAG, "timeout command from queue: %s", command);
      this->command_queue_[this->command_queue_position_] = std::string("");
      this->command_queue_position_ = (command_queue_position_ + 1) % COMMAND_QUEUE_LENGTH;
      this->state_ = STATE_IDLE;
      return;
    } else {
    }
  }
  if (this->state_ == STATE_POLL) {
    if (millis() - this->command_start_millis_ > esphome::anern_solar_evo::AnernSolarEvo::COMMAND_TIMEOUT) {
      // command timeout
      ESP_LOGD(TAG, "timeout command to poll: %s", this->used_polling_commands_[this->last_polling_command_].command);
      this->state_ = STATE_IDLE;
    } else {
    }
  }
}

uint8_t AnernSolarEvo::check_incoming_length_(uint8_t length) {
  if (this->read_pos_ - 3 == length) {
    return 1;
  }
  return 0;
}

uint8_t AnernSolarEvo::check_incoming_crc_() {
  uint16_t crc16;
  crc16 = this->anern_solar_crc__evo(read_buffer_, read_pos_ - 3);
  ESP_LOGD(TAG, "checking crc on incoming message");
  if (((uint8_t) ((crc16) >> 8)) == read_buffer_[read_pos_ - 3] &&
      ((uint8_t) ((crc16) &0xff)) == read_buffer_[read_pos_ - 2]) {
    ESP_LOGD(TAG, "CRC OK");
    read_buffer_[read_pos_ - 1] = 0;
    read_buffer_[read_pos_ - 2] = 0;
    read_buffer_[read_pos_ - 3] = 0;
    return 1;
  }
  ESP_LOGD(TAG, "CRC NOK expected: %X %X but got: %X %X", ((uint8_t) ((crc16) >> 8)), ((uint8_t) ((crc16) &0xff)),
           read_buffer_[read_pos_ - 3], read_buffer_[read_pos_ - 2]);
  return 0;
}

// send next command used
uint8_t AnernSolarEvo::send_next_command_() {
  uint16_t crc16;
  if (!this->command_queue_[this->command_queue_position_].empty()) {
    const char *command = this->command_queue_[this->command_queue_position_].c_str();
    uint8_t byte_command[16];
    uint8_t length = this->command_queue_[this->command_queue_position_].length();
    for (uint8_t i = 0; i < length; i++) {
      byte_command[i] = (uint8_t) this->command_queue_[this->command_queue_position_].at(i);
    }
    this->state_ = STATE_COMMAND;
    this->command_start_millis_ = millis();
    this->empty_uart_buffer_();
    this->read_pos_ = 0;
    crc16 = this->anern_solar_crc__evo(byte_command, length);
    this->write_str(command);
    // checksum
    this->write(((uint8_t) ((crc16) >> 8)));   // highbyte
    this->write(((uint8_t) ((crc16) &0xff)));  // lowbyte
    // end Byte
    this->write(0x0D);
    ESP_LOGD(TAG, "Sending command from queue: %s with length %d", command, length);
    return 1;
  }
  return 0;
}

// Start of completely replaced function
void AnernSolarEvo::send_next_poll_() {
  this->initialize_polling_logic_();

  uint8_t command_index_to_send = 0xFF; // 0xFF means no command to send

  if (this->normal_command_indices_.empty() && this->priority_command_indices_.empty()) {
    return; // No polling commands configured at all
  }

  // If there are no priority commands, just cycle through the normal ones
  if (this->priority_command_indices_.empty()) {
    if (!this->normal_command_indices_.empty()) {
      command_index_to_send = this->normal_command_indices_[this->normal_poll_cursor_];
      this->normal_poll_cursor_ = (this->normal_poll_cursor_ + 1) % this->normal_command_indices_.size();
    }
  } else { // Advanced logic with priority commands
    if (this->next_is_priority_cycle_) {
      command_index_to_send = this->priority_command_indices_[this->priority_poll_cursor_];
      this->priority_poll_cursor_++;

      // If we've finished a full priority cycle, the next one is a normal command
      if (this->priority_poll_cursor_ >= this->priority_command_indices_.size()) {
        this->priority_poll_cursor_ = 0;
        this->next_is_priority_cycle_ = false;
      }
    } else {
      if (!this->normal_command_indices_.empty()) {
        command_index_to_send = this->normal_command_indices_[this->normal_poll_cursor_];
        this->normal_poll_cursor_ = (this->normal_poll_cursor_ + 1) % this->normal_command_indices_.size();
      }
      // After sending a normal command, the next cycle is a priority one
      this->next_is_priority_cycle_ = true;
    }
  }

  if (command_index_to_send == 0xFF) {
    return;
  }

  // The rest of this function sends the selected command over UART
  uint16_t crc16;
  this->last_polling_command_ = command_index_to_send;
  if (this->used_polling_commands_[this->last_polling_command_].length == 0) {
    return;
  }
  this->state_ = STATE_POLL;
  this->command_start_millis_ = millis();
  this->empty_uart_buffer_();
  this->read_pos_ = 0;
  crc16 = this->anern_solar_crc__evo(this->used_polling_commands_[this->last_polling_command_].command,
                                     this->used_polling_commands_[this->last_polling_command_].length);
  this->write_array(this->used_polling_commands_[this->last_polling_command_].command,
                    this->used_polling_commands_[this->last_polling_command_].length);
  // checksum
  this->write(((uint8_t)((crc16) >> 8)));   // highbyte
  this->write(((uint8_t)((crc16) & 0xff)));  // lowbyte
  // end Byte
  this->write(0x0D);
  ESP_LOGD(TAG, "Sending polling command : %s with length %d",
           this->used_polling_commands_[this->last_polling_command_].command,
           this->used_polling_commands_[this->last_polling_command_].length);
}
// End of completely replaced function

void AnernSolarEvo::queue_command_(const char *command, uint8_t length) {
  uint8_t next_position = command_queue_position_;
  for (uint8_t i = 0; i < COMMAND_QUEUE_LENGTH; i++) {
    uint8_t testposition = (next_position + i) % COMMAND_QUEUE_LENGTH;
    if (command_queue_[testposition].empty()) {
      command_queue_[testposition] = command;
      ESP_LOGD(TAG, "Command queued successfully: %s with length %u at position %d", command,
               command_queue_[testposition].length(), testposition);
      return;
    }
  }
  ESP_LOGD(TAG, "Command queue full dropping command: %s", command);
}

void AnernSolarEvo::switch_command(const std::string &command) {
  ESP_LOGD(TAG, "got command: %s", command.c_str());
  queue_command_(command.c_str(), command.length());
}
void AnernSolarEvo::dump_config() {
  ESP_LOGCONFIG(TAG, "AnernSolarEvo:\n"
                     "used commands:");
  for (auto &used_polling_command : this->used_polling_commands_) {
    if (used_polling_command.length != 0) {
      ESP_LOGCONFIG(TAG, "%s", used_polling_command.command);
    }
  }
}
void AnernSolarEvo::update() {}

void AnernSolarEvo::add_polling_command_(const char *command, ENUMPollingCommand polling_command) {
  for (auto &used_polling_command : this->used_polling_commands_) {
    if (used_polling_command.length == strlen(command)) {
      uint8_t len = strlen(command);
      if (memcmp(used_polling_command.command, command, len) == 0) {
        return;
      }
    }
    if (used_polling_command.length == 0) {
      size_t length = strlen(command) + 1;
      const char *beg = command;
      const char *end = command + length;
      used_polling_command.command = new uint8_t[length];  // NOLINT(cppcoreguidelines-owning-memory)
      size_t i = 0;
      for (; beg != end; ++beg, ++i) {
        used_polling_command.command[i] = (uint8_t) (*beg);
      }
      used_polling_command.errors = 0;
      used_polling_command.identifier = polling_command;
      used_polling_command.length = length - 1;
      return;
    }
  }
}

uint16_t AnernSolarEvo::anern_solar_crc__evo(uint8_t *msg, uint8_t len) {
  uint16_t crc = crc16be(msg, len);
  uint8_t crc_low = crc & 0xff;
  uint8_t crc_high = crc >> 8;
  if (crc_low == 0x28 || crc_low == 0x0d || crc_low == 0x0a)
    crc_low++;
  if (crc_high == 0x28 || crc_high == 0x0d || crc_high == 0x0a)
    crc_high++;
  crc = (crc_high << 8) | crc_low;
  return crc;
}

}  // namespace anern_solar_evo
}  // namespace esphome
