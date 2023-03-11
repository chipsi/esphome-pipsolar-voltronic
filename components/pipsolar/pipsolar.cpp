#include "pipsolar.h"
#include "esphome/core/log.h"

namespace esphome {
namespace pipsolar {

static const char *const TAG = "pipsolar";

void Pipsolar::setup() {
  this->state_ = STATE_IDLE;
  this->command_start_millis_ = 0;
}

void Pipsolar::empty_uart_buffer_() {
  uint8_t byte;
  while (this->available()) {
    this->read_byte(&byte);
  }
}

void Pipsolar::loop() {
  // Read message
  if (this->state_ == STATE_IDLE) {
    this->empty_uart_buffer_();
    switch (this->send_next_command_()) {
      case 0:
        // no command send (empty queue) time to poll
        if (millis() - this->last_poll_ > this->update_interval_) {
          this->send_next_poll_();
          this->last_poll_ = millis();
        }
        return;
        break;
      case 1:
        // command send
        return;
        break;
    }
  }
  if (this->state_ == STATE_COMMAND_COMPLETE) {
    if (this->check_incoming_length_(4)) {
      ESP_LOGD(TAG, "response length for command OK");
      if (this->check_incoming_crc_()) {
        // crc ok
        if (this->read_buffer_[1] == 'A' && this->read_buffer_[2] == 'C' && this->read_buffer_[3] == 'K') {
          ESP_LOGD(TAG, "command successful");
        } else {
          ESP_LOGD(TAG, "command not successful");
        }
        this->command_queue_[this->command_queue_position_] = std::string("");
        this->command_queue_position_ = (command_queue_position_ + 1) % COMMAND_QUEUE_LENGTH;
        this->state_ = STATE_IDLE;

      } else {
        // crc failed
        this->command_queue_[this->command_queue_position_] = std::string("");
        this->command_queue_position_ = (command_queue_position_ + 1) % COMMAND_QUEUE_LENGTH;
        this->state_ = STATE_IDLE;
      }
    } else {
      ESP_LOGD(TAG, "response length for command %s not OK: with length %zu",
               this->command_queue_[this->command_queue_position_].c_str(), this->read_pos_);
      this->command_queue_[this->command_queue_position_] = std::string("");
      this->command_queue_position_ = (command_queue_position_ + 1) % COMMAND_QUEUE_LENGTH;
      this->state_ = STATE_IDLE;
    }
  }

  if (this->state_ == STATE_POLL_DECODED) {
    std::string mode;
    switch (this->used_polling_commands_[this->last_polling_command_].identifier) {
      case POLLING_0P007PIRI:
        if (this->ac_input_rating_voltage_) {
          this->ac_input_rating_voltage_->publish_state(value_ac_input_rating_voltage_);
        }
        if (this->ac_input_rating_current_) {
          this->ac_input_rating_current_->publish_state(value_ac_input_rating_current_);
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
        if (this->battery_redischarge_voltage_) {
          this->battery_redischarge_voltage_->publish_state(value_battery_redischarge_voltage_);
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
          this->battery_type_->publish_state(value_battery_type_);
        }
        if (this->max_ac_charging_current_) {
          this->max_ac_charging_current_->publish_state(value_max_ac_charging_current_);
        }
        if (this->max_charging_current_) {
          this->max_charging_current_->publish_state(value_max_charging_current_);
        }
        if (this->input_voltage_range_) {
          this->input_voltage_range_->publish_state(value_input_voltage_range_);
        }
        // special for input voltage range switch
        if (this->input_voltage_range_switch_) {
          this->input_voltage_range_switch_->publish_state(value_input_voltage_range_ == 1);
        }
        if (this->output_source_priority_) {
          this->output_source_priority_->publish_state(value_output_source_priority_);
        }
        if (this->charger_source_priority_) {
          this->charger_source_priority_->publish_state(value_charger_source_priority_);
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
        if (this->output_model_setting_) {
          this->output_model_setting_->publish_state(value_output_model_setting_);
        }
        if (this->solar_power_priority_) {
          this->solar_power_priority_->publish_state(value_solar_power_priority_);
        }
        if (this->mppt_string_) {
          this->mppt_string_->publish_state(value_mppt_string_);
        }
        if (this->bulk1_) {
          this->bulk1_->publish_state(value_bulk1_);
        }
        this->state_ = STATE_IDLE;
        break;
      case POLLING_0P005GS:
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
        if (this->battery_voltage_) {
          this->battery_voltage_->publish_state(value_battery_voltage_);
        }
        if (this->battery_voltage_from_scc_) {
          this->battery_voltage_from_scc_->publish_state(value_battery_voltage_from_scc_);
        }
        if (this->battery_voltage_from_scc2_) {
          this->battery_voltage_from_scc2_->publish_state(value_battery_voltage_from_scc2_);
        }
        if (this->battery_discharge_current_) {
          this->battery_discharge_current_->publish_state(value_battery_discharge_current_);
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
        if (this->mppt1_charger_temperature_) {
          this->mppt1_charger_temperature_->publish_state(value_mppt1_charger_temperature_);
        }
        if (this->mppt2_charger_temperature_) {
          this->mppt2_charger_temperature_->publish_state(value_mppt2_charger_temperature_);
        }
        if (this->pv1_input_power_) {
          this->pv1_input_power_->publish_state(value_pv1_input_power_);
        }
        if (this->pv2_input_power_) {
          this->pv2_input_power_->publish_state(value_pv2_input_power_);
        }
        if (this->pv1_input_voltage_) {
          this->pv1_input_voltage_->publish_state(value_pv1_input_voltage_);
        }
        if (this->pv2_input_voltage_) {
          this->pv2_input_voltage_->publish_state(value_pv2_input_voltage_);
        }
        if (this->setting_value_configuration_state_) {
          this->setting_value_configuration_state_->publish_state(value_setting_value_configuration_state_);
        }
        if (this->mppt1_charger_status_) {
          this->mppt1_charger_status_->publish_state(value_mppt1_charger_status_);
        }
        if (this->mppt2_charger_status_) {
          this->mppt2_charger_status_->publish_state(value_mppt2_charger_status_);
        }
        if (this->load_connection_) {
          this->load_connection_->publish_state(value_load_connection_);
        }
        if (this->battery_power_direction_) {
          this->battery_power_direction_->publish_state(value_battery_power_direction_);
        }
        if (this->dc_ac_power_direction_) {
          this->dc_ac_power_direction_->publish_state(value_dc_ac_power_direction_);
        }
        if (this->line_power_direction_) {
          this->line_power_direction_->publish_state(value_line_power_direction_);
        }
        if (this->local_parallel_id_) {
          this->local_parallel_id_->publish_state(value_local_parallel_id_);
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
        if (this->warnung_low_pv_energy_) {
          this->warnung_low_pv_energy_->publish_state(value_warnung_low_pv_energy_);
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
      case POLLING_QBATCD:
        if (this->discharge_onoff_) {
          this->discharge_onoff_->publish_state(value_discharge_onoff_);
        }
        if (this->discharge_with_standby_onoff_) {
          this->discharge_with_standby_onoff_->publish_state(value_discharge_with_standby_onoff_);
        }
        if (this->charge_onoff_) {
          this->charge_onoff_->publish_state(value_charge_onoff_);
        }
        if (this->charging_discharging_control_select_) {
          this->charging_discharging_control_select_->map_and_publish(value_charging_discharging_control_select_);
        }
        this->state_ = STATE_IDLE;
        break;
      case POLLING_QT:
      case POLLING_QMN:
        this->state_ = STATE_IDLE;
        break;
    }
  }

  if (this->state_ == STATE_POLL_CHECKED) {
    bool enabled = true;
    std::string fc;
    char tmp[PIPSOLAR_READ_BUFFER_LENGTH];
    sprintf(tmp, "%s", this->read_buffer_);
    switch (this->used_polling_commands_[this->last_polling_command_].identifier) {
      case POLLING_0P007PIRI:
        ESP_LOGD(TAG, "Decode ^P007PIRI");
        ESP_LOGD(TAG, "String ^P007PIRI : %s ", tmp);
        // 240.0 15.0 240.0 50.0 15.0 3600 3600 24.0 24.0 23.5 29.2 29.0 2 010 100 0 2 3 1 01 0 0 26.5 0 0 (Axpert VM IV 24v 3600w)
        // 240.0 22.9 240.0 50.0 22.9 5500 5500 48.0 51.0 42.0 55.1 54.7 2 02 100 1 2 1 1 01 0 0 53.0 0 1  (EASUN SML-II 48V 5500W)
        // ^D0892400,233,2400,500,233,5600,5600,480,500,530,480,547,547,2,002,120,0,1,1,9,0,0,0,1,1,01     (EASUN SV-IV 48V 5600W)
        //       A    B   C    D   E    F    G   H   I   J   K   L   M  N  O   P  Q R S T U V W Z a  b
        ESP_LOGD(TAG, "SSCANF return : %d ", sscanf(tmp, "^D089%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",          // NOLINT
        //                A  B  C  D  E  F  G  H  I  J  K  L  M  N  O  P  Q  R  S  T  U  V  W  Z  a  b?
               &value_ac_input_rating_voltage_,                                                 //           A     // NOLINT
               &value_ac_input_rating_current_,                                                 //           B     // NOLINT
               &value_ac_output_rating_voltage_,                                                //           C     // NOLINT
               &value_ac_output_rating_frequency_,                                              //           D     // NOLINT
               &value_ac_output_rating_current_,                                                //           E     // NOLINT
               &value_ac_output_rating_apparent_power_,                                         //           F     // NOLINT
               &value_ac_output_rating_active_power_,                                           //           G     // NOLINT
               &value_battery_rating_voltage_,                                                  //           H     // NOLINT
               &value_battery_recharge_voltage_,                                                //           I     // NOLINT
               &value_battery_redischarge_voltage_,                                             //           J     // NOLINT
               &value_battery_under_voltage_,                                                   //           K     // NOLINT
               &value_battery_bulk_voltage_,                                                    //           L     // NOLINT
               &value_battery_float_voltage_,                                                   //           M     // NOLINT
               &value_battery_type_,                                                            //           N     // NOLINT
               &value_max_ac_charging_current_,                                                 //           O     // NOLINT
               &value_max_charging_current_,                                                    //           P     // NOLINT
               &value_input_voltage_range_,                                                     //           Q     // NOLINT
               &value_output_source_priority_,                                                  //           R     // NOLINT
               &value_charger_source_priority_,                                                 //           S     // NOLINT
               &value_parallel_max_num_,                                                        //           T     // NOLINT
               &value_machine_type_,                                                            //           U     // NOLINT
               &value_topology_,                                                                //           V     // NOLINT
               &value_output_model_setting_,                                                    //           W     // NOLINT
               &value_solar_power_priority_,                                                    //           Z     // NOLINT
               &value_mppt_string_,                                                             //           a     // NOLINT
               &value_bulk1_                                                                    //           b     // NOLINT
        ));
        if (this->last_qpiri_) {
          this->last_qpiri_->publish_state(tmp);
        }
        this->state_ = STATE_POLL_DECODED;
        break;
      case POLLING_0P005GS:
        ESP_LOGD(TAG, "Decode ^P005GS");
        ESP_LOGD(TAG, "String ^P005GS : %s ", tmp);
        // Response examples of the PIP 2424MSE1
        // 226.7 49.9 226.7 49.9 0498 0479 016 427 27.00 005 100 0035 01.9 255.1 00.00 00000 10010110 00 00 00510 110 (2424MSE1)
        // 225.8 49.9 225.8 49.9 0609 0565 020 427 27.00 005 100 0035 02.2 259.9 00.00 00000 10010110 00 00 00590 110 (2424MSE1)
        // 247.3 50.0 239.0 50.0 0931 0805 025 360 26.10 007 060 0017 04.6 179.2 00.00 00001 00010110 00 00 00831 011 (Axpert VM IV 24v 3600w)
        // 232.6 50.0 229.9 49.9 0391 0312 007 402 54.40 042 072 0066 0042 284.6 00.00 00000 00010010 00 00 02901 010 (PIP-5048Mg FW71.85)
        // 218.1 49.9 218.1 49.9 0327 0295 005 360 51.20 000 100 0037 00.0 000.0 00.00 00000 00010000 00 00 00002 011 0 00 0000 (PIP6048MT)
        // 000.0 00.0 239.5 50.0 0144 0014 002 422 53.10 004 100 0034 02.2 206.0 00.00 00000 00010110 00 00 00457 110 (EASUN SML-II 48V 5500W)
        // ^D1062417,499,2417,499,0483,0364,008,510,000,000,000,000,028,022,000,000,0392,0000,2143,0000,0,2,0,1,1,2,1,0 (EASUN SV-IV 48V 5600W)
        //       A    B   C    D   E    F    G   H   I   J   K   L   M   N   O   P   Q    R    S    T   U V W X Y Z a b
        ESP_LOGD(TAG, "SSCANF return : %d ", sscanf(                                                                                                  // NOLINT
            tmp,                                                                                                 // NOLINT
            "(^D106%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",         // NOLINT
        //         A  B  C  D  E  F  G  H  I  J  K  L  M  N  O  P  Q  R  S  T  U  V  W  X  Y  Z  a  b
            &value_grid_voltage_,                                                             //           A     // NOLINT
            &value_grid_frequency_,                                                           //           B     // NOLINT
            &value_ac_output_voltage_,                                                        //           C     // NOLINT
            &value_ac_output_frequency_,                                                      //           D     // NOLINT
            &value_ac_output_apparent_power_,                                                 //           E     // NOLINT
            &value_ac_output_active_power_,                                                   //           F     // NOLINT
            &value_output_load_percent_,                                                      //           G     // NOLINT
            &value_battery_voltage_,                                                          //           H     // NOLINT
            &value_battery_voltage_from_scc_,                                                 //           I     // NOLINT
            &value_battery_voltage_from_scc2_,                                                //           J     // NOLINT
            &value_battery_discharge_current_,                                                //           K     // NOLINT
            &value_battery_charging_current_,                                                 //           L     // NOLINT
            &value_battery_capacity_percent_,                                                 //           M     // NOLINT
            &value_inverter_heat_sink_temperature_,                                           //           N     // NOLINT
            &value_mppt1_charger_temperature_,                                                //           O     // NOLINT
            &value_mppt2_charger_temperature_,                                                //           P     // NOLINT
            &value_pv1_input_power_,                                                          //           Q     // NOLINT
            &value_pv2_input_power_,                                                          //           R     // NOLINT
            &value_pv1_input_voltage_,                                                        //           S     // NOLINT
            &value_pv2_input_voltage_,                                                        //           T     // NOLINT
            &value_setting_value_configuration_state_,                                        //           U     // NOLINT
            &value_mppt1_charger_status_,                                                     //           V     // NOLINT
            &value_mppt2_charger_status_,                                                     //           W     // NOLINT
            &value_load_connection_,                                                          //           X     // NOLINT
            &value_battery_power_direction_,                                                  //           Y     // NOLINT
            &value_dc_ac_power_direction_,                                                    //           Z     // NOLINT
            &value_line_power_direction_,                                                     //           a     // NOLINT
            &value_local_parallel_id_                                                         //           b     // NOLINT
        ));
        ESP_LOGD(TAG, "Value value_grid_voltage_ : %d ", value_grid_voltage_);
        if (this->last_qgs_) {
          this->last_qgs_->publish_state(tmp);
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
        this->value_faults_present_ = true;

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
              this->value_warnung_low_pv_energy_ = enabled;
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
      case POLLING_QBATCD:
        ESP_LOGD(TAG, "Decode QBATCD");
        // '(000'
        for (size_t i = 1; i < strlen(tmp); i++) {
          enabled = tmp[i] == '1';
          switch (i) {
            case 1:
              this->value_discharge_onoff_ = enabled;
              break;
            case 2:
              this->value_discharge_with_standby_onoff_ = enabled;
              break;
            case 3:
              this->value_charge_onoff_ = enabled;
              break;
          }
        }
        this->value_charging_discharging_control_select_ = tmp;

        if (this->last_qbatcd_) {
          this->last_qbatcd_->publish_state(tmp);
        }
        this->state_ = STATE_POLL_DECODED;
        break;
      default:
        this->state_ = STATE_IDLE;
        break;
    }
    return;
  }

  if (this->state_ == STATE_POLL_COMPLETE) {
    if (this->check_incoming_crc_()) {
      if (this->read_buffer_[0] == '(' && this->read_buffer_[1] == 'N' && this->read_buffer_[2] == 'A' &&
          this->read_buffer_[3] == 'K') {
        this->state_ = STATE_IDLE;
        return;
      }
      // crc ok
      this->state_ = STATE_POLL_CHECKED;
      return;
    } else {
      this->state_ = STATE_IDLE;
    }
  }

  if (this->state_ == STATE_COMMAND || this->state_ == STATE_POLL) {
    while (this->available()) {
      uint8_t byte;
      this->read_byte(&byte);

      if (this->read_pos_ == PIPSOLAR_READ_BUFFER_LENGTH) {
        this->read_pos_ = 0;
        this->empty_uart_buffer_();
      }
      this->read_buffer_[this->read_pos_] = byte;
      this->read_pos_++;

      // end of answer
      if (byte == 0x0D) {
        this->read_buffer_[this->read_pos_] = 0;
        this->empty_uart_buffer_();
        if (this->state_ == STATE_POLL) {
          this->state_ = STATE_POLL_COMPLETE;
        }
        if (this->state_ == STATE_COMMAND) {
          this->state_ = STATE_COMMAND_COMPLETE;
        }
      }
    }  // available
  }
  if (this->state_ == STATE_COMMAND) {
    if (millis() - this->command_start_millis_ > esphome::pipsolar::Pipsolar::COMMAND_TIMEOUT) {
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
    if (millis() - this->command_start_millis_ > esphome::pipsolar::Pipsolar::COMMAND_TIMEOUT) {
      // command timeout
      ESP_LOGD(TAG, "timeout command to poll: %s", this->used_polling_commands_[this->last_polling_command_].command);
      this->state_ = STATE_IDLE;
    } else {
    }
  }
}

uint8_t Pipsolar::check_incoming_length_(uint8_t length) {
  if (this->read_pos_ - 3 == length) {
    return 1;
  }
  return 0;
}

uint8_t Pipsolar::check_incoming_crc_() {
  uint16_t crc16;
  crc16 = cal_crc_half_(read_buffer_, read_pos_ - 3);
  ESP_LOGD(TAG, "checking crc on incoming message");
  if (((uint8_t)((crc16) >> 8)) == read_buffer_[read_pos_ - 3] &&
      ((uint8_t)((crc16) &0xff)) == read_buffer_[read_pos_ - 2]) {
    ESP_LOGD(TAG, "CRC OK");
    read_buffer_[read_pos_ - 1] = 0;
    read_buffer_[read_pos_ - 2] = 0;
    read_buffer_[read_pos_ - 3] = 0;
    return 1;
  }
  ESP_LOGD(TAG, "CRC NOK expected: %X %X but got: %X %X", ((uint8_t)((crc16) >> 8)), ((uint8_t)((crc16) &0xff)),
           read_buffer_[read_pos_ - 3], read_buffer_[read_pos_ - 2]);
  return 0;
}

// send next command used
uint8_t Pipsolar::send_next_command_() {
  uint16_t crc16;
  if (this->command_queue_[this->command_queue_position_].length() != 0) {
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
    crc16 = cal_crc_half_(byte_command, length);
    this->write_str(command);
    // checksum
    this->write(((uint8_t)((crc16) >> 8)));   // highbyte
    this->write(((uint8_t)((crc16) &0xff)));  // lowbyte
    // end Byte
    this->write(0x0D);
    ESP_LOGD(TAG, "Sending command from queue: %s with length %d", command, length);
    return 1;
  }
  return 0;
}

void Pipsolar::send_next_poll_() {
  uint16_t crc16;
  this->last_polling_command_ = (this->last_polling_command_ + 1) % 15;
  if (this->used_polling_commands_[this->last_polling_command_].length == 0) {
    this->last_polling_command_ = 0;
  }
  if (this->used_polling_commands_[this->last_polling_command_].length == 0) {
    // no command specified
    return;
  }
  this->state_ = STATE_POLL;
  this->command_start_millis_ = millis();
  this->empty_uart_buffer_();
  this->read_pos_ = 0;
  crc16 = cal_crc_half_(this->used_polling_commands_[this->last_polling_command_].command,
                        this->used_polling_commands_[this->last_polling_command_].length);
  this->write_array(this->used_polling_commands_[this->last_polling_command_].command,
                    this->used_polling_commands_[this->last_polling_command_].length);
  // checksum
  this->write(((uint8_t)((crc16) >> 8)));   // highbyte
  this->write(((uint8_t)((crc16) &0xff)));  // lowbyte
  // end Byte
  this->write(0x0D);
  ESP_LOGD(TAG, "Sending polling command : %s with length %d",
           this->used_polling_commands_[this->last_polling_command_].command,
           this->used_polling_commands_[this->last_polling_command_].length);
}

void Pipsolar::queue_command_(const char *command, uint8_t length) {
  uint8_t next_position = command_queue_position_;
  for (uint8_t i = 0; i < COMMAND_QUEUE_LENGTH; i++) {
    uint8_t testposition = (next_position + i) % COMMAND_QUEUE_LENGTH;
    if (command_queue_[testposition].length() == 0) {
      command_queue_[testposition] = command;
      ESP_LOGD(TAG, "Command queued successfully: %s with length %u at position %d", command,
               command_queue_[testposition].length(), testposition);
      return;
    }
  }
  ESP_LOGD(TAG, "Command queue full dropping command: %s", command);
}

void Pipsolar::switch_command(const std::string &command) {
  ESP_LOGD(TAG, "got command: %s", command.c_str());
  queue_command_(command.c_str(), command.length());
}
void Pipsolar::dump_config() {
  ESP_LOGCONFIG(TAG, "Pipsolar:");
  ESP_LOGCONFIG(TAG, "used commands:");
  for (auto &used_polling_command : this->used_polling_commands_) {
    if (used_polling_command.length != 0) {
      ESP_LOGCONFIG(TAG, "%s", used_polling_command.command);
    }
  }
}
void Pipsolar::update() {}

void Pipsolar::add_polling_command_(const char *command, ENUMPollingCommand polling_command) {
  for (auto &used_polling_command : this->used_polling_commands_) {
    if (used_polling_command.length == strlen(command)) {
      uint8_t len = strlen(command);
      if (memcmp(used_polling_command.command, command, len) == 0) {
        used_polling_command.command[0] = 0x5e;
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
        used_polling_command.command[i] = (uint8_t)(*beg);
      }
      used_polling_command.errors = 0;
      used_polling_command.identifier = polling_command;
      used_polling_command.length = length - 1;
      return;
    }
  }
}

uint16_t Pipsolar::cal_crc_half_(uint8_t *msg, uint8_t len) {
  uint16_t crc;

  uint8_t da;
  uint8_t *ptr;
  uint8_t b_crc_hign;
  uint8_t b_crc_low;

  uint16_t crc_ta[16] = {0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
                         0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef};

  ptr = msg;
  crc = 0;

  while (len-- != 0) {
    da = ((uint8_t)(crc >> 8)) >> 4;
    crc <<= 4;
    crc ^= crc_ta[da ^ (*ptr >> 4)];
    da = ((uint8_t)(crc >> 8)) >> 4;
    crc <<= 4;
    crc ^= crc_ta[da ^ (*ptr & 0x0f)];
    ptr++;
  }

  b_crc_low = crc;
  b_crc_hign = (uint8_t)(crc >> 8);

  if (b_crc_low == 0x28 || b_crc_low == 0x0d || b_crc_low == 0x0a)
    b_crc_low++;
  if (b_crc_hign == 0x28 || b_crc_hign == 0x0d || b_crc_hign == 0x0a)
    b_crc_hign++;

  crc = ((uint16_t) b_crc_hign) << 8;
  crc += b_crc_low;
  return (crc);
}

}  // namespace pipsolar
}  // namespace esphome
