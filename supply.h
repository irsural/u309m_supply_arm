#ifndef supplyh
#define supplyh

#include <irsdefs.h>

#include <irsadc.h>
#include <irsdsp.h>
#include <armspi.h>

#include "data.h"
#include "cfg.h"

#include <irsfinal.h>

namespace u309m {

class supply_t
{
public:
  supply_t(
    irs::arm::arm_spi_t* ap_spi,
    supply_pins_t* ap_supply_pins,
    supply_eth_data_t* ap_supply_eth_data,
    supply_add_data_t* ap_add_eth_data,
    eeprom_supply_data_t* ap_supply_eeprom_data
  );
  void on();
  void off();
  inline bool operated() { return m_operate; };
  void dac_log_enable();
  void refresh_dac_values();
  void disable_saving_aux_th_ref();
  void tick();
private:
  enum mode_thermo_off_t {
    mto_command_wait,
    mto_heater_wait
  };

  supply_pins_t* mp_supply_pins;
  supply_eth_data_t* mp_eth_data;
  supply_add_data_t* mp_add_eth_data;
  eeprom_supply_data_t* mp_eeprom_data;
  irs::th_lm95071_t m_th_base;
  irs::th_lm95071_data_t m_th_base_data;
  irs::th_lm95071_t m_th_aux;
  irs::th_lm95071_data_t m_th_aux_data;
  irs::adc_adc102s021_t m_adc102;
  irs::adc_adc102s021_data_t m_adc102_data;
  irs::dac_ad5293_t m_ad5293;
  irs::dac_ad5293_data_t m_ad5293_data;
  irs::dac_ltc2622_t m_volt_reg;
  irs::dac_ltc2622_data_t m_volt_reg_data;
  irs::dac_ltc2622_t m_temp_reg;
  irs::dac_ltc2622_data_t m_temp_reg_data;
  irs::loop_timer_t m_timer;
  irs::loop_timer_t m_timer_2;
  irs_u16 m_tc_write;
  float m_prev_dac_reg_write;
  float m_fin_dac_reg_write;
  float m_prev_adc_koef;
  float m_fin_adc_koef;
  float m_prev_dac_koef;
  float m_fin_dac_koef;
  //float m_prev_tr_reg;  - чо это?
  //float m_fin_tr_reg;
  float m_temp_base_ki;
  float m_temp_base_kd;
  float m_temp_aux_ki;
  float m_temp_aux_kd;
  irs::pid_data_t m_temp_base_pid_data;
  irs::pid_data_t m_temp_aux_pid_data;
  float m_dt;
  irs::loop_timer_t m_timer_reg;
  irs::isodr_data_t m_temp_base_isodr_2;
  irs::isodr_data_t m_temp_base_isodr;
  float m_temp_base_time_const;
  irs::isodr_data_t m_temp_aux_isodr;
  float m_temp_aux_time_const;
  bool m_operate;
  bool m_enable_saving_aux_th_ref;
  bool m_th_base_start;
  bool m_th_aux_start;
  bool m_pid_reg_start;
  mode_thermo_off_t mode_thermo_off;
  bool m_temp_reg_off;
  bool m_heater_spi_off;
  irs::bit_data_t::bit_t thermo_off_prev;
}; // supply_t

} // namespace u309m

#endif // supplyh
