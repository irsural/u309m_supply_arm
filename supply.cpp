#include <irsdefs.h>

#include "supply.h"

#include <irsfinal.h>

u309m::supply_t::supply_t(
  irs::arm::arm_spi_t* ap_spi,
  supply_pins_t* ap_supply_pins,
  supply_eth_data_t* ap_supply_eth_data,
  eeprom_supply_data_t* ap_supply_eeprom_data
):
  mp_supply_pins(ap_supply_pins),
  mp_eth_data(ap_supply_eth_data),
  mp_eeprom_data(ap_supply_eeprom_data),
  m_th_base(ap_spi, mp_supply_pins->termo_sense_base_cs,
    irs::make_cnt_ms(300)),
  m_th_base_data(&m_th_base),
  m_th_aux(ap_spi, mp_supply_pins->termo_sense_aux_cs,
    irs::make_cnt_ms(300)),
  m_th_aux_data(&m_th_aux),
  m_adc102(ap_spi, mp_supply_pins->adc_cs,
    irs::make_cnt_ms(300)),
  m_adc102_data(&m_adc102),
  m_ad5293(ap_spi, mp_supply_pins->tc_cs),
  m_ad5293_data(&m_ad5293),
  m_volt_reg(ap_spi, mp_supply_pins->volt_reg_cs, 0, 0),
  m_volt_reg_data(&m_volt_reg),
  m_temp_reg(ap_spi, mp_supply_pins->temp_reg_cs, 0, 0),
  m_temp_reg_data(&m_temp_reg),
  m_timer(irs::make_cnt_ms(200)),
  m_tc_write(0),
  m_prev_dac_reg_write(0),
  m_fin_dac_reg_write(0),
  m_prev_adc_koef(0),
  m_fin_adc_koef(0),
  m_prev_dac_koef(0),
  m_fin_dac_koef(0),
  m_prev_tr_reg(0),
  m_fin_tr_reg(0),
  m_temp_base_pid_data(irs::zero_struct_t<irs::pid_data_t>::get()),
  m_temp_aux_pid_data(irs::zero_struct_t<irs::pid_data_t>::get()),
  m_dt(0.5),
  m_timer_reg(irs::make_cnt_s(m_dt)),
  m_temp_base_isodr(),
  m_temp_base_time_const(0),
  m_temp_aux_isodr(),
  m_temp_aux_time_const(0)
{
  mp_eth_data->resistance_code = m_ad5293_data.resistance_code;
  m_tc_write = mp_eeprom_data->resistance_code;
  m_prev_adc_koef = mp_eeprom_data->koef_adc_volt_prev;
  m_fin_adc_koef = mp_eeprom_data->koef_adc_volt_fin;
  mp_eth_data->prev_adc_data.koef = m_prev_adc_koef;
  mp_eth_data->fin_adc_data.koef = m_fin_adc_koef;
  m_prev_dac_koef = mp_eeprom_data->koef_reg_prev;
  m_fin_dac_koef = mp_eeprom_data->koef_reg_fin;
  mp_eth_data->prev_dac_data.koef = m_prev_dac_koef;
  mp_eth_data->fin_dac_data.koef = m_fin_dac_koef;
  
  m_temp_base_pid_data.k = mp_eeprom_data->temp_base_k;
  m_temp_base_pid_data.ki = mp_eeprom_data->temp_base_ki;
  m_temp_base_pid_data.kd = mp_eeprom_data->temp_base_kd;
  m_temp_base_pid_data.min = 0;
  m_temp_base_pid_data.max = 65535;
  m_temp_base_pid_data.prev_e = 0.;
  m_temp_base_pid_data.pp_e = 0.;
  m_temp_base_pid_data.prev_out = 0.;
  m_temp_base_pid_data.block_int = 0;
  m_temp_base_pid_data.block_int_ext = 0;
  m_temp_base_pid_data.int_val = 0.;
  m_temp_base_pid_data.k_d_pid = 0.1;
  
  mp_eth_data->base_tr_data.temp_k = m_temp_base_pid_data.k;
  mp_eth_data->base_tr_data.temp_ki = m_temp_base_pid_data.ki/m_dt;
  mp_eth_data->base_tr_data.temp_kd = m_temp_base_pid_data.kd*m_dt;
  
  m_temp_base_time_const = mp_eeprom_data->temp_base_time_const;
  mp_eth_data->base_tr_data.temp_time_const = m_temp_base_time_const;
  mp_eth_data->base_tr_data.temp_prop_koef =
    mp_eeprom_data->temp_base_prop_koef;
  m_temp_base_isodr.k = mp_eth_data->base_tr_data.temp_prop_koef;
  m_temp_base_isodr.fd.x1 = m_th_base_data.temperature_code*
    m_th_base.get_conv_koef();
  m_temp_base_isodr.fd.y1 = m_temp_base_isodr.fd.x1;
  m_temp_base_isodr.fd.t = mp_eth_data->base_tr_data.temp_time_const;
  
  mp_eth_data->base_tr_data.temperature_ref = mp_eeprom_data->temp_base_ref;
  
  m_temp_aux_pid_data.k = mp_eeprom_data->temp_aux_k;
  m_temp_aux_pid_data.ki = mp_eeprom_data->temp_aux_ki;
  m_temp_aux_pid_data.kd = mp_eeprom_data->temp_aux_kd;
  m_temp_aux_pid_data.min = 0;
  m_temp_aux_pid_data.max = 65535;
  m_temp_aux_pid_data.prev_e = 0.;
  m_temp_aux_pid_data.pp_e = 0.;
  m_temp_aux_pid_data.prev_out = 0.;
  m_temp_aux_pid_data.block_int = 0;
  m_temp_aux_pid_data.block_int_ext = 0;
  m_temp_aux_pid_data.int_val = 0.;
  m_temp_aux_pid_data.k_d_pid = 0.1;
    
  mp_eth_data->aux_tr_data.temp_k = m_temp_aux_pid_data.k;
  mp_eth_data->aux_tr_data.temp_ki = m_temp_aux_pid_data.ki/m_dt;
  mp_eth_data->aux_tr_data.temp_kd = m_temp_aux_pid_data.kd*m_dt;
  
  m_temp_aux_time_const = mp_eeprom_data->temp_aux_time_const;
  mp_eth_data->aux_tr_data.temp_time_const = m_temp_aux_time_const;
  mp_eth_data->aux_tr_data.temp_prop_koef =
    mp_eeprom_data->temp_aux_prop_koef;
  m_temp_aux_isodr.k = mp_eth_data->aux_tr_data.temp_prop_koef;
  m_temp_aux_isodr.fd.x1 = m_th_base_data.temperature_code*
    m_th_aux.get_conv_koef();
  m_temp_aux_isodr.fd.y1 = m_temp_base_isodr.fd.x1;
  m_temp_aux_isodr.fd.t = mp_eth_data->aux_tr_data.temp_time_const;
  
  mp_eth_data->aux_tr_data.temperature_ref = mp_eeprom_data->temp_aux_ref;
}

void u309m::supply_t::tick()
{
  m_th_base.tick();
  m_th_aux.tick();
  m_adc102.tick();
  m_ad5293.tick();
  m_volt_reg.tick();
  m_temp_reg.tick();

  bool tc_change = 
    (m_tc_write != mp_eth_data->resistance_code);
  if (tc_change) {
    m_tc_write = 
      mp_eth_data->resistance_code;
    if (m_tc_write > 1023) {
      m_tc_write = 1023;
      m_ad5293_data.resistance_code = m_tc_write;
      mp_eth_data->resistance_code = m_tc_write;
    } else {
      m_ad5293_data.resistance_code = m_tc_write;
    } 
  }
  bool prev_dac_reg_change = 
    (m_prev_dac_reg_write != mp_eth_data->prev_dac_data.voltage_code);
  if (prev_dac_reg_change) {
    m_prev_dac_reg_write = 
      mp_eth_data->prev_dac_data.voltage_code;
    m_volt_reg_data.voltage_code_A = 
      static_cast<irs_u16>(m_prev_dac_reg_write*m_prev_dac_koef);
  }
  bool fin_dac_reg_change = 
    (m_fin_dac_reg_write != mp_eth_data->fin_dac_data.voltage_code);
  if (fin_dac_reg_change) {
    m_fin_dac_reg_write = 
      mp_eth_data->fin_dac_data.voltage_code;
    m_volt_reg_data.voltage_code_B = 
      static_cast<irs_u16>(m_fin_dac_reg_write*m_fin_dac_koef);
  }
  
  bool prev_dac_koef_change = 
    (m_prev_dac_koef != mp_eth_data->prev_dac_data.koef);
  if (prev_dac_koef_change) {
    m_prev_dac_koef = 
      mp_eth_data->prev_dac_data.koef;
    mp_eeprom_data->koef_reg_prev = m_prev_dac_koef;
    m_volt_reg_data.voltage_code_A = 
      static_cast<irs_u16>(m_prev_dac_reg_write*m_prev_dac_koef);
  }
  bool fin_dac_koef_change = 
    (m_fin_dac_koef != mp_eth_data->fin_dac_data.koef);
  if (fin_dac_koef_change) {
    m_fin_dac_koef = 
      mp_eth_data->fin_dac_data.koef;
    mp_eeprom_data->koef_reg_fin = m_fin_dac_koef;
    m_volt_reg_data.voltage_code_B = 
      static_cast<irs_u16>(m_fin_dac_reg_write*m_fin_dac_koef);
  }
  bool prev_adc_koef_change =
    (m_prev_adc_koef != mp_eth_data->prev_adc_data.koef);
  if (prev_adc_koef_change) {
    m_prev_adc_koef =
      mp_eth_data->prev_adc_data.koef;
    mp_eeprom_data->koef_adc_volt_prev = m_prev_adc_koef;
    mp_eth_data->prev_adc_data.voltage_code =
      m_prev_adc_koef*m_adc102_data.voltage_code_A;
  }
  bool fin_adc_koef_change = 
    (m_fin_adc_koef != mp_eth_data->fin_adc_data.koef);
  if (fin_adc_koef_change) {
    m_fin_adc_koef = 
      mp_eth_data->fin_adc_data.koef;
    mp_eeprom_data->koef_adc_volt_fin = m_fin_adc_koef;
    mp_eth_data->fin_adc_data.voltage_code =
      m_fin_adc_koef*m_adc102_data.voltage_code_B;
  }
  
  bool temp_base_pid_k_changed = 
    (m_temp_base_pid_data.k != mp_eth_data->base_tr_data.temp_k);
  if (temp_base_pid_k_changed) {
    m_temp_base_pid_data.k = mp_eth_data->base_tr_data.temp_k;
    mp_eeprom_data->temp_base_k = mp_eth_data->base_tr_data.temp_k;
  }
  
  float temp_base_pid_ki = 
    mp_eth_data->base_tr_data.temp_ki*m_dt;
  bool temp_base_pid_ki_changed = 
    (m_temp_base_pid_data.ki != temp_base_pid_ki);
  if (temp_base_pid_ki_changed) {
    m_temp_base_pid_data.ki = temp_base_pid_ki;
    mp_eeprom_data->temp_base_ki = mp_eth_data->base_tr_data.temp_ki;
  }
  
  float temp_base_pid_kd = 
    mp_eth_data->base_tr_data.temp_kd/m_dt;
  bool temp_base_pid_kd_changed = 
    (m_temp_base_pid_data.kd != temp_base_pid_kd);
  if (temp_base_pid_kd_changed) {
    m_temp_base_pid_data.kd = temp_base_pid_kd;
    mp_eeprom_data->temp_base_kd = mp_eth_data->base_tr_data.temp_kd;
  }

  if (temp_base_pid_k_changed || temp_base_pid_ki_changed ||
    temp_base_pid_kd_changed)
  {
    pid_reg_sync(&m_temp_base_pid_data);
  }
  
  bool temp_aux_pid_k_changed = 
    (m_temp_aux_pid_data.k != mp_eth_data->aux_tr_data.temp_k);
  if (temp_aux_pid_k_changed) {
    m_temp_aux_pid_data.k = mp_eth_data->aux_tr_data.temp_k;
    mp_eeprom_data->temp_aux_k = mp_eth_data->aux_tr_data.temp_k;
  }
  
  float temp_aux_pid_ki = 
    mp_eth_data->aux_tr_data.temp_ki*m_dt;
  bool temp_aux_pid_ki_changed = 
    (m_temp_aux_pid_data.ki != temp_aux_pid_ki);
  if (temp_aux_pid_ki_changed) {
    m_temp_aux_pid_data.ki = temp_aux_pid_ki;
    mp_eeprom_data->temp_aux_ki = mp_eth_data->aux_tr_data.temp_ki;
  }
  
  float temp_aux_pid_kd = 
    mp_eth_data->aux_tr_data.temp_kd/m_dt;
  bool temp_aux_pid_kd_changed = 
    (m_temp_aux_pid_data.kd != temp_aux_pid_kd);
  if (temp_aux_pid_kd_changed) {
    m_temp_aux_pid_data.kd = temp_aux_pid_kd;
    mp_eeprom_data->temp_aux_kd = mp_eth_data->aux_tr_data.temp_kd;
  } 

  if (temp_aux_pid_k_changed || temp_aux_pid_ki_changed ||
    temp_aux_pid_kd_changed)
  {
    pid_reg_sync(&m_temp_aux_pid_data);
  }
  
  if (m_temp_base_time_const !=
    mp_eth_data->base_tr_data.temp_time_const)
  {
    m_temp_base_time_const = 
      static_cast<float>(mp_eth_data->base_tr_data.temp_time_const);
    mp_eeprom_data->temp_base_time_const = m_temp_base_time_const;
    m_temp_base_isodr.fd.t = m_temp_base_time_const;
  }
  
  if (m_temp_base_isodr.k != mp_eth_data->base_tr_data.temp_prop_koef) {
    m_temp_base_isodr.k = mp_eth_data->base_tr_data.temp_prop_koef;
    mp_eeprom_data->temp_base_prop_koef = m_temp_base_isodr.k;
  }
  
  if (m_temp_aux_time_const != mp_eth_data->aux_tr_data.temp_time_const)
  {
    m_temp_aux_time_const = 
      static_cast<float>(mp_eth_data->aux_tr_data.temp_time_const);
    mp_eeprom_data->temp_aux_time_const = m_temp_aux_time_const;
    m_temp_aux_isodr.fd.t = m_temp_aux_time_const;
  }
  
  if (m_temp_aux_isodr.k != mp_eth_data->aux_tr_data.temp_prop_koef) {
    m_temp_aux_isodr.k = mp_eth_data->aux_tr_data.temp_prop_koef;
    mp_eeprom_data->temp_aux_prop_koef = m_temp_aux_isodr.k;
  }
  
  if (m_timer.check()) {
    mp_eth_data->base_temp_data.value = 
     m_th_base_data.temperature_code*m_th_base.get_conv_koef();
    mp_eth_data->base_temp_data.filtered_value =
      fade(&m_temp_base_isodr.fd,
      static_cast<float>(mp_eth_data->base_temp_data.value));
    mp_eth_data->aux_temp_data.value = 
     m_th_aux_data.temperature_code*m_th_aux.get_conv_koef();
    mp_eth_data->aux_temp_data.filtered_value =
      fade(&m_temp_aux_isodr.fd,
      static_cast<float>(mp_eth_data->aux_temp_data.value));
    
    mp_eth_data->prev_adc_data.voltage_code = 
      m_prev_adc_koef*m_adc102_data.voltage_code_A;
    mp_eth_data->fin_adc_data.voltage_code = 
      m_fin_adc_koef*m_adc102_data.voltage_code_B;
  }
  
  if (mp_eeprom_data->temp_base_ref !=
    mp_eth_data->base_tr_data.temperature_ref)
  {
    mp_eeprom_data->temp_base_ref = 
      mp_eth_data->base_tr_data.temperature_ref;
  }
  
  if (mp_eeprom_data->temp_aux_ref !=
    mp_eth_data->aux_tr_data.temperature_ref)
  {
    mp_eeprom_data->temp_aux_ref = 
      mp_eth_data->aux_tr_data.temperature_ref;
  }
  
  if (m_timer_reg.check()) {
    double base_pid_reg_data_in =
      mp_eth_data->base_tr_data.temperature_ref -
      mp_eth_data->base_temp_data.filtered_value;
    m_temp_reg_data.voltage_code_A = 
      static_cast<irs_u16>(irs::pid_reg(&m_temp_base_pid_data,
      base_pid_reg_data_in));
    mp_eth_data->base_tr_data.dac_value = 
      m_temp_reg_data.voltage_code_A;
    mp_eth_data->base_tr_data.int_val =
      static_cast<irs_u8>(m_temp_base_pid_data.int_val*m_temp_base_pid_data.k*
      m_temp_base_pid_data.ki);
    
    double aux_pid_reg_data_in =
      mp_eth_data->aux_tr_data.temperature_ref -
      mp_eth_data->aux_temp_data.filtered_value;
    m_temp_reg_data.voltage_code_B = 
      static_cast<irs_u16>(irs::pid_reg(&m_temp_aux_pid_data,
      aux_pid_reg_data_in));
    mp_eth_data->aux_tr_data.dac_value =
      m_temp_reg_data.voltage_code_B;
    mp_eth_data->aux_tr_data.int_val =
      static_cast<irs_u8>(m_temp_aux_pid_data.int_val*m_temp_aux_pid_data.k*
      m_temp_aux_pid_data.ki);
  }
}
