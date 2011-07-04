#include <irsdefs.h>

#include "app.h"

#include <irsfinal.h>

//---------------------------- main application --------------------------------

u309m::app_t::app_t(cfg_t* ap_cfg):
  mp_cfg(ap_cfg),
  #ifdef MEAS_COMM_TEST
  mp_meas_comm(ap_cfg->meas_comm()),
  #endif // MEAS_COMM_TEST
  #ifdef SUPPLY_COMM_TEST
  mp_supply_comm(ap_cfg->supply_comm()),
  #endif // SUPPLY_COMM_TEST
  #ifdef SUPPLY_TEST
  m_supply_200V(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_200V,
    &ap_cfg->eth_data()->supply_200V,
    &ap_cfg->eeprom_data()->supply_200V),
  m_supply_20V(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_20V,
    &ap_cfg->eth_data()->supply_20V,
    &ap_cfg->eeprom_data()->supply_20V),
  m_supply_2V(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_2V,
    &ap_cfg->eth_data()->supply_2V,
    &ap_cfg->eeprom_data()->supply_2V),
  m_supply_1A(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_1A,
    &ap_cfg->eth_data()->supply_1A,
    &ap_cfg->eeprom_data()->supply_1A),
  m_supply_17A(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_17A,
    &ap_cfg->eth_data()->supply_17A,
    &ap_cfg->eeprom_data()->supply_17A),
  #endif // SUPPLY_TEST
  m_bistable_rele_change(false),
  m_mode(rele_check_mode),
  m_rele_timer(irs::make_cnt_ms(10)),
  m_SYM_2V(mp_cfg->eth_data()->rele_ext.SYM_2V),
  m_SYM_20V(mp_cfg->eth_data()->rele_ext.SYM_20V),
  m_SYM_200V(mp_cfg->eth_data()->rele_ext.SYM_200V),
  m_KZ_2V(mp_cfg->eth_data()->rele_ext.KZ_2V),
  //
  m_rel_220V_timer(irs::make_cnt_s(1)),
  //  check
  m_internal_th_value(mp_cfg->eth_data()->arm_adc.internal_temp, 18.f, 50.f),
  m_ptc_a_value(mp_cfg->eth_data()->arm_adc.PTC_A, 0.f, 0.33f),
  m_ptc_lc_value(mp_cfg->eth_data()->arm_adc.PTC_LC, 0.f, 0.33f),
  m_ptc_pwr_value(mp_cfg->eth_data()->arm_adc.PTC_PWR, 0.f, 0.33f),
  m_ptc_17A_value(mp_cfg->eth_data()->arm_adc.PTC_17A, 0.f, 0.33f),
  m_tr_24V_value(mp_cfg->eth_data()->arm_adc.TR_24V_TEST, 23.f, 25.f),
  m_24V_value(mp_cfg->eth_data()->arm_adc.TEST_24V, 23.f, 25.f),
  m_5V_value(mp_cfg->eth_data()->arm_adc.TEST_5V, 4.5f, 5.5f),
  m_izm_6V_value(mp_cfg->eth_data()->arm_adc.IZM_6V_TEST, 4.5f, 6.5f),
  m_izm_3_3V_value(mp_cfg->eth_data()->arm_adc.IZM_3_3V_TEST, 3.15f, 3.45f),
  m_izm_1_2V_value(mp_cfg->eth_data()->arm_adc.IZM_1_2V_TEST, 1.1f, 1.3f),
  m_izm_th1_value(mp_cfg->eth_data()->meas_comm.th1_value, 18.f, 35.f),
  m_izm_th2_value(mp_cfg->eth_data()->meas_comm.th2_value, 18.f, 35.f),
  m_izm_th3_value(mp_cfg->eth_data()->meas_comm.th3_value, 18.f, 35.f),
  m_izm_th4_value(mp_cfg->eth_data()->meas_comm.th4_value, 18.f, 35.f),
  m_izm_th5_value(mp_cfg->eth_data()->meas_comm.th5_value, 18.f, 35.f),
  m_200V_th_base_value(mp_cfg->eth_data()->supply_200V.base_temp_data.value,
    18.f, 70.f),
  m_200V_th_aux_value(mp_cfg->eth_data()->supply_200V.aux_temp_data.value,
    18.f, 70.f),
  m_20V_th_base_value(mp_cfg->eth_data()->supply_20V.base_temp_data.value,
    18.f, 70.f),
  m_20V_th_aux_value(mp_cfg->eth_data()->supply_20V.aux_temp_data.value,
    18.f, 70.f),
  m_2V_th_base_value(mp_cfg->eth_data()->supply_2V.base_temp_data.value,
    18.f, 70.f),
  m_2V_th_aux_value(mp_cfg->eth_data()->supply_200V.aux_temp_data.value,
    18.f, 70.f),
  m_1A_th_base_value(mp_cfg->eth_data()->supply_1A.base_temp_data.value,
    18.f, 70.f),
  m_1A_th_aux_value(mp_cfg->eth_data()->supply_1A.aux_temp_data.value,
    18.f, 70.f),
  m_17A_th_base_value(mp_cfg->eth_data()->supply_17A.base_temp_data.value,
    18.f, 90.f),
  m_17A_th_aux_value(mp_cfg->eth_data()->supply_17A.aux_temp_data.value,
    18.f, 90.f),
  m_alarm_timer(irs::make_cnt_ms(100)),
  m_status(OFF)
{
  m_rel_220V_timer.start();
  m_alarm_timer.start();
}

void u309m::app_t::tick()
{
  mp_cfg->tick();
  #ifdef MEAS_COMM_TEST
  mp_meas_comm->tick();
  #endif // MEAS_COMM_TEST
  #ifdef SUPPLY_COMM_TEST
  mp_supply_comm->tick();
  #endif // SUPPLY_COMM_TEST

  #ifdef SUPPLY_TEST
  m_supply_200V.tick();
  m_supply_20V.tick();
  m_supply_2V.tick();
  m_supply_1A.tick();
  m_supply_17A.tick();
  #endif // SUPPLY_TEST

  if (m_rel_220V_timer.check())
  {
    m_rel_220V_timer.stop();
    mp_cfg->rele_ext_pins()->REL_220V->set();
    mp_cfg->eth_data()->rele_ext.REL_220V = 1;
  }

  switch (m_mode)
  {
    case rele_check_mode:
    {
      if (m_SYM_2V != mp_cfg->eth_data()->rele_ext.SYM_2V) {
        m_bistable_rele_change = true;
        m_SYM_2V = mp_cfg->eth_data()->rele_ext.SYM_2V;
        if (m_SYM_2V) {
          mp_cfg->rele_ext_pins()->SYM_2V_on->set();
          mp_cfg->rele_ext_pins()->SYM_2V_off->clear();
        } else {
          mp_cfg->rele_ext_pins()->SYM_2V_on->clear();
          mp_cfg->rele_ext_pins()->SYM_2V_off->set();
        }
      }
      if (m_SYM_20V != mp_cfg->eth_data()->rele_ext.SYM_20V) {
        m_bistable_rele_change = true;
        m_SYM_20V = mp_cfg->eth_data()->rele_ext.SYM_20V;
        if (m_SYM_20V) {
          mp_cfg->rele_ext_pins()->SYM_20V_on->set();
          mp_cfg->rele_ext_pins()->SYM_20V_off->clear();
        } else {
          mp_cfg->rele_ext_pins()->SYM_20V_on->clear();
          mp_cfg->rele_ext_pins()->SYM_20V_off->set();
        }
      }
      if (m_SYM_200V != mp_cfg->eth_data()->rele_ext.SYM_200V) {
        m_bistable_rele_change = true;
        m_SYM_200V = mp_cfg->eth_data()->rele_ext.SYM_200V;
        if (m_SYM_200V) {
          mp_cfg->rele_ext_pins()->SYM_200V_on->set();
          mp_cfg->rele_ext_pins()->SYM_200V_off->clear();
        } else {
          mp_cfg->rele_ext_pins()->SYM_200V_on->clear();
          mp_cfg->rele_ext_pins()->SYM_200V_off->set();
        }
      }
      if (m_KZ_2V != mp_cfg->eth_data()->rele_ext.KZ_2V) {
        m_bistable_rele_change = true;
        m_KZ_2V = mp_cfg->eth_data()->rele_ext.KZ_2V;
        if (m_KZ_2V) {
          mp_cfg->rele_ext_pins()->KZ_2V_on->set();
          mp_cfg->rele_ext_pins()->KZ_2V_off->clear();
        } else {
          mp_cfg->rele_ext_pins()->KZ_2V_on->clear();
          mp_cfg->rele_ext_pins()->KZ_2V_off->set();
        }
      }
      if (mp_cfg->eth_data()->rele_ext.KZ_1A) {
        mp_cfg->rele_ext_pins()->KZ_1A->set();
      } else {
        mp_cfg->rele_ext_pins()->KZ_1A->clear();
      }
      if (mp_cfg->eth_data()->rele_ext.KZ_17A) {
        mp_cfg->rele_ext_pins()->KZ_17A->set();
      } else {
        mp_cfg->rele_ext_pins()->KZ_17A->clear();
      }
      if (mp_cfg->eth_data()->rele_ext.REL_220V) {
        mp_cfg->rele_ext_pins()->REL_220V->set();
      } else {
        mp_cfg->rele_ext_pins()->REL_220V->clear();
      }
      if (mp_cfg->eth_data()->rele_ext.SYM_OFF) {
        mp_cfg->rele_ext_pins()->SYM_OFF->set();
      } else {
        mp_cfg->rele_ext_pins()->SYM_OFF->clear();
      }
      if (m_bistable_rele_change) {
        m_rele_timer.start();
        m_mode = rele_voltage_off_mode;
      }
    } break;
    case rele_voltage_off_mode:
    {
      if (m_rele_timer.check()) {
        m_bistable_rele_change = false;
        mp_cfg->rele_ext_pins()->SYM_2V_on->clear();
        mp_cfg->rele_ext_pins()->SYM_2V_off->clear();
        mp_cfg->rele_ext_pins()->SYM_20V_on->clear();
        mp_cfg->rele_ext_pins()->SYM_20V_off->clear();
        mp_cfg->rele_ext_pins()->SYM_200V_on->clear();
        mp_cfg->rele_ext_pins()->SYM_200V_off->clear();
        mp_cfg->rele_ext_pins()->KZ_2V_on->clear();
        mp_cfg->rele_ext_pins()->KZ_2V_off->clear();
        m_mode = rele_check_mode;
      }
    } break;
  }
  mp_cfg->eth_data()->rele_ext.SYM_OFF_TEST =
    mp_cfg->rele_ext_pins()->SYM_OFF_TEST->pin();

  if (m_alarm_timer.check())
  {
    mp_cfg->eth_data()->control.alarm_internal_th
      = m_internal_th_value.valid();
    mp_cfg->eth_data()->control.alarm_ptc_a = m_ptc_a_value.valid();
    mp_cfg->eth_data()->control.alarm_ptc_lc = m_ptc_lc_value.valid();
    mp_cfg->eth_data()->control.alarm_ptc_pwr = m_ptc_pwr_value.valid();
    mp_cfg->eth_data()->control.alarm_ptc_17A = m_ptc_17A_value.valid();
    mp_cfg->eth_data()->control.alarm_tr_24V = m_tr_24V_value.valid();
    mp_cfg->eth_data()->control.alarm_24V = m_24V_value.valid();
    mp_cfg->eth_data()->control.alarm_5V = m_5V_value.valid();
    mp_cfg->eth_data()->control.alarm_izm_6V = m_izm_6V_value.valid();
    mp_cfg->eth_data()->control.alarm_izm_3_3V = m_izm_3_3V_value.valid();
    mp_cfg->eth_data()->control.alarm_izm_1_2V = m_izm_1_2V_value.valid();
    mp_cfg->eth_data()->control.alarm_izm_th1 = m_izm_th1_value.valid();
    mp_cfg->eth_data()->control.alarm_izm_th2 = m_izm_th2_value.valid();
    mp_cfg->eth_data()->control.alarm_izm_th3 = m_izm_th3_value.valid();
    mp_cfg->eth_data()->control.alarm_izm_th4 = m_izm_th4_value.valid();
    mp_cfg->eth_data()->control.alarm_izm_th5 = m_izm_th5_value.valid();
    mp_cfg->eth_data()->control.alarm_200V_th_base
      = m_200V_th_base_value.valid();
    mp_cfg->eth_data()->control.alarm_200V_th_aux
      = m_200V_th_aux_value.valid();
    mp_cfg->eth_data()->control.alarm_20V_th_base
      = m_20V_th_base_value.valid();
    mp_cfg->eth_data()->control.alarm_20V_th_aux
      = m_20V_th_aux_value.valid();
    mp_cfg->eth_data()->control.alarm_2V_th_base
      = m_2V_th_base_value.valid();
    mp_cfg->eth_data()->control.alarm_2V_th_aux
      = m_2V_th_aux_value.valid();
    mp_cfg->eth_data()->control.alarm_1A_th_base
      = m_1A_th_base_value.valid();
    mp_cfg->eth_data()->control.alarm_1A_th_aux
      = m_1A_th_aux_value.valid();
    mp_cfg->eth_data()->control.alarm_17A_th_base
      = m_17A_th_base_value.valid();
    mp_cfg->eth_data()->control.alarm_17A_th_aux
      = m_17A_th_aux_value.valid();

    switch (m_status)
    {
      case OFF:
      {
        if (!(mp_cfg->eth_data()->control.alarm & m_alarm_mask)
          || mp_cfg->eth_data()->control.unlock == m_unlock_command)
        {
          m_status = ON;
          mp_cfg->eth_data()->control.on = 1;
          mp_meas_comm->on();
          mp_supply_comm->on();
          m_supply_200V.on();
          m_supply_20V.on();
          m_supply_2V.on();
          m_supply_1A.on();
          m_supply_17A.on();
        }
        break;
      }
      case ON:
      {
        if ((mp_cfg->eth_data()->control.alarm & m_alarm_mask)
          && !(mp_cfg->eth_data()->control.unlock == m_unlock_command))
        {
          m_status = OFF;
          mp_cfg->eth_data()->control.on = 0;
          mp_meas_comm->off();
          mp_supply_comm->off();
          m_supply_200V.off();
          m_supply_20V.off();
          m_supply_2V.off();
          m_supply_1A.off();
          m_supply_17A.off();
        }
        break;
      }
    }
    if (mp_cfg->eth_data()->control.on != m_status)
    {
      mp_cfg->eth_data()->control.on = m_status;
    }
  }
}
