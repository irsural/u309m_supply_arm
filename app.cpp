#include <irsdefs.h>

#include "app.h"

#include <irsfinal.h>

//---------------------------- main application --------------------------------

u309m::app_t::app_t(cfg_t* ap_cfg):
  mp_cfg(ap_cfg),
  m_meas_comm_app(
    //ap_cfg->adc(),
    ap_cfg->spi_general_purpose(),
    ap_cfg->spi_supply_comm_plis(),
    ap_cfg->command_pins()->meas_comm,
    &ap_cfg->eth_data()->meas_comm
  ),
  //m_supply_comm_app(ap_cfg),
  /*m_supply_200V(ap_cfg->spi_general_purpose(),
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
    &ap_cfg->eeprom_data()->supply_17A),*/
  m_bistable_rele_change(false),
  m_mode(rele_check_mode),
  m_rele_timer(irs::make_cnt_ms(10)),
  m_SYM_2V(mp_cfg->eth_data()->rele_ext.SYM_2V),
  m_SYM_20V(mp_cfg->eth_data()->rele_ext.SYM_20V),
  m_SYM_200V(mp_cfg->eth_data()->rele_ext.SYM_200V),
  m_KZ_2V(mp_cfg->eth_data()->rele_ext.KZ_2V)
{
}

void u309m::app_t::tick()
{
  mp_cfg->tick();
  m_meas_comm_app.tick();
  
  //m_supply_comm_app.tick();
  /*m_supply_200V.tick();
  m_supply_20V.tick();
  m_supply_2V.tick();
  m_supply_1A.tick();
  m_supply_17A.tick();*/
  
  /*switch (m_mode)
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
      if (mp_cfg->eth_data()->rele_ext.SYM_OFF_TEST) {
        mp_cfg->rele_ext_pins()->SYM_OFF_TEST->set();
      } else {
        mp_cfg->rele_ext_pins()->SYM_OFF_TEST->clear();
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
  }*/
  
}
