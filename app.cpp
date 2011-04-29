#include <irsdefs.h>

#include "app.h"

#include <irsfinal.h>

//---------------------------- main application --------------------------------

u309m::app_t::app_t(cfg_t* ap_cfg):
  mp_cfg(ap_cfg),
  m_meas_comm_app(
    ap_cfg->adc(),
    ap_cfg->spi_general_purpose(),
    ap_cfg->spi_supply_comm_plis(),
    ap_cfg->command_pins()->meas_comm_pins,
    &ap_cfg->eth_data()->meas_comm_data
  ),
  //m_supply_comm_app(ap_cfg),
  m_supply_200V(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_200V_pins,
    &ap_cfg->eth_data()->supply_200V_data),
  m_supply_20V(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_20V_pins,
    &ap_cfg->eth_data()->supply_20V_data),
  m_supply_2V(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_2V_pins,
    &ap_cfg->eth_data()->supply_2V_data),
  m_supply_1A(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_1A_pins,
    &ap_cfg->eth_data()->supply_1A_data),
  m_supply_17A(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_17A_pins,
    &ap_cfg->eth_data()->supply_17A_data)
{
}

void u309m::app_t::tick()
{
  mp_cfg->tick();
  m_meas_comm_app.tick();
  
  //m_supply_comm_app.tick();
  m_supply_200V.tick();
  m_supply_20V.tick();
  m_supply_2V.tick();
  m_supply_1A.tick();
  m_supply_17A.tick();
}
