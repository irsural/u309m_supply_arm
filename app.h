#ifndef apph
#define apph

#include <irsdefs.h>

#include <irsadc.h>

#include "supply.h"
#include "cfg.h"

#include <irsfinal.h>

namespace u309m
{

class app_t
{
public:
  app_t(cfg_t* ap_cfg);
  void tick();
private:
  enum mode_t {
    rele_check_mode,
    rele_voltage_off_mode
  };

  cfg_t* mp_cfg;
  #ifdef MEAS_COMM_TEST
  meas_comm_t* mp_meas_comm;
  #endif // MEAS_COMM_TEST
  #ifdef SUPPLY_COMM_TEST
  supply_comm_t* mp_supply_comm;
  #endif // SUPPLY_COMM_TEST
  #ifdef SUPPLY_TEST
  supply_t m_supply_20V;
  supply_t m_supply_200V;
  supply_t m_supply_2V;
  supply_t m_supply_1A;
  supply_t m_supply_17A;
  #endif // SUPPLY_TEST
  bool m_bistable_rele_change;
  mode_t m_mode;
  irs::timer_t m_rele_timer;
  bool m_SYM_2V;
  bool m_SYM_20V;
  bool m_SYM_200V;
  bool m_KZ_2V;
  //
  irs::timer_t m_rel_220V_timer;
}; // app_t

} // namespace u309m

#endif // apph
