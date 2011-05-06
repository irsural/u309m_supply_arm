#ifndef apph
#define apph

#include <irsdefs.h>

#include <irsadc.h>

#include "supply.h"
#include "comm.h"
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
  meas_comm_t m_meas_comm_app;
  //supply_comm_t m_supply_comm_app;
  /*supply_t m_supply_200V;
  supply_t m_supply_20V;
  supply_t m_supply_2V;
  supply_t m_supply_1A;
  supply_t m_supply_17A;*/
  bool m_bistable_rele_change;
  mode_t m_mode;
  irs::timer_t m_rele_timer;
  bool m_SYM_2V;
  bool m_SYM_20V;
  bool m_SYM_200V;
  bool m_KZ_2V;
}; // app_t

} // namespace u309m

#endif // apph
