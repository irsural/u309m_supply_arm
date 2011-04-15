#ifndef apph
#define apph

#include <irsdefs.h>

#include <irsadc.h>

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
  cfg_t* mp_cfg;
  meas_comm_t m_meas_comm_app;
  //supply_comm_t m_supply_comm_app;
  // supply_t m_supply_200V;
  // supply_t m_supply_20V;
  // supply_t m_supply_2V;
  // supply_t m_supply_1A;
  // supply_t m_supply_17A;
}; // app_t

} // namespace u309m

#endif // apph
