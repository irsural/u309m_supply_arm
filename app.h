#ifndef apph
#define apph

#include <irsdefs.h>

#include <irsadc.h>
#include <irsdev.h>

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
  enum status_t
  {
    OFF = 0,
    ON = 1,
    START = 2
  };
  enum
  {
    m_alarm_mask = 0x7FFFFFF,
    m_unlock_command = 116,
    m_clear_alarm_command = 207
  };
  class check_value_t
  {
  public:
    check_value_t(const irs::conn_data_t<float> &a_value,
      const float a_min, const float a_max):
      m_value(a_value),
      m_min(a_min),
      m_max(a_max),
      m_alarm(false)
    {}
    bool alarm()
    {
      float value = m_value;
      bool min = value < m_min;
      bool max = value > m_max;
      m_alarm |= (min || max);
      return m_alarm;
    }
    void clear_alarm() { m_alarm = false; }
  private:
    const irs::conn_data_t<float> &m_value;
    const float m_min;
    const float m_max;
    bool m_alarm;
  };
  void clear_all_alarms();

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
  bool m_SYM_OFF;
  
  irs_u8 m_ip_0;
  irs_u8 m_ip_1;
  irs_u8 m_ip_2;
  irs_u8 m_ip_3;
  //
  irs::timer_t m_rel_220V_timer;
  //
  check_value_t m_internal_th_value;
  check_value_t m_ptc_a_value;
  check_value_t m_ptc_lc_value;
  check_value_t m_ptc_pwr_value;
  check_value_t m_ptc_17A_value;
  check_value_t m_tr_24V_value;
  check_value_t m_24V_value;
  check_value_t m_5V_value;
  check_value_t m_izm_6V_value;
  check_value_t m_izm_3_3V_value;
  check_value_t m_izm_1_2V_value;
  check_value_t m_izm_th1_value;
  check_value_t m_izm_th2_value;
  check_value_t m_izm_th3_value;
  check_value_t m_izm_th4_value;
  check_value_t m_izm_th5_value;
  check_value_t m_200V_th_base_value;
  check_value_t m_200V_th_aux_value;
  check_value_t m_20V_th_base_value;
  check_value_t m_20V_th_aux_value;
  check_value_t m_2V_th_base_value;
  check_value_t m_2V_th_aux_value;
  check_value_t m_1A_th_base_value;
  check_value_t m_1A_th_aux_value;
  check_value_t m_17A_th_base_value;
  check_value_t m_17A_th_aux_value;
  irs::loop_timer_t m_alarm_timer;
  irs::timer_t m_start_alarm_timer;
  irs::timer_t m_upper_level_connect_timer;
  status_t m_status;
  irs_u32 m_connect_counter;
  bool m_upper_level_unconnected;
  bool m_refresh_timeout;
  irs::timer_t m_refresh_timer;
  
  irs::arm::watchdog_timer_t m_watchdog;
}; // app_t

} // namespace u309m

#endif // apph
