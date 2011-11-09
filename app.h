#ifndef apph
#define apph

#include <irsdefs.h>

#include <irsadc.h>
#include <irsdev.h>

#include "supply.h"
#include "cfg.h"
#include "comm.h"

#include <irsfinal.h>

namespace u309m
{

class meas_comm_th_t
{
  public:
    meas_comm_th_t(
      irs::spi_t* ap_spi,
      meas_comm_th_pins_t* ap_pins,
      meas_comm_th_data_t& a_data,
      irs::bit_data_t& a_ee_izm_th_spi_enable,
      irs::bit_data_t& a_eth_izm_th_spi_enable);
    void tick();
    inline bool operated() { return m_izm_th_spi_enable & m_th_new_data; }
  private:
    const counter_t m_th_interval;
    meas_comm_th_data_t& m_data;
    irs::th_lm95071_t m_th1;
    irs::th_lm95071_data_t m_th1_data;
    irs::th_lm95071_t m_th2;
    irs::th_lm95071_data_t m_th2_data;
    irs::th_lm95071_t m_th3;
    irs::th_lm95071_data_t m_th3_data;
    irs::th_lm95071_t m_th4;
    irs::th_lm95071_data_t m_th4_data;
    irs::th_lm95071_t m_th5;
    irs::th_lm95071_data_t m_th5_data;
    irs::loop_timer_t m_timer;
    irs::bit_data_t& m_ee_izm_th_spi_enable;
    irs::bit_data_t& m_eth_izm_th_spi_enable;
    bool m_izm_th_spi_enable;
    bool m_th_new_data;
    irs::gpio_pin_t& m_izm_th_enable_pin;
};

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
    m_clear_alarm_command = 207,
    m_fail_max = 5
  };
  class check_value_t
  {
  public:
    check_value_t(const irs::conn_data_t<float> &a_value,
      const float a_min, const float a_max):
      m_value(a_value),
      m_min(a_min),
      m_max(a_max),
      m_fail_cnt(0)
    {}
    bool alarm()
    {
      float value = m_value;
      bool min = value < m_min;
      bool max = value > m_max;
      if (min || max) {
        m_fail_cnt++;
      } else {
        m_fail_cnt = 0;
      }
      return (m_fail_cnt >= m_fail_max);
    }
    void clear_alarm() { m_fail_cnt = 0; }
  private:
    const irs::conn_data_t<float> &m_value;
    const float m_min;
    const float m_max;
    irs_u8 m_fail_cnt;
  };
  void clear_all_alarms();

  cfg_t* mp_cfg;
  supply_t m_supply_200V;
  supply_t m_supply_20V;
  supply_t m_supply_2V;
  supply_t m_supply_1A;
  supply_t m_supply_17A;
  bool m_bistable_rele_change;
  mode_t m_mode;
  irs::timer_t m_rele_timer;
  bool m_SYM_2V;
  bool m_SYM_20V;
  bool m_SYM_200V;
  bool m_KZ_2V;
  bool m_SYM_OFF;
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

  irs::loop_timer_t m_eth_data_refresh_timer;

  meas_comm_th_t m_meas_comm_th;

  meas_comm_t m_meas_comm;
  supply_comm_t m_supply_comm;
}; // app_t

} // namespace u309m

#endif // apph
