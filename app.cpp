#include <irsdefs.h>

#include "app.h"

#include <irsfinal.h>

//---------------------------- main application --------------------------------

u309m::app_t::app_t(cfg_t* ap_cfg):
  mp_cfg(ap_cfg),
  m_modbus_server(mp_cfg->hardflow(), 0, 14, 323, 0, irs::make_cnt_ms(200)),
  m_eth_data(&m_modbus_server),
  m_eeprom_data(mp_cfg->eeprom()),

  m_supply_200V(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_200V,
    &m_eth_data.supply_200V,
    &m_eeprom_data.supply_200V),
  m_supply_20V(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_20V,
    &m_eth_data.supply_20V,
    &m_eeprom_data.supply_20V),
  m_supply_2V(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_2V,
    &m_eth_data.supply_2V,
    &m_eeprom_data.supply_2V),
  m_supply_1A(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_1A,
    &m_eth_data.supply_1A,
    &m_eeprom_data.supply_1A),
  m_supply_17A(ap_cfg->spi_general_purpose(),
    ap_cfg->command_pins()->supply_17A,
    &m_eth_data.supply_17A,
    &m_eeprom_data.supply_17A),
  m_bistable_rele_change(false),
  m_mode(rele_check_mode),
  m_rele_timer(irs::make_cnt_ms(10)),
  m_SYM_2V(m_eth_data.rele_ext.SYM_2V),
  m_SYM_20V(m_eth_data.rele_ext.SYM_20V),
  m_SYM_200V(m_eth_data.rele_ext.SYM_200V),
  m_KZ_2V(m_eth_data.rele_ext.KZ_2V),
  m_SYM_OFF(m_eth_data.rele_ext.SYM_OFF),
  //
  m_rel_220V_timer(irs::make_cnt_s(1)),
  //  check
  m_internal_th_value(m_eth_data.arm_adc.internal_temp, 18.f, 50.f),
  m_ptc_a_value(m_eth_data.arm_adc.PTC_A, 0.f, 0.33f),
  m_ptc_lc_value(m_eth_data.arm_adc.PTC_LC, 0.f, 0.33f),
  m_ptc_pwr_value(m_eth_data.arm_adc.PTC_PWR, 0.f, 0.33f),
  m_ptc_17A_value(m_eth_data.arm_adc.PTC_17A, 0.f, 0.33f),
  m_tr_24V_value(m_eth_data.arm_adc.TR_24V_TEST, 23.f, 25.f),
  m_24V_value(m_eth_data.arm_adc.TEST_24V, 23.f, 25.f),
  m_5V_value(m_eth_data.arm_adc.TEST_5V, 4.5f, 5.5f),
  m_izm_6V_value(m_eth_data.arm_adc.IZM_6V_TEST, 4.5f, 6.5f),
  m_izm_3_3V_value(m_eth_data.arm_adc.IZM_3_3V_TEST, 3.15f, 3.45f),
  m_izm_1_2V_value(m_eth_data.arm_adc.IZM_1_2V_TEST, 1.1f, 1.3f),
  m_izm_th1_value(m_eth_data.meas_comm_th.th1_value, 18.f, 45.f),
  m_izm_th2_value(m_eth_data.meas_comm_th.th2_value, 18.f, 45.f),
  m_izm_th3_value(m_eth_data.meas_comm_th.th3_value, 18.f, 45.f),
  m_izm_th4_value(m_eth_data.meas_comm_th.th4_value, 18.f, 45.f),
  m_izm_th5_value(m_eth_data.meas_comm_th.th5_value, 18.f, 45.f),
  m_200V_th_base_value(m_eth_data.supply_200V.base_temp_data.value,
    18.f, 85.f),
  m_200V_th_aux_value(m_eth_data.supply_200V.aux_temp_data.value,
    18.f, 85.f),
  m_20V_th_base_value(m_eth_data.supply_20V.base_temp_data.value,
    18.f, 85.f),
  m_20V_th_aux_value(m_eth_data.supply_20V.aux_temp_data.value,
    18.f, 85.f),
  m_2V_th_base_value(m_eth_data.supply_2V.base_temp_data.value,
    18.f, 85.f),
  m_2V_th_aux_value(m_eth_data.supply_2V.aux_temp_data.value,
    18.f, 85.f),
  m_1A_th_base_value(m_eth_data.supply_1A.base_temp_data.value,
    18.f, 85.f),
  m_1A_th_aux_value(m_eth_data.supply_1A.aux_temp_data.value,
    18.f, 85.f),
  m_17A_th_base_value(m_eth_data.supply_17A.base_temp_data.value,
    18.f, 90.f),
  m_17A_th_aux_value(m_eth_data.supply_17A.aux_temp_data.value,
    18.f, 120.f),
  m_alarm_timer(irs::make_cnt_ms(100)),
  m_start_alarm_timer(irs::make_cnt_s(5)),
  m_upper_level_connect_timer(irs::make_cnt_s(5)),
  m_status(START),
  m_connect_counter(0),
  m_upper_level_unconnected(false),
  m_refresh_timeout(false),
  m_refresh_timer(irs::make_cnt_s(1)),
  m_watchdog(5),
  m_eth_data_refresh_timer(irs::make_cnt_ms(300)),
  m_meas_comm_th(
    mp_cfg->spi_general_purpose(),
    mp_cfg->meas_comm_th_pins(),
    m_eth_data.meas_comm_th,
    m_eeprom_data.izm_th_spi_enable,
    m_eth_data.control.izm_th_spi_enable),

  m_supply_plis(mp_cfg->supply_comm_pins(), mp_cfg->supply_tact_gen(),
    *mp_cfg->spi_general_purpose()),
  m_supply_comm(m_supply_plis, &m_eth_data.supply_comm.apply,
    &m_eth_data.supply_comm.reset),
  m_supply_plis_debug_check(m_supply_plis, m_eth_data.supply_comm.debug,
    m_eeprom_data.supply_comm_debug),

  #ifdef  OLD_MEAS_COMM
  m_meas_comm(
    mp_cfg->spi_meas_comm_plis(),
    mp_cfg->meas_pins(),
    &m_eth_data.meas_comm
  )
  #else //!OLD_MEAS_COMM
  m_meas_plis(mp_cfg->meas_comm_pins(), mp_cfg->meas_tact_gen(),
    *mp_cfg->spi_meas_comm_plis()),
  m_meas_comm(m_meas_plis, &m_eth_data.meas_comm.apply,
    &m_eth_data.meas_comm.reset),
  m_meas_plis_debug_check(m_meas_plis, m_eth_data.meas_comm.debug,
    m_eeprom_data.meas_comm_debug)
  #endif  //  OLD_MEAS_COMM
{
  m_rel_220V_timer.start();
  mp_cfg->rele_ext_pins()->SYM_OFF->set();

  if (mp_cfg->eeprom_error()) {
    m_eeprom_data.reset_to_default();
  }

  m_eth_data.ip_0 = m_eeprom_data.ip_0;
  m_eth_data.ip_1 = m_eeprom_data.ip_1;
  m_eth_data.ip_2 = m_eeprom_data.ip_2;
  m_eth_data.ip_3 = m_eeprom_data.ip_3;

  mxip_t ip = mxip_t::zero_ip();
  ip.val[0] = m_eeprom_data.ip_0;
  ip.val[1] = m_eeprom_data.ip_1;
  ip.val[2] = m_eeprom_data.ip_2;
  ip.val[3] = m_eeprom_data.ip_3;
  char ip_str[IP_STR_LEN];
  mxip_to_cstr(ip_str, ip);
  mp_cfg->hardflow()->set_param("local_addr", ip_str);
  m_eth_data.control.on = 0;
  m_alarm_timer.start();
  m_start_alarm_timer.start();
  m_eth_data.control.upper_level_check =
    m_eeprom_data.upper_level_check;

  m_supply_17A.disable_saving_aux_th_ref();

  m_eth_data.control.watchdog_reset_cause =
    m_watchdog.watchdog_reset_cause();
  //m_watchdog.start();
  enum {
    ERROR_POS = 0,
    ON_POS = 2,
    POL_CH_POS = 3,
    POL_ET_POS = 4,
    CHECKED_POS = 5,
    ETALON_POS = 9,
    SUPPLY_POS =  13
  };
  m_supply_comm.add_byte(&m_eth_data.supply_comm.supply_index, SUPPLY_POS);
  m_supply_comm.add_byte(&m_eth_data.supply_comm.etalon_cell, ETALON_POS);
  m_supply_comm.add_byte(&m_eth_data.supply_comm.calibrated_cell, CHECKED_POS);
  m_supply_comm.add_bit(&m_eth_data.supply_comm.polarity_etalon, POL_ET_POS);
  m_supply_comm.add_bit(&m_eth_data.supply_comm.polarity_calibrated,POL_CH_POS);
  m_supply_comm.add_bit(&m_eth_data.supply_comm.on, ON_POS, true);
  m_supply_comm.add_flag(&m_eth_data.supply_comm.error, ERROR_POS);
  m_supply_comm.on();
  #ifndef OLD_MEAS_COMM
  enum {
    MEAS_MODE_POS = 14,
    MEAS_ETALON_POS = 10,
    MEAS_CHECKED_POS = 6,
    MEAS_LOAD_POS = 5
  };
  m_meas_comm.add_byte(&m_eth_data.meas_comm.mode, MEAS_MODE_POS);
  m_meas_comm.add_byte(&m_eth_data.meas_comm.etalon_cell, MEAS_ETALON_POS);
  m_meas_comm.add_byte(&m_eth_data.meas_comm.calibrated_cell, MEAS_CHECKED_POS);
  m_meas_comm.add_bit(&m_eth_data.meas_comm.load_resistor, MEAS_LOAD_POS);
  m_meas_comm.add_bit(&m_eth_data.meas_comm.on, ON_POS, true);
  m_meas_comm.add_flag(&m_eth_data.meas_comm.error, ERROR_POS);
  m_meas_comm.on();
  #endif  //  OLD_MEAS_COMM
}

void u309m::app_t::tick()
{
  m_supply_plis.tick();
  m_supply_comm.tick();
  m_supply_plis_debug_check.tick();

  #ifdef  OLD_MEAS_COMM
  m_meas_comm.tick();
  #else //  !OLD_MEAS_COMM
  m_meas_plis.tick();
  m_meas_comm.tick();
  m_meas_plis_debug_check.tick();
  #endif  //  OLD_MEAS_COMM

  m_supply_200V.tick();
  m_supply_20V.tick();
  m_supply_2V.tick();
  m_supply_1A.tick();
  m_supply_17A.tick();

  mp_cfg->adc()->tick();
  mp_cfg->eeprom()->tick();

  m_meas_comm_th.tick();
  m_modbus_server.tick();

  #ifndef NOP
  bool change_ip_0 =
    (m_eeprom_data.ip_0 != m_eth_data.ip_0);
  bool change_ip_1 =
    (m_eeprom_data.ip_1 != m_eth_data.ip_1);
  bool change_ip_2 =
    (m_eeprom_data.ip_2 != m_eth_data.ip_2);
  bool change_ip_3 =
    (m_eeprom_data.ip_3 != m_eth_data.ip_3);

  if (change_ip_0 || change_ip_1 || change_ip_2 || change_ip_3)
  {
    mxip_t ip = mxip_t::zero_ip();

    m_eeprom_data.ip_0 = m_eth_data.ip_0;
    m_eeprom_data.ip_1 = m_eth_data.ip_1;
    m_eeprom_data.ip_2 = m_eth_data.ip_2;
    m_eeprom_data.ip_3 = m_eth_data.ip_3;

    ip.val[0] = m_eth_data.ip_0;
    ip.val[1] = m_eth_data.ip_1;
    ip.val[2] = m_eth_data.ip_2;
    ip.val[3] = m_eth_data.ip_3;
    char ip_str[IP_STR_LEN];
    mxip_to_cstr(ip_str, ip);
    mp_cfg->hardflow()->set_param("local_addr", ip_str);
  }
  #endif //NOP

  if (m_rel_220V_timer.check())
  {
    mp_cfg->rele_ext_pins()->REL_220V->set();
    m_eth_data.rele_ext.REL_220V = 1;
  }

  switch (m_mode)
  {
    case rele_check_mode:
    {
      if (m_SYM_2V != static_cast<bool>(m_eth_data.rele_ext.SYM_2V))
      {
        if (m_SYM_OFF)
        {
          m_SYM_2V = false;
          m_eth_data.rele_ext.SYM_2V = 0;
        }
        else
        {
          m_bistable_rele_change = true;
          m_SYM_2V = m_eth_data.rele_ext.SYM_2V;
          if (m_SYM_2V) {
            mp_cfg->rele_ext_pins()->SYM_2V_on->set();
            mp_cfg->rele_ext_pins()->SYM_2V_off->clear();
          } else {
            mp_cfg->rele_ext_pins()->SYM_2V_on->clear();
            mp_cfg->rele_ext_pins()->SYM_2V_off->set();
          }
        }
      }
      if (m_SYM_20V != static_cast<bool>(m_eth_data.rele_ext.SYM_20V))
      {
        if (m_SYM_OFF)
        {
          m_SYM_20V = false;
          m_eth_data.rele_ext.SYM_20V = 0;
        }
        else
        {
          m_bistable_rele_change = true;
          m_SYM_20V = m_eth_data.rele_ext.SYM_20V;
          if (m_SYM_20V) {
            mp_cfg->rele_ext_pins()->SYM_20V_on->set();
            mp_cfg->rele_ext_pins()->SYM_20V_off->clear();
          } else {
            mp_cfg->rele_ext_pins()->SYM_20V_on->clear();
            mp_cfg->rele_ext_pins()->SYM_20V_off->set();
          }
        }
      }
      if (m_SYM_200V != static_cast<bool>(m_eth_data.rele_ext.SYM_200V))
      {
        if (m_SYM_OFF)
        {
          m_SYM_200V = false;
          m_eth_data.rele_ext.SYM_200V = 0;
        }
        else
        {
          m_bistable_rele_change = true;
          m_SYM_200V = m_eth_data.rele_ext.SYM_200V;
          if (m_SYM_200V) {
            mp_cfg->rele_ext_pins()->SYM_200V_on->set();
            mp_cfg->rele_ext_pins()->SYM_200V_off->clear();
          } else {
            mp_cfg->rele_ext_pins()->SYM_200V_on->clear();
            mp_cfg->rele_ext_pins()->SYM_200V_off->set();
          }
        }
      }
      if (m_KZ_2V != static_cast<bool>(m_eth_data.rele_ext.KZ_2V))
      {
        if (m_SYM_OFF)
        {
          m_KZ_2V = false;
          m_eth_data.rele_ext.KZ_2V = 0;
        }
        else
        {
          m_bistable_rele_change = true;
          m_KZ_2V = m_eth_data.rele_ext.KZ_2V;
          if (m_KZ_2V) {
            mp_cfg->rele_ext_pins()->KZ_2V_on->set();
            mp_cfg->rele_ext_pins()->KZ_2V_off->clear();
          } else {
            mp_cfg->rele_ext_pins()->KZ_2V_on->clear();
            mp_cfg->rele_ext_pins()->KZ_2V_off->set();
          }
        }
      }
      if (m_eth_data.rele_ext.KZ_1A) {
        mp_cfg->rele_ext_pins()->KZ_1A->set();
      } else {
        mp_cfg->rele_ext_pins()->KZ_1A->clear();
      }
      if (m_eth_data.rele_ext.KZ_17A) {
        mp_cfg->rele_ext_pins()->KZ_17A->set();
      } else {
        mp_cfg->rele_ext_pins()->KZ_17A->clear();
      }
      if (m_eth_data.rele_ext.REL_220V) {
        mp_cfg->rele_ext_pins()->REL_220V->set();
      } else {
        mp_cfg->rele_ext_pins()->REL_220V->clear();
      }

      if (static_cast<bool>(m_eth_data.rele_ext.SYM_OFF) != m_SYM_OFF)
      {
        m_SYM_OFF = m_eth_data.rele_ext.SYM_OFF;
        if (m_SYM_OFF)
        {
          mp_cfg->rele_ext_pins()->SYM_OFF->clear();
          mp_cfg->rele_ext_pins()->SYM_2V_on->clear();
          mp_cfg->rele_ext_pins()->SYM_2V_off->clear();
          mp_cfg->rele_ext_pins()->SYM_20V_on->clear();
          mp_cfg->rele_ext_pins()->SYM_20V_off->clear();
          mp_cfg->rele_ext_pins()->SYM_200V_on->clear();
          mp_cfg->rele_ext_pins()->SYM_200V_off->clear();
          mp_cfg->rele_ext_pins()->KZ_2V_on->clear();
          mp_cfg->rele_ext_pins()->KZ_2V_off->clear();
          m_KZ_2V = false;
          m_eth_data.rele_ext.KZ_2V = 0;
          m_SYM_2V = false;
          m_eth_data.rele_ext.SYM_2V = 0;
          m_SYM_20V = false;
          m_eth_data.rele_ext.SYM_20V = 0;
          m_SYM_200V = false;
          m_eth_data.rele_ext.SYM_200V = 0;
          mp_cfg->rele_ext_pins()->KZ_2V_on->clear();
          mp_cfg->rele_ext_pins()->KZ_2V_off->clear();
          mp_cfg->rele_ext_pins()->SYM_2V_on->clear();
          mp_cfg->rele_ext_pins()->SYM_2V_off->clear();
          mp_cfg->rele_ext_pins()->SYM_20V_on->clear();
          mp_cfg->rele_ext_pins()->SYM_20V_off->clear();
          mp_cfg->rele_ext_pins()->SYM_200V_on->clear();
          mp_cfg->rele_ext_pins()->SYM_200V_off->clear();
          m_bistable_rele_change = false;
          m_mode = rele_check_mode;
        }
        else
        {
          mp_cfg->rele_ext_pins()->SYM_OFF->set();
        }
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
  m_eth_data.rele_ext.SYM_OFF_TEST =
    mp_cfg->rele_ext_pins()->SYM_OFF_TEST->pin();

  if (m_alarm_timer.check())
  {
    if (!m_refresh_timeout)
    {
      if (m_eth_data.control.refresh_all_sources == 1)
      {
        //irs::mlog() << "ќбновление уставок всех источников начато" << endl;
        m_supply_200V.refresh_dac_values();
        m_supply_20V.refresh_dac_values();
        m_supply_2V.refresh_dac_values();
        m_supply_1A.refresh_dac_values();
        m_supply_17A.refresh_dac_values();

        m_refresh_timeout = true;
        m_refresh_timer.start();
      }
    }
    else
    {
      if (m_refresh_timer.check())
      {
        m_eth_data.control.refresh_all_sources = 0;
        m_refresh_timeout = false;
        //irs::mlog() << "ќбновление уставок всех источников закончено" << endl;
      }
    }

    if (m_status != START) {
      bool internal_th_alarm = m_internal_th_value.alarm();
      m_eth_data.control.alarm_internal_th = internal_th_alarm;

      bool ptc_a_alarm = m_ptc_a_value.alarm();
      m_eth_data.control.alarm_ptc_a = ptc_a_alarm;

      bool ptc_lc_alarm = m_ptc_lc_value.alarm();
      m_eth_data.control.alarm_ptc_lc = ptc_lc_alarm;

      bool ptc_pwr_alarm = m_ptc_pwr_value.alarm();
      m_eth_data.control.alarm_ptc_pwr = ptc_pwr_alarm;

      bool ptc_17A_alarm = m_ptc_17A_value.alarm();
      m_eth_data.control.alarm_ptc_17A = ptc_17A_alarm;

      bool tr_24V_alarm = m_tr_24V_value.alarm();
      m_eth_data.control.alarm_tr_24V = tr_24V_alarm;

      bool v_24V_alarm = m_24V_value.alarm();
      m_eth_data.control.alarm_24V = v_24V_alarm;

      bool v_5V_alarm = m_5V_value.alarm();
      m_eth_data.control.alarm_5V = v_5V_alarm;

      bool izm_6V_alarm = m_izm_6V_value.alarm();
      m_eth_data.control.alarm_izm_6V = izm_6V_alarm;

      bool izm_3_3V_alarm = m_izm_3_3V_value.alarm();
      m_eth_data.control.alarm_izm_3_3V = izm_3_3V_alarm;

      bool izm_1_2V_alarm = m_izm_1_2V_value.alarm();
      m_eth_data.control.alarm_izm_1_2V = izm_1_2V_alarm;

      if (m_meas_comm_th.operated()) {
        bool izm_th1_alarm = m_izm_th1_value.alarm();
        m_eth_data.control.alarm_izm_th1 = izm_th1_alarm;

        bool izm_th2_alarm = m_izm_th2_value.alarm();
        m_eth_data.control.alarm_izm_th2 = izm_th2_alarm;

        bool izm_th3_alarm = m_izm_th3_value.alarm();
        m_eth_data.control.alarm_izm_th3 = izm_th3_alarm;

        bool izm_th4_alarm = m_izm_th4_value.alarm();
        m_eth_data.control.alarm_izm_th4 = izm_th4_alarm;

        bool izm_th5_alarm = m_izm_th5_value.alarm();
        m_eth_data.control.alarm_izm_th5 = izm_th5_alarm;
      }

      bool t_200V_th_base_alarm = m_200V_th_base_value.alarm();
      m_eth_data.control.alarm_200V_th_base
        = t_200V_th_base_alarm;

      bool t_200V_th_aux_alarm = m_200V_th_aux_value.alarm();
      m_eth_data.control.alarm_200V_th_aux
        = t_200V_th_aux_alarm;

      bool t_20V_th_base_alarm = m_20V_th_base_value.alarm();
      m_eth_data.control.alarm_20V_th_base
        = t_20V_th_base_alarm;

      bool t_20V_th_aux_alarm = m_20V_th_aux_value.alarm();
      m_eth_data.control.alarm_20V_th_aux
        = t_20V_th_aux_alarm;

      bool t_2V_th_base_alarm = m_2V_th_base_value.alarm();
      m_eth_data.control.alarm_2V_th_base
        = t_2V_th_base_alarm;

      bool t_2V_th_aux_alarm = m_2V_th_aux_value.alarm();
      m_eth_data.control.alarm_2V_th_aux
        = t_2V_th_aux_alarm;

      bool t_1A_th_base_alarm = m_1A_th_base_value.alarm();
      m_eth_data.control.alarm_1A_th_base
        = t_1A_th_base_alarm;

      bool t_1A_th_aux_alarm = m_1A_th_aux_value.alarm();
      m_eth_data.control.alarm_1A_th_aux
        = t_1A_th_aux_alarm;

      bool t_17A_th_base_alarm = m_17A_th_base_value.alarm();
      m_eth_data.control.alarm_17A_th_base
        = t_17A_th_base_alarm;

      bool t_17A_th_aux_alarm = m_17A_th_aux_value.alarm();
      m_eth_data.control.alarm_17A_th_aux
        = t_17A_th_aux_alarm;
    }

    m_eth_data.control.alarm_upper_level = m_upper_level_unconnected;

    volatile irs_u32 alarm = m_eth_data.control.alarm;
    switch (m_status)
    {
      case START:
      {
        if (m_start_alarm_timer.check())
        {
          clear_all_alarms();
          m_status = ON;
          m_eth_data.control.on = 1;
          m_meas_comm.on();
          m_supply_comm.on();
          m_supply_200V.on();
          m_supply_20V.on();
          m_supply_2V.on();
          m_supply_1A.on();
          m_supply_17A.on();
        }
        break;
      }
      case OFF:
      {
        if (!(m_eth_data.control.alarm & m_alarm_mask)
          || m_eth_data.control.unlock == m_unlock_command)
        {
          m_status = ON;
          m_eth_data.control.on = 1;
          m_meas_comm.on();
          m_supply_comm.on();
          m_supply_200V.on();
          m_supply_20V.on();
          m_supply_2V.on();
          m_supply_1A.on();
          m_supply_17A.on();
        }
        if (m_eth_data.control.on)
        {
          m_eth_data.control.on = 0;
        }
        break;
      }
      case ON:
      {
        if ((m_eth_data.control.alarm & m_alarm_mask)
          && !(m_eth_data.control.unlock == m_unlock_command))
        {
          m_status = OFF;
          m_eth_data.control.on = 0;
          m_meas_comm.off();
          m_supply_comm.off();
          m_supply_200V.off();
          m_supply_20V.off();
          m_supply_2V.off();
          m_supply_1A.off();
          m_supply_17A.off();
          m_eth_data.supply_17A.aux_tr_data.temperature_ref = 0;
        }
        if (!m_eth_data.control.on)
        {
          m_eth_data.control.on = 1;
        }
        break;
      }
    }

    if (m_eth_data.control.unlock == m_clear_alarm_command)
    {
      m_eth_data.control.unlock = 0;
      clear_all_alarms();
    }

    /*static volatile int i = 0;
    if (mp_cfg->eth_data()->control.upper_level_check != i)
    {
      i = mp_cfg->eth_data()->control.upper_level_check;
      irs::mlog() << "i = " << i << endl;
    }*/
    if (m_eth_data.control.upper_level_check !=
      m_eeprom_data.upper_level_check)
    {
      m_eeprom_data.upper_level_check =
        m_eth_data.control.upper_level_check;
      irs::mlog() << "eeprom = "
        << m_eeprom_data.upper_level_check << endl;
      if (m_eth_data.control.upper_level_check)
      {
        m_connect_counter = m_eth_data.control.connect_counter;
        m_upper_level_unconnected = false;
        m_upper_level_connect_timer.start();
      }
    }
    if (m_eeprom_data.upper_level_check)
    {
      if (m_connect_counter != m_eth_data.control.connect_counter)
      {
        m_connect_counter = m_eth_data.control.connect_counter;
        m_upper_level_connect_timer.start();
      }
      if (m_upper_level_connect_timer.check())
      {
        m_upper_level_unconnected = true;
      }
    }
    if (!m_eth_data.control.watchdog_test)
    {
      m_watchdog.restart();
    }
  }

  if (m_eth_data_refresh_timer.check()) {
    enum {
      PTC_A_num = 9,
      PTC_LC_num = 8,
      TR_24V_TEST_num = 7,
      IZM_3_3V_TEST_num = 6,
      IZM_6V_TEST_num = 5,
      IZM_1_2V_TEST_num = 4,
      TEST_24V_num = 3,
      TEST_5V_num = 2,
      PTC_PWR_num = 1,
      PTC_17A_num = 0
    };
    m_eth_data.arm_adc.PTC_A
      = mp_cfg->adc()->get_float_data(PTC_A_num);
    m_eth_data.arm_adc.PTC_LC
      = mp_cfg->adc()->get_float_data(PTC_LC_num);
    m_eth_data.arm_adc.TR_24V_TEST
      = mp_cfg->adc()->get_float_data(TR_24V_TEST_num) * 36.348f;
    m_eth_data.arm_adc.IZM_3_3V_TEST
      = mp_cfg->adc()->get_float_data(IZM_3_3V_TEST_num) * 9.446f;
    m_eth_data.arm_adc.IZM_6V_TEST
      = mp_cfg->adc()->get_float_data(IZM_6V_TEST_num) * 9.313f;
    m_eth_data.arm_adc.IZM_1_2V_TEST
      = mp_cfg->adc()->get_float_data(IZM_1_2V_TEST_num) * 10.028f;
    m_eth_data.arm_adc.TEST_24V
      = mp_cfg->adc()->get_float_data(TEST_24V_num) * 32.999f;
    m_eth_data.arm_adc.TEST_5V
      = mp_cfg->adc()->get_float_data(TEST_5V_num) * 35.163f;
    m_eth_data.arm_adc.PTC_PWR
      = mp_cfg->adc()->get_float_data(PTC_PWR_num);
    m_eth_data.arm_adc.PTC_17A
      = mp_cfg->adc()->get_float_data(PTC_17A_num);
    m_eth_data.arm_adc.internal_temp
      = mp_cfg->adc()->get_temperature();
  }
}

void u309m::app_t::clear_all_alarms()
{
  m_internal_th_value.clear_alarm();
  m_ptc_a_value.clear_alarm();
  m_ptc_lc_value.clear_alarm();
  m_ptc_pwr_value.clear_alarm();
  m_ptc_17A_value.clear_alarm();
  m_tr_24V_value.clear_alarm();
  m_24V_value.clear_alarm();
  m_5V_value.clear_alarm();
  m_izm_6V_value.clear_alarm();
  m_izm_3_3V_value.clear_alarm();
  m_izm_1_2V_value.clear_alarm();
  m_izm_th1_value.clear_alarm();
  m_izm_th2_value.clear_alarm();
  m_izm_th3_value.clear_alarm();
  m_izm_th4_value.clear_alarm();
  m_izm_th5_value.clear_alarm();
  m_200V_th_base_value.clear_alarm();
  m_200V_th_aux_value.clear_alarm();
  m_20V_th_base_value.clear_alarm();
  m_20V_th_aux_value.clear_alarm();
  m_2V_th_base_value.clear_alarm();
  m_2V_th_aux_value.clear_alarm();
  m_1A_th_base_value.clear_alarm();
  m_1A_th_aux_value.clear_alarm();
  m_17A_th_base_value.clear_alarm();
  m_17A_th_aux_value.clear_alarm();
  m_upper_level_unconnected = false;
}

u309m::meas_comm_th_t::meas_comm_th_t(
  irs::spi_t* ap_spi,
  meas_comm_th_pins_t* ap_pins,
  meas_comm_th_data_t& a_data,
  irs::bit_data_t& a_ee_izm_th_spi_enable,
  irs::bit_data_t& a_eth_izm_th_spi_enable):

  m_th_interval(irs::make_cnt_ms(300)),
  m_data(a_data),
  m_th1(ap_spi, ap_pins->termo_sense_1, m_th_interval),
  m_th1_data(&m_th1),
  m_th2(ap_spi, ap_pins->termo_sense_2, m_th_interval),
  m_th2_data(&m_th2),
  m_th3(ap_spi, ap_pins->termo_sense_3, m_th_interval),
  m_th3_data(&m_th3),
  m_th4(ap_spi, ap_pins->termo_sense_4, m_th_interval),
  m_th4_data(&m_th4),
  m_th5(ap_spi, ap_pins->termo_sense_5, m_th_interval),
  m_th5_data(&m_th5),
  m_timer(m_th_interval),
  m_ee_izm_th_spi_enable(a_ee_izm_th_spi_enable),
  m_eth_izm_th_spi_enable(a_eth_izm_th_spi_enable),
  m_izm_th_spi_enable(m_ee_izm_th_spi_enable),
  m_th_new_data(false),
  m_izm_th_enable_pin(*ap_pins->izm_th_enable)
{
  m_eth_izm_th_spi_enable = m_izm_th_spi_enable;
  if (m_izm_th_spi_enable) {
    m_izm_th_enable_pin.set();
    m_th1_data.stop_bit = 0;
    m_th2_data.stop_bit = 0;
    m_th3_data.stop_bit = 0;
    m_th4_data.stop_bit = 0;
    m_th5_data.stop_bit = 0;
  } else {
    m_izm_th_enable_pin.clear();
    m_th1_data.stop_bit = 1;
    m_th2_data.stop_bit = 1;
    m_th3_data.stop_bit = 1;
    m_th4_data.stop_bit = 1;
    m_th5_data.stop_bit = 1;
  }
}

void u309m::meas_comm_th_t::tick()
{
  m_th1.tick();
  m_th2.tick();
  m_th3.tick();
  m_th4.tick();
  m_th5.tick();
  if (m_timer.check())
  {
    m_th_new_data = m_th1_data.new_data_bit & m_th2_data.new_data_bit &
      m_th3_data.new_data_bit & m_th4_data.new_data_bit &
      m_th5_data.new_data_bit;
    m_data.th1_value = m_th1_data.temperature_code * m_th1.get_conv_koef();
    m_data.th2_value = m_th2_data.temperature_code * m_th2.get_conv_koef();
    m_data.th3_value = m_th3_data.temperature_code * m_th3.get_conv_koef();
    m_data.th4_value = m_th4_data.temperature_code * m_th4.get_conv_koef();
    m_data.th5_value = m_th5_data.temperature_code * m_th5.get_conv_koef();

    if (m_izm_th_spi_enable != static_cast<bool>(m_eth_izm_th_spi_enable)) {
      m_izm_th_spi_enable = m_eth_izm_th_spi_enable;
      m_ee_izm_th_spi_enable = m_izm_th_spi_enable;
      if (m_izm_th_spi_enable) {
        m_izm_th_enable_pin.set();
        m_th1_data.stop_bit = 0;
        m_th2_data.stop_bit = 0;
        m_th3_data.stop_bit = 0;
        m_th4_data.stop_bit = 0;
        m_th5_data.stop_bit = 0;
      } else {
        m_izm_th_enable_pin.clear();
        m_th1_data.stop_bit = 1;
        m_th2_data.stop_bit = 1;
        m_th3_data.stop_bit = 1;
        m_th4_data.stop_bit = 1;
        m_th5_data.stop_bit = 1;
      }
    }
  }
}

u309m::plis_debug_check_t::plis_debug_check_t(plis_t& a_plis,
  irs::bit_data_t& a_eth_bit, irs::bit_data_t& a_ee_bit):
  m_plis(a_plis),
  m_eth_bit(a_eth_bit),
  m_ee_bit(a_ee_bit),
  m_recfg_flag(true)
{
  m_plis.add_reset_flag(&m_recfg_flag);
  m_eth_bit = m_ee_bit;
}

u309m::plis_debug_check_t::~plis_debug_check_t()
{
}

void u309m::plis_debug_check_t::tick()
{
  bool change_debug = m_recfg_flag || (m_eth_bit != m_ee_bit);
  if (change_debug && m_plis.ready()) {
    m_recfg_flag = false;
    if (m_eth_bit != m_ee_bit) {
      m_ee_bit = m_eth_bit;
    }
    enum {
      m_debug_on_command = 3,
      m_debug_off_command = 1
    };
    if (m_eth_bit) {
      m_plis.write(m_debug_on_command);
    } else {
      m_plis.write(m_debug_off_command);
    }
  }
}
