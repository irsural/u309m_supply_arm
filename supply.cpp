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
  //m_prev_tr_reg(0),
  //m_fin_tr_reg(0),
  m_temp_base_ki(0),
  m_temp_base_kd(0),
  m_temp_aux_ki(0),
  m_temp_aux_kd(0),
  m_temp_base_pid_data(irs::zero_struct_t<irs::pid_data_t>::get()),
  m_temp_aux_pid_data(irs::zero_struct_t<irs::pid_data_t>::get()),
  m_dt(0.5),
  m_timer_reg(irs::make_cnt_s(m_dt)),
  m_temp_base_isodr(),
  m_temp_base_time_const(0),
  m_temp_aux_isodr(),
  m_temp_aux_time_const(0),
  m_operate(false)
{
  #ifdef EEPROM_TEST
  mp_eth_data->resistance_code = mp_eeprom_data->resistance_code;
  m_ad5293_data.resistance_code = mp_eeprom_data->resistance_code;
  m_tc_write = mp_eeprom_data->resistance_code;
  #else
  mp_eth_data->resistance_code = m_ad5293_data.resistance_code;
  m_tc_write = mp_eeprom_data->resistance_code;
  #endif // EEPROM_TEST
  #ifdef EEPROM_TEST
  m_prev_adc_koef = mp_eeprom_data->koef_adc_volt_prev;
  m_fin_adc_koef = mp_eeprom_data->koef_adc_volt_fin;
  #else
  m_prev_adc_koef = (4.096/1024)*(23.9/3.9)*1.2;
  m_fin_adc_koef = (4.096/1024)*(23.9/3.9)*1.2;
  #endif // EEPROM_TEST
  mp_eth_data->prev_adc_data.koef = m_prev_adc_koef;
  mp_eth_data->fin_adc_data.koef = m_fin_adc_koef;
  #ifdef EEPROM_TEST
  m_prev_dac_koef = mp_eeprom_data->koef_reg_prev;
  m_fin_dac_koef = mp_eeprom_data->koef_reg_fin;
  #else
  m_prev_dac_koef = 2633;
  m_fin_dac_koef = 2702;
  #endif // EEPROM_TEST
  mp_eth_data->prev_dac_data.koef = m_prev_dac_koef;
  mp_eth_data->fin_dac_data.koef = m_fin_dac_koef;

  #ifdef EEPROM_TEST
  m_temp_base_pid_data.k = mp_eeprom_data->temp_base_k;
  mp_eth_data->base_tr_data.temp_k = m_temp_base_pid_data.k;

  m_temp_base_ki = mp_eeprom_data->temp_base_ki;
  m_temp_base_pid_data.ki = m_temp_base_ki * m_dt;
  mp_eth_data->base_tr_data.temp_ki = m_temp_base_ki;

  m_temp_base_kd = mp_eeprom_data->temp_base_kd;
  m_temp_base_pid_data.kd = m_temp_base_kd / m_dt;
  mp_eth_data->base_tr_data.temp_kd = m_temp_base_kd;
  #else
  m_temp_base_pid_data.k = 15000;
  mp_eth_data->base_tr_data.temp_k = m_temp_base_pid_data.k;

  m_temp_base_ki = 0.0015;
  m_temp_base_pid_data.ki = m_temp_base_ki * m_dt;
  mp_eth_data->base_tr_data.temp_ki = m_temp_base_ki;

  m_temp_base_kd = 100;
  m_temp_base_pid_data.kd = m_temp_base_kd / m_dt;
  mp_eth_data->base_tr_data.temp_kd = m_temp_base_kd;
  /*mp_eth_data->base_tr_data.temp_k = m_temp_base_pid_data.k;
  mp_eth_data->base_tr_data.temp_ki = m_temp_base_pid_data.ki/m_dt;
  mp_eth_data->base_tr_data.temp_kd = m_temp_base_pid_data.kd*m_dt;*/
  #endif // EEPROM_TEST
  m_temp_base_pid_data.min = 0;
  m_temp_base_pid_data.max = 65535;
  m_temp_base_pid_data.prev_e = 0.;
  m_temp_base_pid_data.pp_e = 0.;
  m_temp_base_pid_data.prev_out = 0.;
  m_temp_base_pid_data.block_int = 0;
  m_temp_base_pid_data.block_int_ext = 0;
  m_temp_base_pid_data.int_val = 0.;
  m_temp_base_pid_data.k_d_pid = 0.1;

  #ifdef EEPROM_TEST
  m_temp_base_time_const = mp_eeprom_data->temp_base_time_const;
  #else
  m_temp_base_time_const = 20;
  #endif // EEPROM_TEST
  mp_eth_data->base_tr_data.temp_time_const = m_temp_base_time_const;
  #ifdef EEPROM_TEST
  mp_eth_data->base_tr_data.temp_prop_koef =
    mp_eeprom_data->temp_base_prop_koef;
  #else
  mp_eth_data->base_tr_data.temp_prop_koef = 0;
  #endif // EEPROM_TEST
  m_temp_base_isodr.k = mp_eth_data->base_tr_data.temp_prop_koef;
  m_temp_base_isodr.fd.x1 = m_th_base_data.temperature_code*
    m_th_base.get_conv_koef();
  m_temp_base_isodr.fd.y1 = m_temp_base_isodr.fd.x1;
  m_temp_base_isodr.fd.t = mp_eth_data->base_tr_data.temp_time_const;

  #ifdef EEPROM_TEST
  mp_eth_data->base_tr_data.temperature_ref = mp_eeprom_data->temp_base_ref;
  #else
  mp_eth_data->base_tr_data.temperature_ref = 60;
  #endif // EEPROM_TEST

  #ifdef EEPROM_TEST
  m_temp_aux_pid_data.k = mp_eeprom_data->temp_aux_k;
  m_temp_aux_pid_data.ki = mp_eeprom_data->temp_aux_ki * m_dt;
  m_temp_aux_pid_data.kd = mp_eeprom_data->temp_aux_kd / m_dt;
  #else
  m_temp_aux_pid_data.k = 15000;
  m_temp_aux_pid_data.ki = 0.00075;
  m_temp_aux_pid_data.kd = 200;
  #endif // EEPROM_TEST
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
  mp_eth_data->aux_tr_data.temp_ki = m_temp_aux_pid_data.ki / m_dt;
  mp_eth_data->aux_tr_data.temp_kd = m_temp_aux_pid_data.kd * m_dt;

  #ifdef EEPROM_TEST
  m_temp_aux_time_const = mp_eeprom_data->temp_aux_time_const;
  #else
  m_temp_aux_time_const = 20;
  #endif // EEPROM_TEST
  mp_eth_data->aux_tr_data.temp_time_const = m_temp_aux_time_const;
  #ifdef EEPROM_TEST
  mp_eth_data->aux_tr_data.temp_prop_koef =
    mp_eeprom_data->temp_aux_prop_koef;
  #else
  mp_eth_data->aux_tr_data.temp_prop_koef = 0;
  #endif // EEPROM_TEST
  m_temp_aux_isodr.k = mp_eth_data->aux_tr_data.temp_prop_koef;
  m_temp_aux_isodr.fd.x1 = m_th_base_data.temperature_code*
    m_th_aux.get_conv_koef();
  m_temp_aux_isodr.fd.y1 = m_temp_base_isodr.fd.x1;
  m_temp_aux_isodr.fd.t = mp_eth_data->aux_tr_data.temp_time_const;

  #ifdef EEPROM_TEST
  mp_eth_data->aux_tr_data.temperature_ref = mp_eeprom_data->temp_aux_ref;
  #else
  mp_eth_data->aux_tr_data.temperature_ref = 60;
  #endif // EEPROM_TEST
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
      #ifdef EEPROM_TEST
      mp_eeprom_data->resistance_code = m_tc_write;
      #endif  //  EEPROM_TEST
    } else {
      m_ad5293_data.resistance_code = m_tc_write;
      #ifdef EEPROM_TEST
      mp_eeprom_data->resistance_code = m_tc_write;
      #endif  //  EEPROM_TEST
    }
  }

  //  Уставки источника
  const float m_dac_code_max = 65535.f;
  if (m_prev_dac_reg_write != mp_eth_data->prev_dac_data.voltage_code)
  {
    if (m_operate)
    {
      if (mp_eth_data->prev_dac_data.voltage_code < 0.f)
      {
        mp_eth_data->prev_dac_data.voltage_code = m_prev_dac_reg_write;
      }
      else
      {
        float dac_code = 
          mp_eth_data->prev_dac_data.voltage_code * m_prev_dac_koef;
        if (dac_code <= m_dac_code_max)
        {
          m_volt_reg_data.voltage_code_A = static_cast<irs_u16>(dac_code);
        }
        else
        {
          mp_eth_data->prev_dac_data.voltage_code = 
            m_dac_code_max / m_prev_dac_koef;
          m_volt_reg_data.voltage_code_A 
            = static_cast<irs_u16>(m_dac_code_max);
          
        }
        m_prev_dac_reg_write = mp_eth_data->prev_dac_data.voltage_code;
      }
    }
    else
    {
      mp_eth_data->prev_dac_data.voltage_code = m_prev_dac_reg_write;
    }
  }
  if (m_fin_dac_reg_write != mp_eth_data->fin_dac_data.voltage_code)
  {
    if (m_operate)
    {
      if (mp_eth_data->fin_dac_data.voltage_code < 0.f)
      {
        mp_eth_data->fin_dac_data.voltage_code = m_fin_dac_reg_write;
      }
      else
      {
        float dac_code = 
          mp_eth_data->fin_dac_data.voltage_code * m_fin_dac_koef;
        if (dac_code <= m_dac_code_max)
        {
          m_volt_reg_data.voltage_code_B = static_cast<irs_u16>(dac_code);
        }
        else
        {
          mp_eth_data->fin_dac_data.voltage_code = 
            m_dac_code_max / m_fin_dac_koef;
          m_volt_reg_data.voltage_code_B 
            = static_cast<irs_u16>(m_dac_code_max);
          
        }
        m_fin_dac_reg_write = mp_eth_data->fin_dac_data.voltage_code;
      }
    }
    else
    {
      mp_eth_data->fin_dac_data.voltage_code = m_fin_dac_reg_write;
    }
  }
//  if (m_fin_dac_reg_write != mp_eth_data->fin_dac_data.voltage_code)
//  {
//    if (m_operate)
//    {
//      m_fin_dac_reg_write = mp_eth_data->fin_dac_data.voltage_code;
//      m_volt_reg_data.voltage_code_B =
//        static_cast<irs_u16>(m_fin_dac_reg_write*m_fin_dac_koef);
//    }
//    else
//    {
//      mp_eth_data->fin_dac_data.voltage_code = m_fin_dac_reg_write;
//    }
//  }

  //  Коэффициенты уставок источников
  if (m_prev_dac_koef != mp_eth_data->prev_dac_data.koef)
  {
    m_prev_dac_koef = mp_eth_data->prev_dac_data.koef;
    #ifdef EEPROM_TEST
    mp_eeprom_data->koef_reg_prev = m_prev_dac_koef;
    #endif // EEPROM_TEST
    if (m_operate)
    {
      m_volt_reg_data.voltage_code_A =
        static_cast<irs_u16>(m_prev_dac_reg_write*m_prev_dac_koef);
    }
  }
  if (m_fin_dac_koef != mp_eth_data->fin_dac_data.koef)
  {
    m_fin_dac_koef = mp_eth_data->fin_dac_data.koef;
    #ifdef EEPROM_TEST
    mp_eeprom_data->koef_reg_fin = m_fin_dac_koef;
    #endif // EEPROM_TEST
    if (m_operate)
    {
      m_volt_reg_data.voltage_code_B =
        static_cast<irs_u16>(m_fin_dac_reg_write*m_fin_dac_koef);
    }
  }

  //  Коэффициенты АЦП
  if (m_prev_adc_koef != mp_eth_data->prev_adc_data.koef)
  {
    m_prev_adc_koef = mp_eth_data->prev_adc_data.koef;
    #ifdef EEPROM_TEST
    mp_eeprom_data->koef_adc_volt_prev = m_prev_adc_koef;
    #endif // EEPROM_TEST
    mp_eth_data->prev_adc_data.voltage_code =
      m_prev_adc_koef*m_adc102_data.voltage_code_A;
  }
  if (m_fin_adc_koef != mp_eth_data->fin_adc_data.koef)
  {
    m_fin_adc_koef = mp_eth_data->fin_adc_data.koef;
    #ifdef EEPROM_TEST
    mp_eeprom_data->koef_adc_volt_fin = m_fin_adc_koef;
    #endif // EEPROM_TEST
    mp_eth_data->fin_adc_data.voltage_code =
      m_fin_adc_koef*m_adc102_data.voltage_code_B;
  }

  //  Коэффициенты регуляторов температуры
  if (m_temp_base_pid_data.k != mp_eth_data->base_tr_data.temp_k)
  {
    m_temp_base_pid_data.k = mp_eth_data->base_tr_data.temp_k;
    pid_reg_sync(&m_temp_base_pid_data);
    #ifdef EEPROM_TEST
    mp_eeprom_data->temp_base_k = mp_eth_data->base_tr_data.temp_k;
    #endif // EEPROM_TEST
  }

  if (m_temp_base_ki != mp_eth_data->base_tr_data.temp_ki)
  {
    m_temp_base_ki = mp_eth_data->base_tr_data.temp_ki;
    m_temp_base_pid_data.ki = m_temp_base_ki * m_dt;
    pid_reg_sync(&m_temp_base_pid_data);
    #ifdef EEPROM_TEST
    mp_eeprom_data->temp_base_ki = m_temp_base_ki;
    #endif // EEPROM_TEST
  }

  if (m_temp_base_kd != mp_eth_data->base_tr_data.temp_kd)
  {
    m_temp_base_kd = mp_eth_data->base_tr_data.temp_kd;
    m_temp_base_pid_data.kd = m_temp_base_kd / m_dt;
    pid_reg_sync(&m_temp_base_pid_data);
    #ifdef EEPROM_TEST
    mp_eeprom_data->temp_base_kd = m_temp_base_kd;
    #endif // EEPROM_TEST
  }

  if (m_temp_aux_pid_data.k != mp_eth_data->aux_tr_data.temp_k)
  {
    m_temp_aux_pid_data.k = mp_eth_data->aux_tr_data.temp_k;
    pid_reg_sync(&m_temp_aux_pid_data);
    #ifdef EEPROM_TEST
    mp_eeprom_data->temp_aux_k = mp_eth_data->aux_tr_data.temp_k;
    #endif // EEPROM_TEST
  }

  if (m_temp_aux_ki != mp_eth_data->aux_tr_data.temp_ki)
  {
    m_temp_aux_ki = mp_eth_data->aux_tr_data.temp_ki;
    m_temp_aux_pid_data.ki = m_temp_aux_ki * m_dt;
    pid_reg_sync(&m_temp_aux_pid_data);
    #ifdef EEPROM_TEST
    mp_eeprom_data->temp_aux_ki = m_temp_aux_ki;
    #endif // EEPROM_TEST
  }

  if (m_temp_aux_kd != mp_eth_data->aux_tr_data.temp_kd)
  {
    m_temp_aux_kd = mp_eth_data->aux_tr_data.temp_kd;
    m_temp_aux_pid_data.kd = m_temp_aux_kd / m_dt;
    pid_reg_sync(&m_temp_aux_pid_data);
    #ifdef EEPROM_TEST
    mp_eeprom_data->temp_aux_kd = m_temp_aux_kd;
    #endif // EEPROM_TEST
  }

  if (m_temp_base_time_const != mp_eth_data->base_tr_data.temp_time_const)
  {
    m_temp_base_time_const =
      static_cast<float>(mp_eth_data->base_tr_data.temp_time_const);
    #ifdef EEPROM_TEST
    mp_eeprom_data->temp_base_time_const = m_temp_base_time_const;
    #endif // EEPROM_TEST
    m_temp_base_isodr.fd.t = m_temp_base_time_const;
  }

  if (m_temp_base_isodr.k != mp_eth_data->base_tr_data.temp_prop_koef)
  {
    m_temp_base_isodr.k = mp_eth_data->base_tr_data.temp_prop_koef;
    #ifdef EEPROM_TEST
    mp_eeprom_data->temp_base_prop_koef = m_temp_base_isodr.k;
    #endif // EEPROM_TEST
  }

  if (m_temp_aux_time_const != mp_eth_data->aux_tr_data.temp_time_const)
  {
    m_temp_aux_time_const =
      static_cast<float>(mp_eth_data->aux_tr_data.temp_time_const);
    #ifdef EEPROM_TEST
    mp_eeprom_data->temp_aux_time_const = m_temp_aux_time_const;
    #endif // EEPROM_TEST
    m_temp_aux_isodr.fd.t = m_temp_aux_time_const;
  }

  if (m_temp_aux_isodr.k != mp_eth_data->aux_tr_data.temp_prop_koef)
  {
    m_temp_aux_isodr.k = mp_eth_data->aux_tr_data.temp_prop_koef;
    #ifdef EEPROM_TEST
    mp_eeprom_data->temp_aux_prop_koef = m_temp_aux_isodr.k;
    #endif // EEPROM_TEST
  }

  //  Вывод температуры и показаний АЦП
  if (m_timer.check())
  {
    mp_eth_data->base_temp_data.value =
     m_th_base_data.temperature_code * m_th_base.get_conv_koef();
    mp_eth_data->base_temp_data.filtered_value =
      fade(&m_temp_base_isodr.fd,
      static_cast<float>(mp_eth_data->base_temp_data.value));
    mp_eth_data->aux_temp_data.value =
     m_th_aux_data.temperature_code * m_th_aux.get_conv_koef();
    mp_eth_data->aux_temp_data.filtered_value =
      fade(&m_temp_aux_isodr.fd,
      static_cast<float>(mp_eth_data->aux_temp_data.value));

    mp_eth_data->prev_adc_data.voltage_code =
      m_prev_adc_koef * m_adc102_data.voltage_code_A;
    mp_eth_data->fin_adc_data.voltage_code =
      m_fin_adc_koef * m_adc102_data.voltage_code_B;
  }

  #ifdef EEPROM_TEST
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
  #endif // EEPROM_TEST
  if (m_operate)
  {
    if (m_timer_reg.check())
    {
      double base_pid_reg_data_in =
        mp_eth_data->base_tr_data.temperature_ref -
        mp_eth_data->base_temp_data.filtered_value;
      m_temp_reg_data.voltage_code_A =
        static_cast<irs_u16>(irs::pid_reg(&m_temp_base_pid_data,
        base_pid_reg_data_in));
      mp_eth_data->base_tr_data.dac_value =
        m_temp_reg_data.voltage_code_A;
      mp_eth_data->base_tr_data.int_val =
        static_cast<irs_i32>(m_temp_base_pid_data.int_val*m_temp_base_pid_data.k*
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
        static_cast<irs_i32>(m_temp_aux_pid_data.int_val*m_temp_aux_pid_data.k*
        m_temp_aux_pid_data.ki);
    }
  }
}

void u309m::supply_t::on()
{
  //  Синхронизация пид-регулятора
  double pid_reg_data_in =
    mp_eth_data->base_tr_data.temperature_ref -
    mp_eth_data->base_temp_data.filtered_value;
  pid_reg_sync(&m_temp_base_pid_data, pid_reg_data_in, 0);
  pid_reg_data_in =
    mp_eth_data->aux_tr_data.temperature_ref -
    mp_eth_data->aux_temp_data.filtered_value;
  pid_reg_sync(&m_temp_aux_pid_data, pid_reg_data_in, 0);
  m_operate = true;
}

void u309m::supply_t::off()
{
  //  Выключение источника, выключение нагревателей
  m_prev_dac_reg_write = 0;
  m_volt_reg_data.voltage_code_A = 0;
  mp_eth_data->prev_dac_data.voltage_code = 0;
  m_fin_dac_reg_write = 0;
  m_volt_reg_data.voltage_code_B = 0;
  mp_eth_data->fin_dac_data.voltage_code = 0;
  m_temp_reg_data.voltage_code_A = 0;
  mp_eth_data->base_tr_data.dac_value = m_temp_reg_data.voltage_code_A;
  m_temp_reg_data.voltage_code_B = 0;
  mp_eth_data->aux_tr_data.dac_value = m_temp_reg_data.voltage_code_B;
  m_operate = false;
}
