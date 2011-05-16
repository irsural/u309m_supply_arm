#include <irsdefs.h>

#include <armioregs.h>

#include <irssysutils.h>

#include "cfg.h"

#include <irsfinal.h>

u309m::cfg_t::cfg_t():
  m_supply_comm_cfg_done(GPIO_PORTC, 7, irs::gpio_pin_t::dir_in),
  m_ipt_plis_ready(&m_supply_comm_cfg_done, irs::make_cnt_ms(200)),
  m_meas_comm_cfg_done(GPIO_PORTJ, 1, irs::gpio_pin_t::dir_in),
  m_meas_plis_ready(&m_meas_comm_cfg_done, irs::make_cnt_ms(200)),
  m_spi_buf_size(3),
  m_f_osc(80000000),
  #ifdef ARM_ADC_TEST
  m_adc(irs::make_cnt_ms(100)),
  #endif // ARM_ADC_TEST
  m_spi_meas_comm_plis(m_spi_buf_size, m_f_osc, irs::arm::arm_spi_t::SPI,
    irs::arm::arm_spi_t::SSI1, GPIO_PORTE, GPIO_PORTE, GPIO_PORTE),
  m_spi_general_purpose(m_spi_buf_size, m_f_osc, irs::arm::arm_spi_t::SPI,
    irs::arm::arm_spi_t::SSI0),

  m_spi_cs_code_0(GPIO_PORTH, 0, irs::gpio_pin_t::dir_out),
  m_spi_cs_code_1(GPIO_PORTH, 1, irs::gpio_pin_t::dir_out),
  m_spi_cs_code_2(GPIO_PORTH, 2, irs::gpio_pin_t::dir_out),
  m_spi_cs_code_3(GPIO_PORTH, 3, irs::gpio_pin_t::dir_out),
  m_spi_cs_code_4(GPIO_PORTH, 4, irs::gpio_pin_t::dir_out),
  m_spi_cs_enable(GPIO_PORTF, 5, irs::gpio_pin_t::dir_out),
  m_spi_demux_cs_data(&m_spi_cs_code_0, &m_spi_cs_code_1, &m_spi_cs_code_2,
    &m_spi_cs_code_3, &m_spi_cs_code_4, &m_spi_cs_enable),
  m_spi_demux(&m_spi_demux_cs_data),

  m_dac_cs_code_0(GPIO_PORTB, 0, irs::gpio_pin_t::dir_out),
  m_dac_cs_code_1(GPIO_PORTB, 1, irs::gpio_pin_t::dir_out),
  m_dac_cs_code_2(GPIO_PORTB, 2, irs::gpio_pin_t::dir_out),
  m_dac_demux_cs_data(&m_dac_cs_code_0, &m_dac_cs_code_1, &m_dac_cs_code_2),
  m_dac_demux(&m_dac_demux_cs_data),
  
  #ifndef EEPROM_TEST
  m_local_mac(irs::make_mxmac(0, 0, 192, 168, 0, 211)),
  #else // EEPROM_TEST
  m_local_mac(mxmac_t::zero_mac()),
  #endif // EEPROM_TEST
  m_arm_eth(irs::simple_ethernet_t::double_buf, 300, m_local_mac),
  #ifndef EEPROM_TEST
  m_local_ip(irs::make_mxip(192, 168, 0, 211)),
  #else // EEPROM_TEST
  m_local_ip(mxip_t::zero_ip()),
  #endif // EEPROM_TEST
  m_local_port(5006),
  m_dest_ip(irs::make_mxip(192, 168, 0, 28)),
  m_dest_port(5006),
  m_tcpip(&m_arm_eth, m_local_ip, m_dest_ip, 10),
  m_simple_hardflow(&m_tcpip, m_local_ip, m_local_port,
    m_dest_ip, m_dest_port, 10),
  m_modbus_server(&m_simple_hardflow, 0, 14, 317, 0, irs::make_cnt_ms(200)),
  m_eth_data(&m_modbus_server),
  
  m_izm_th_enable(GPIO_PORTG, 1, irs::gpio_pin_t::dir_out),
  m_meas_comm_cs(GPIO_PORTB, 3, irs::gpio_pin_t::dir_out),
  m_meas_comm_reset(GPIO_PORTH, 5, irs::gpio_pin_t::dir_out),
  m_meas_comm_apply(GPIO_PORTF, 1, irs::gpio_pin_t::dir_in),
  m_meas_comm_error(GPIO_PORTJ, 7, irs::gpio_pin_t::dir_in),
  m_meas_comm_pins(&m_meas_comm_cs, &m_meas_comm_reset, &m_meas_comm_apply,
    &m_meas_comm_error, m_spi_demux.cs_code(IZM_TH_CS_1),
    m_spi_demux.cs_code(IZM_TH_CS_2), m_spi_demux.cs_code(IZM_TH_CS_3),
    m_spi_demux.cs_code(IZM_TH_CS_4), m_spi_demux.cs_code(IZM_TH_CS_5)),

  m_supply_comm_reset(GPIO_PORTG, 0, irs::gpio_pin_t::dir_out),
  m_supply_comm_pins(m_spi_demux.cs_code(CS_PLIS), &m_supply_comm_reset),

  m_supply_200V_pins(
    m_spi_demux.cs_code(CS_TH1_200V), m_spi_demux.cs_code(CS_TH2_200V),
    m_spi_demux.cs_code(CS_ADC_200V), m_spi_demux.cs_code(CS_TC_200V),
    m_dac_demux.cs_code(CS_DAC_200V), m_spi_demux.cs_code(CS_TR_5)),
  m_supply_20V_pins(
    m_spi_demux.cs_code(CS_TH1_20V), m_spi_demux.cs_code(CS_TH2_20V),
    m_spi_demux.cs_code(CS_ADC_20V), m_spi_demux.cs_code(CS_TC_20V),
    m_dac_demux.cs_code(CS_DAC_20V), m_spi_demux.cs_code(CS_TR_4)),
  m_supply_2V_pins(
    m_spi_demux.cs_code(CS_TH1_2V), m_spi_demux.cs_code(CS_TH2_2V),
    m_spi_demux.cs_code(CS_ADC_2V), m_spi_demux.cs_code(CS_TC_2V),
    m_dac_demux.cs_code(CS_DAC_2V), m_spi_demux.cs_code(CS_TR_3)),
  m_supply_1A_pins(
    m_spi_demux.cs_code(CS_TH1_1A), m_spi_demux.cs_code(CS_TH2_1A),
    m_spi_demux.cs_code(CS_ADC_1A), m_spi_demux.cs_code(CS_TC_1A),
    m_dac_demux.cs_code(CS_DAC_1A), m_spi_demux.cs_code(CS_TR_2)),
  m_supply_17A_pins(
    m_spi_demux.cs_code(CS_TH1_17A), m_spi_demux.cs_code(CS_TH2_17A),
    m_spi_demux.cs_code(CS_ADC_17A), m_spi_demux.cs_code(CS_TC_17A),
    m_dac_demux.cs_code(CS_DAC_17A), m_spi_demux.cs_code(CS_TR_1)),
  m_command_pins(&m_meas_comm_pins, &m_supply_comm_pins, &m_supply_200V_pins,
    &m_supply_20V_pins, &m_supply_2V_pins, &m_supply_1A_pins,
    &m_supply_17A_pins),
  #ifdef MEAS_COMM_TEST
  m_meas_comm(
    &m_spi_general_purpose,
    &m_spi_meas_comm_plis,
    m_command_pins.meas_comm,
    &m_eth_data.meas_comm
  ),
  #endif // MEAS_COMM_TEST
  #ifdef SUPPLY_COMM_TEST
  m_supply_comm(
    &m_spi_general_purpose,
    m_command_pins.supply_comm,
    &m_eth_data.supply_comm
  ),
  #endif // SUPPLY_COMM_TEST
  m_SYM_2V_on(GPIO_PORTA, 3, irs::gpio_pin_t::dir_out),
  m_SYM_2V_off(GPIO_PORTJ, 0, irs::gpio_pin_t::dir_out),
  m_SYM_20V_on(GPIO_PORTC, 4, irs::gpio_pin_t::dir_out),
  m_SYM_20V_off(GPIO_PORTD, 2, irs::gpio_pin_t::dir_out),
  m_SYM_200V_on(GPIO_PORTC, 5, irs::gpio_pin_t::dir_out),
  m_SYM_200V_off(GPIO_PORTA, 6, irs::gpio_pin_t::dir_out),
  m_KZ_2V_on(GPIO_PORTH, 7, irs::gpio_pin_t::dir_out),
  m_KZ_2V_off(GPIO_PORTD, 3, irs::gpio_pin_t::dir_out),
  m_KZ_1A(GPIO_PORTC, 6, irs::gpio_pin_t::dir_out),
  m_KZ_17A(GPIO_PORTA, 7, irs::gpio_pin_t::dir_out),
  m_REL_220V(GPIO_PORTG, 7, irs::gpio_pin_t::dir_out),
  m_SYM_OFF(GPIO_PORTB, 5, irs::gpio_pin_t::dir_out),
  m_SYM_OFF_TEST(GPIO_PORTB, 4, irs::gpio_pin_t::dir_in),
  m_rele_ext_pins(&m_SYM_2V_on, &m_SYM_2V_off, &m_SYM_20V_on, &m_SYM_20V_off,
    &m_SYM_200V_on, &m_SYM_200V_off, &m_KZ_2V_on, &m_KZ_2V_off,
    &m_KZ_1A, &m_KZ_17A, &m_REL_220V, &m_SYM_OFF, &m_SYM_OFF_TEST),
  #ifdef EEPROM_TEST
  m_eeprom_size(334),
  m_eeprom_command(&m_spi_general_purpose, m_spi_demux.cs_code(CS_EE),
    irs::eeprom_command_t::at25128a),
  m_eeprom(&m_eeprom_command, m_eeprom_size),
  m_eeprom_data(&m_eeprom),
  #endif // EEPROM_TEST
  m_timer(irs::make_cnt_ms(200))
{
  #ifdef EEPROM_TEST
  if (m_eeprom.error()) {
    m_eeprom_data.reset_to_default(sup_200V);
    m_eeprom_data.reset_to_default(sup_20V);
    m_eeprom_data.reset_to_default(sup_2V);
    m_eeprom_data.reset_to_default(sup_1A);
    m_eeprom_data.reset_to_default(sup_17A);
    m_eth_data.reset_to_default(sup_200V);
    m_eth_data.reset_to_default(sup_20V);
    m_eth_data.reset_to_default(sup_2V);
    m_eth_data.reset_to_default(sup_1A);
    m_eth_data.reset_to_default(sup_17A);
  }
  m_eth_data.ip_0 = m_eeprom_data.ip_0;
  m_eth_data.ip_1 = m_eeprom_data.ip_1;
  m_eth_data.ip_2 = m_eeprom_data.ip_2;
  m_eth_data.ip_3 = m_eeprom_data.ip_3;
  irs::string ip_0_str = irst("");
  irs::number_to_string(m_eeprom_data.ip_0, &ip_0_str);
  irs::string ip_1_str = irst("");
  irs::number_to_string(m_eeprom_data.ip_0, &ip_1_str);
  irs::string ip_2_str = irst("");
  irs::number_to_string(m_eeprom_data.ip_0, &ip_2_str);
  irs::string ip_3_str = irst("");
  irs::number_to_string(m_eeprom_data.ip_0, &ip_3_str);
  irs::string ip_str = irst(ip_3_str + "." + ip_2_str + "." +
    ip_1_str + "." + ip_0_str);
  m_simple_hardflow.set_param("local_addr", ip_str);
  #endif // EEPROM_TEST
  m_izm_th_enable.set();
  m_meas_comm_reset.clear();
  m_supply_comm_reset.clear();
}

u309m::command_pins_t* u309m::cfg_t::command_pins()
{
  return& m_command_pins;
}

irs::arm::adc_t* u309m::cfg_t::adc()
{
  #ifdef ARM_ADC_TEST
  return& m_adc;
  #else
  return IRS_NULL;
  #endif // ARM_ADC_TEST
}

irs::arm::arm_spi_t* u309m::cfg_t::spi_meas_comm_plis()
{
  return& m_spi_meas_comm_plis;
}

irs::arm::arm_spi_t* u309m::cfg_t::spi_general_purpose()
{
  return& m_spi_general_purpose;
}

irs::arm::arm_spi_t* u309m::cfg_t::spi_supply_comm_plis()
{
  return& m_spi_general_purpose;
}

u309m::eth_data_t* u309m::cfg_t::eth_data()
{
  return& m_eth_data;
}

u309m::eeprom_data_t* u309m::cfg_t::eeprom_data()
{
  #ifdef EEPROM_TEST
  return& m_eeprom_data;
  #else 
  return IRS_NULL;
  #endif // EEPROM_TEST
}

irs::hardflow::simple_udp_flow_t* u309m::cfg_t::hardflow()
{
  return& m_simple_hardflow;
}

u309m::rele_ext_pins_t* u309m::cfg_t::rele_ext_pins()
{
  return& m_rele_ext_pins;
}

u309m::meas_comm_t* u309m::cfg_t::meas_comm()
{
  #ifdef MEAS_COMM_TEST
  return& m_meas_comm;
  #else
  return IRS_NULL;
  #endif // MEAS_COMM_TEST
}

u309m::supply_comm_t* u309m::cfg_t::supply_comm()
{
  #ifdef SUPPLY_COMM_TEST
  return& m_supply_comm;
  #else
  return IRS_NULL;
  #endif // SUPPLY_COMM_TEST
}

void u309m::cfg_t::tick()
{
  #ifdef ARM_ADC_TEST
  m_adc.tick();
  #endif // ARM_ADC_TEST
  m_spi_meas_comm_plis.tick();
  m_spi_general_purpose.tick();
  m_modbus_server.tick();
  #ifdef EEPROM_TEST
  m_eeprom.tick();
  #endif // EEPROM_TEST
  
  if (m_timer.check()) {
    #ifdef ARM_ADC_TEST
    m_eth_data.arm_adc.PTC_A = m_adc.get_data(PTC_A_channel);
    if (m_eth_data.arm_adc.PTC_A > 1) {
      // ALARM!!! OVERHEAT!!!
    }
    m_eth_data.arm_adc.PTC_LC = m_adc.get_data(PTC_LC_channel);
    if (m_eth_data.arm_adc.PTC_LC > 1) {
      // ALARM!!! OVERHEAT!!!
    }
    m_eth_data.arm_adc.TR_24V_TEST =
      m_adc.get_data(TR_24V_TEST_channel)*11.f;
    m_eth_data.arm_adc.IZM_3_3V_TEST =
      m_adc.get_data(IZM_3_3V_TEST_channel)*3.f;
    m_eth_data.arm_adc.IZM_6V_TEST = m_adc.get_data(IZM_6V_TEST_channel)*3.f;
    m_eth_data.arm_adc.IZM_1_2V_TEST =
      m_adc.get_data(IZM_1_2V_TEST_channel)*3.f;
    m_eth_data.arm_adc.TEST_24V = m_adc.get_data(TEST_24V_channel)*11.f;
    m_eth_data.arm_adc.TEST_5V = m_adc.get_data(TEST_5V_channel)*11.f;
    m_eth_data.arm_adc.PTC_PWR = m_adc.get_data(PTC_PWR_channel);
    if (m_eth_data.arm_adc.PTC_PWR > 1) {
      // ALARM!!! OVERHEAT!!!
    }
    m_eth_data.arm_adc.PTC_17A = m_adc.get_data(PTC_17A_channel);
    if (m_eth_data.arm_adc.PTC_17A > 1) {
      // ALARM!!! OVERHEAT!!!
    }
    m_eth_data.arm_adc.internal_temp = m_adc.get_temperature();
    #endif // ARM_ADC_TEST
  }
}
