#include <irsdefs.h>

#include <armioregs.h>
#include <irssysutils.h>

#include "cfg.h"
#include "privatecfg.h"

#include <irsfinal.h>

// ���� �����������, �� ���������� ������� ���� "privatecfg.h"
// ����� ����������� ���� "privatecfg0.h"
// IP �������� �� �����������

u309m::cfg_t::cfg_t():
  m_supply_comm_cfg_done(GPIO_PORTC, 7, irs::gpio_pin_t::dir_in),
  m_ipt_plis_ready(&m_supply_comm_cfg_done, irs::make_cnt_ms(200)),
  m_meas_comm_cfg_done(GPIO_PORTJ, 1, irs::gpio_pin_t::dir_in),
  m_meas_plis_ready(&m_meas_comm_cfg_done, irs::make_cnt_ms(200)),
  m_f_osc(80000000),
  m_adc(((1 << PTC_A_channel) | (1 << PTC_LC_channel) |
        (1 << TR_24V_TEST_channel) | (1 << IZM_3_3V_TEST_channel) |
        (1 << IZM_6V_TEST_channel) | (1 << IZM_1_2V_TEST_channel) |
        (1 << TEST_24V_channel) | (1 << TEST_5V_channel) |
        (1 << PTC_PWR_channel) | (1 << PTC_17A_channel)),
        irs::arm::adc_stellaris_t::EXT_REF),
  m_spi_bitrate(500000),
  m_spi_buf_size(3),
  m_spi_meas_comm_plis(m_spi_bitrate, m_spi_buf_size, 
    irs::arm::arm_spi_t::SPI, irs::arm::arm_spi_t::SSI1, GPIO_PORTE,
    GPIO_PORTE, GPIO_PORTE),
  m_spi_general_purpose(m_spi_bitrate, m_spi_buf_size,
    irs::arm::arm_spi_t::SPI, irs::arm::arm_spi_t::SSI0),

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
  m_local_mac(mxmac_t::zero_mac()),
  m_arm_eth(irs::simple_ethernet_t::double_buf, 300, m_local_mac),
  m_local_ip(mxip_t::zero_ip()),
  m_local_port(5006),
  m_dest_ip(irs::make_mxip(192, 168, 0, 28)),
  m_dest_port(5006),
  m_tcpip(&m_arm_eth, m_local_ip, m_dest_ip, 10),
  m_simple_hardflow(&m_tcpip, m_local_ip, m_local_port,
    m_dest_ip, m_dest_port, 10),
  //m_modbus_server(&m_simple_hardflow, 0, 14, 317, 0, irs::make_cnt_ms(200)),
  m_modbus_server(&m_simple_hardflow, 0, 14, 323, 0, irs::make_cnt_ms(200)),
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
  m_meas_comm(
    &m_spi_general_purpose,
    &m_spi_meas_comm_plis,
    m_command_pins.meas_comm,
    &m_eth_data.meas_comm
  ),
  m_supply_comm(
    &m_spi_general_purpose,
    m_command_pins.supply_comm,
    &m_eth_data.supply_comm
  ),
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
  m_eeprom_size(338),
  m_eeprom(m_eeprom_size),
  m_eeprom_data(&m_eeprom),
  m_timer(irs::make_cnt_ms(200))
{
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
  
  mxip_t ip = mxip_t::zero_ip();
  ip.val[0] = m_eeprom_data.ip_0;
  ip.val[1] = m_eeprom_data.ip_1;
  ip.val[2] = m_eeprom_data.ip_2;
  ip.val[3] = m_eeprom_data.ip_3;
  char ip_str[IP_STR_LEN];
  mxip_to_cstr(ip_str, ip);
  m_simple_hardflow.set_param("local_addr", ip_str);
  m_izm_th_enable.set();
  m_meas_comm_reset.clear();
  m_supply_comm_reset.clear();
  m_REL_220V.set();
}

u309m::command_pins_t* u309m::cfg_t::command_pins()
{
  return& m_command_pins;
}

irs::adc_t* u309m::cfg_t::adc()
{
  return& m_adc;
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
  return& m_eeprom_data;
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
  return& m_meas_comm;
}

u309m::supply_comm_t* u309m::cfg_t::supply_comm()
{
  return& m_supply_comm;
}

void u309m::cfg_t::tick()
{
  m_adc.tick();
  m_spi_meas_comm_plis.tick();
  m_spi_general_purpose.tick();
  m_modbus_server.tick();
  m_eeprom.tick();

  if (m_timer.check()) {
    m_eth_data.arm_adc.PTC_A = m_adc.get_float_data(PTC_A_num);
    m_eth_data.arm_adc.PTC_LC = m_adc.get_float_data(PTC_LC_num);
    m_eth_data.arm_adc.TR_24V_TEST =
      m_adc.get_float_data(TR_24V_TEST_num) * 36.348f;
    m_eth_data.arm_adc.IZM_3_3V_TEST =
      m_adc.get_float_data(IZM_3_3V_TEST_num) * 9.446f;
    m_eth_data.arm_adc.IZM_6V_TEST =
      m_adc.get_float_data(IZM_6V_TEST_num) * 9.313f;
    m_eth_data.arm_adc.IZM_1_2V_TEST =
      m_adc.get_float_data(IZM_1_2V_TEST_num) * 10.028f;
    m_eth_data.arm_adc.TEST_24V
      = m_adc.get_float_data(TEST_24V_num) * 32.999f;
    m_eth_data.arm_adc.TEST_5V
      = m_adc.get_float_data(TEST_5V_num) * 35.163f;
    m_eth_data.arm_adc.PTC_PWR = m_adc.get_float_data(PTC_PWR_num);
    m_eth_data.arm_adc.PTC_17A = m_adc.get_float_data(PTC_17A_num);
    m_eth_data.arm_adc.internal_temp = m_adc.get_temperature();
  }
}
