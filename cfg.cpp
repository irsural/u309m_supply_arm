#include <irsdefs.h>

#include <armioregs.h>

#include "cfg.h"

#include <irsfinal.h>

u309m::cfg_t::cfg_t():
  m_spi_buf_size(3),
  m_f_osc(80000000),
  m_adc(irs::make_cnt_ms(100)),
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

  m_eeprom_size(0),
  m_eeprom_command(&m_spi_general_purpose, m_spi_demux.cs_code(CS_EE),
    irs::eeprom_command_t::at25128a),
  m_eeprom(&m_eeprom_command, m_eeprom_size),
  m_eeprom_data(&m_eeprom),
  m_local_mac(irs::make_mxmac(0, 0, 192, 168, 0, 211)),
  m_arm_eth(irs::simple_ethernet_t::double_buf, 300, m_local_mac),
  m_local_ip(irs::make_mxip(192, 168, 0, 211)),
  m_local_port(5006),
  m_dest_ip(irs::make_mxip(192, 168, 0, 28)),
  m_dest_port(5006),
  m_tcpip(&m_arm_eth, m_local_ip, m_dest_ip, 10),
  m_simple_hardflow(&m_tcpip, m_local_ip, m_local_port,
    m_dest_ip, m_dest_port, 10),
  // œŒ—“¿¬»“‹ œ–¿¬»À‹Õ€≈ –¿«Ã≈–€ Ã¿——»¬Œ¬ ¬ ÃŒƒ¡¿— !!!!!!!!!!!!
  m_modbus_server(&m_simple_hardflow, 100, 100, 500, 200,
    irs::make_cnt_ms(200)),
  // œŒ—“¿¬»“‹ œ–¿¬»À‹Õ€≈ –¿«Ã≈–€ Ã¿——»¬Œ¬ ¬ ÃŒƒ¡¿— !!!!!!!!!!!!
  m_eth_data(&m_modbus_server),
  
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
    &m_supply_17A_pins)
{
  //m_simple_hardflow.set_param("local_addr", "192.168.0.211");
  m_meas_comm_reset.clear();
}

u309m::command_pins_t* u309m::cfg_t::command_pins()
{
  return& m_command_pins;
}

irs::arm::adc_t* u309m::cfg_t::adc()
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

void u309m::cfg_t::tick()
{
  m_adc.tick();
  m_spi_meas_comm_plis.tick();
  m_spi_general_purpose.tick();
  m_modbus_server.tick();
}
