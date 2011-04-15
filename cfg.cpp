#include <irsdefs.h>

#include <armioregs.h>

#include "cfg.h"

#include <irsfinal.h>

u309m::cfg_t::cfg_t():
  m_spi_buf_size(3),
  m_f_osc(80000000),
  m_adc(irs::make_cnt_ms(100)),
  m_spi_meas_comm_plis(m_spi_buf_size, m_f_osc, irs::arm::arm_spi_t::SPI,
    irs::arm::arm_spi_t::SSI0),
  m_spi_meas_comm_term(m_spi_buf_size, m_f_osc, irs::arm::arm_spi_t::SPI,
    irs::arm::arm_spi_t::SSI1),
  m_meas_comm_cs(GPIO_PORTG, 0, irs::gpio_pin_t::dir_out),
  m_meas_comm_reset(GPIO_PORTA, 3, irs::gpio_pin_t::dir_out),
  m_meas_comm_apply(GPIO_PORTA, 7, irs::gpio_pin_t::dir_in),
  m_meas_comm_error(GPIO_PORTD, 3, irs::gpio_pin_t::dir_in),
  m_meas_termo_sense_1_pin(GPIO_PORTJ, 3, irs::gpio_pin_t::dir_out),
  m_meas_termo_sense_2_pin(GPIO_PORTJ, 4, irs::gpio_pin_t::dir_out),
  m_meas_termo_sense_3_pin(GPIO_PORTJ, 5, irs::gpio_pin_t::dir_out),
  m_meas_termo_sense_4_pin(GPIO_PORTJ, 6, irs::gpio_pin_t::dir_out),
  m_meas_termo_sense_5_pin(GPIO_PORTJ, 7, irs::gpio_pin_t::dir_out),
  m_local_mac(irs::make_mxmac(0, 0, 192, 168, 0, 211)),
  m_arm_eth(irs::simple_ethernet_t::double_buf, 300, m_local_mac),
  m_local_ip(irs::make_mxip(192, 168, 0, 211)),
  m_local_port(5006),
  m_dest_ip(irs::make_mxip(192, 168, 0, 28)),
  m_dest_port(5006),
  m_tcpip(&m_arm_eth, m_local_ip, m_dest_ip, 10),
  m_simple_hardflow(&m_tcpip, m_local_ip, m_local_port,
    m_dest_ip, m_dest_port, 10),
  // ÏÎÑÒÀÂÈÒÜ ÏĞÀÂÈËÜÍÛÅ ĞÀÇÌÅĞÛ ÌÀÑÑÈÂÎÂ Â ÌÎÄÁÀÑ !!!!!!!!!!!!
  m_modbus_server(&m_simple_hardflow, 0, 7, 0, 18, irs::make_cnt_ms(200)),
  // ÏÎÑÒÀÂÈÒÜ ÏĞÀÂÈËÜÍÛÅ ĞÀÇÌÅĞÛ ÌÀÑÑÈÂÎÂ Â ÌÎÄÁÀÑ !!!!!!!!!!!!
  m_eth_data(&m_modbus_server),
  m_meas_comm_pins(&m_meas_comm_cs, &m_meas_comm_reset, &m_meas_comm_apply,
    &m_meas_comm_error, &m_meas_termo_sense_1_pin, &m_meas_termo_sense_2_pin,
    &m_meas_termo_sense_3_pin, &m_meas_termo_sense_4_pin,
    &m_meas_termo_sense_5_pin),
  // ÏÎÑÒÀÂÈÒÜ ĞÅÀËÜÍÛÅ ÏÈÍÛ !!!!!!!!!!!!
  m_supply_comm_pins(&m_meas_comm_cs, &m_meas_comm_reset, &m_meas_comm_apply,
    &m_meas_comm_error, &m_meas_termo_sense_1_pin, &m_meas_termo_sense_2_pin,
    &m_meas_termo_sense_3_pin, &m_meas_termo_sense_4_pin,
    &m_meas_termo_sense_5_pin),
  m_supply_200V_pins(&m_meas_comm_cs, &m_meas_comm_reset, &m_meas_comm_apply,
    &m_meas_comm_error, &m_meas_termo_sense_1_pin, &m_meas_termo_sense_2_pin),
  m_supply_20V_pins(&m_meas_comm_cs, &m_meas_comm_reset, &m_meas_comm_apply,
    &m_meas_comm_error, &m_meas_termo_sense_1_pin, &m_meas_termo_sense_2_pin),
  m_supply_2V_pins(&m_meas_comm_cs, &m_meas_comm_reset, &m_meas_comm_apply,
    &m_meas_comm_error, &m_meas_termo_sense_1_pin, &m_meas_termo_sense_2_pin),
  m_supply_1A_pins(&m_meas_comm_cs, &m_meas_comm_reset, &m_meas_comm_apply,
    &m_meas_comm_error, &m_meas_termo_sense_1_pin, &m_meas_termo_sense_2_pin),
  m_supply_17A_pins(&m_meas_comm_cs, &m_meas_comm_reset, &m_meas_comm_apply,
    &m_meas_comm_error, &m_meas_termo_sense_1_pin, &m_meas_termo_sense_2_pin),
  // ÏÎÑÒÀÂÈÒÜ ĞÅÀËÜÍÛÅ ÏÈÍÛ !!!!!!!!!!!!
  m_command_pins(&m_meas_comm_pins, &m_supply_comm_pins, &m_supply_200V_pins,
    &m_supply_20V_pins, &m_supply_2V_pins, &m_supply_1A_pins,
    &m_supply_17A_pins)
{
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

irs::arm::arm_spi_t* u309m::cfg_t::spi_term()
{
  return& m_spi_meas_comm_term;
}

irs::arm::arm_spi_t* u309m::cfg_t::spi_supply_comm_plis()
{
  return& m_spi_meas_comm_term;
}

u309m::eth_data_t* u309m::cfg_t::eth_data()
{
  return& m_eth_data;
}

void u309m::cfg_t::tick()
{
  m_adc.tick();
  m_spi_meas_comm_plis.tick();
  m_spi_meas_comm_term.tick();
  m_modbus_server.tick();
}
