#include <irsdefs.h>

#include <armioregs.h>
#include <irssysutils.h>

#include "cfg.h"
#include "privatecfg.h"

#include <irsfinal.h>

// Если отсутствует, то необходимо создать файл "privatecfg.h"
// Можно скопировать файл "privatecfg0.h"
// IP заменить на необходимый

u309m::cfg_t::cfg_t():
  m_spi_cs_code_0(GPIO_PORTH, 0, irs::gpio_pin_t::dir_out),
  m_spi_cs_code_1(GPIO_PORTH, 1, irs::gpio_pin_t::dir_out),
  m_spi_cs_code_2(GPIO_PORTH, 2, irs::gpio_pin_t::dir_out),
  m_spi_cs_code_3(GPIO_PORTH, 3, irs::gpio_pin_t::dir_out),
  m_spi_cs_code_4(GPIO_PORTH, 4, irs::gpio_pin_t::dir_out),
  m_spi_cs_enable(GPIO_PORTF, 5, irs::gpio_pin_t::dir_out),
  m_spi_demux_cs_data(&m_spi_cs_code_0, &m_spi_cs_code_1, &m_spi_cs_code_2,
    &m_spi_cs_code_3, &m_spi_cs_code_4, &m_spi_cs_enable),
  m_spi_demux(&m_spi_demux_cs_data),

  m_supply_comm_cfg_done(GPIO_PORTC, 7, irs::gpio_pin_t::dir_in),
  m_supply_comm_reset(GPIO_PORTG, 0, irs::gpio_pin_t::dir_out),
  m_supply_comm_pins(m_supply_comm_cfg_done, *m_spi_demux.cs_code(CS_PLIS),
    m_supply_comm_reset),

  #ifdef OLD_MEAS_COMM
  m_meas_comm_cs(GPIO_PORTB, 3, irs::gpio_pin_t::dir_out),
  m_meas_comm_reset(GPIO_PORTH, 5, irs::gpio_pin_t::dir_out),
  m_meas_comm_apply(GPIO_PORTF, 1, irs::gpio_pin_t::dir_in),
  m_meas_comm_error(GPIO_PORTJ, 7, irs::gpio_pin_t::dir_in),
  m_meas_comm_pins(&m_meas_comm_cs, &m_meas_comm_reset, &m_meas_comm_apply,
    &m_meas_comm_error),
  #else //  !OLD_MEAS_COMM
  m_meas_comm_cfg_done(GPIO_PORTJ, 1, irs::gpio_pin_t::dir_in),
  m_meas_comm_cs(GPIO_PORTB, 3, irs::gpio_pin_t::dir_out),
  m_meas_comm_reset(GPIO_PORTJ, 7, irs::gpio_pin_t::dir_out),
  m_meas_comm_pins(m_meas_comm_cfg_done, m_meas_comm_cs, m_meas_comm_reset),
  #endif  //  OLD_MEAS_COMM

  m_izm_th_enable(GPIO_PORTG, 1, irs::gpio_pin_t::dir_out),
  m_meas_comm_th_pins(
    m_spi_demux.cs_code(IZM_TH_CS_1),
    m_spi_demux.cs_code(IZM_TH_CS_2),
    m_spi_demux.cs_code(IZM_TH_CS_3),
    m_spi_demux.cs_code(IZM_TH_CS_4),
    m_spi_demux.cs_code(IZM_TH_CS_5),
    &m_izm_th_enable),

  m_dac_cs_code_0(GPIO_PORTB, 0, irs::gpio_pin_t::dir_out),
  m_dac_cs_code_1(GPIO_PORTB, 1, irs::gpio_pin_t::dir_out),
  m_dac_cs_code_2(GPIO_PORTB, 2, irs::gpio_pin_t::dir_out),
  m_dac_demux_cs_data(&m_dac_cs_code_0, &m_dac_cs_code_1, &m_dac_cs_code_2),
  m_dac_demux(&m_dac_demux_cs_data),

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
  m_command_pins(&m_supply_200V_pins, &m_supply_20V_pins, &m_supply_2V_pins,
    &m_supply_1A_pins, &m_supply_17A_pins),

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
  m_adc(((1 << PTC_A_channel) | (1 << PTC_LC_channel) |
        (1 << TR_24V_TEST_channel) | (1 << IZM_3_3V_TEST_channel) |
        (1 << IZM_6V_TEST_channel) | (1 << IZM_1_2V_TEST_channel) |
        (1 << TEST_24V_channel) | (1 << TEST_5V_channel) |
        (1 << PTC_PWR_channel) | (1 << PTC_17A_channel)),
        irs::arm::adc_stellaris_t::EXT_REF),
  m_spi_bitrate(500000),
  m_spi_meas_comm_plis(m_spi_bitrate,
    irs::arm::arm_spi_t::SPI, irs::arm::arm_spi_t::SSI1, GPIO_PORTE,
    GPIO_PORTE, GPIO_PORTE),
  m_spi_general_purpose(m_spi_bitrate,
    irs::arm::arm_spi_t::SPI, irs::arm::arm_spi_t::SSI0),

  m_supply_tact_gen(PF4, m_spi_bitrate * 10),
  #ifndef OLD_MEAS_COMM
  m_meas_tact_gen(PE1, m_spi_bitrate * 10),
  #endif  //  OLD_MEAS_COMM

  m_local_mac(mxmac_t::zero_mac()),
  m_arm_eth(irs::simple_ethernet_t::double_buf, 300, m_local_mac),
  #ifdef LWIP
  m_ethernet(),
  m_udp_client(),
  m_connector_hardflow(IRS_NULL),
  m_ip(mxip_t::any_ip()),
  m_dhcp(false),
  m_mask(mxip_t::any_ip()),
  m_gateway(mxip_t::any_ip()),
  #else //LWIP
  m_local_ip(mxip_t::zero_ip()),
  m_local_port(5006),
  m_dest_ip(irs::make_mxip(192, 168, 0, 28)),
  m_dest_port(5006),
  m_tcpip(&m_arm_eth, m_local_ip, m_dest_ip, 10),
  m_simple_hardflow(&m_tcpip, m_local_ip, m_local_port,
    m_dest_ip, m_dest_port, 10),
  #endif // LWIP
  m_meas_comm_reset_test(GPIO_PORTJ, 6, irs::gpio_pin_t::dir_in),
  mp_main_info(IRS_NULL)
{
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

irs::hardflow::simple_udp_flow_t* u309m::cfg_t::hardflow()
{
  return& m_simple_hardflow;
}

u309m::rele_ext_pins_t* u309m::cfg_t::rele_ext_pins()
{
  return& m_rele_ext_pins;
}

u309m::meas_comm_th_pins_t* u309m::cfg_t::meas_comm_th_pins()
{
  return& m_meas_comm_th_pins;
}

void u309m::cfg_t::izm_th_spi_enable_pin_set(bool a_value)
{
  if (a_value) {
    m_izm_th_enable.set();
  }
  else m_izm_th_enable.clear();
}

u309m::plis_pins_t& u309m::cfg_t::supply_comm_pins()
{
  return m_supply_comm_pins;
}

#ifdef  OLD_MEAS_COMM
u309m::meas_comm_pins_t* u309m::cfg_t::meas_pins()
{
  return &m_meas_comm_pins;
}
#else //  !OLD_MEAS_COMM
u309m::plis_pins_t& u309m::cfg_t::meas_comm_pins()
{
  return m_meas_comm_pins;
}

irs::pwm_gen_t& u309m::cfg_t::meas_tact_gen()
{
  return m_meas_tact_gen;
}

#endif  //  OLD_MEAS_COMM

irs::pwm_gen_t& u309m::cfg_t::supply_tact_gen()
{
  return m_supply_tact_gen;
}

irs::gpio_pin_t* u309m::cfg_t::pins_eeprom()
{
  return m_spi_demux.cs_code(CS_EE);
}

irs::gpio_pin_t* u309m::cfg_t::pins_meas_comm_reset_test()
{
  return &m_meas_comm_reset_test;
}

#ifdef LWIP
void cfg_t::change_ip(mxip_t a_ip)
{
  network_conf(a_ip, m_mask, m_gateway, m_dhcp);
}
void cfg_t::change_dhcp(bool a_dhcp)
{
  network_conf(m_ip, m_mask, m_gateway, a_dhcp);
}
void cfg_t::change_mask(mxip_t a_mask)
{
  network_conf(m_ip, a_mask, m_gateway, m_dhcp);
}
void cfg_t::change_gateway(mxip_t a_gateway)
{
  network_conf(m_ip, m_mask, a_gateway, m_dhcp);
}
void cfg_t::network_conf(mxip_t a_ip, mxip_t a_mask,
  mxip_t a_gateway, bool a_dhcp)
{
  m_ip = a_ip;
  m_dhcp = a_dhcp;
  m_mask = a_mask;
  m_gateway = a_gateway;

  m_udp_client.reset();
  mxip_t local_ip = mxip_t::any_ip();
  if (!a_dhcp) {
    local_ip = a_ip;
  }
  mxip_t remote_ip = mxip_t::any_ip();
  mxip_t mask = a_mask;
  mxip_t gateway_ip = a_gateway;
  size_t channel_max_count = 3;
  irs::lwip::ethernet_t::configuration_t eth_config;
  eth_config.ip = local_ip;
  eth_config.netmask = mask;
  eth_config.gateway = gateway_ip;
  eth_config.dhcp_enabled = a_dhcp;
  m_ethernet.reset(new irs::lwip::ethernet_t(&m_arm_eth, eth_config));
  irs::hardflow::lwip::udp_t::configuration_t configuration;
  m_udp_client.reset(new irs::hardflow::lwip::udp_t(
    local_ip, 5005, remote_ip, 0, channel_max_count, configuration));
  m_udp_client->
    set_channel_switching_mode(irs::hardflow_t::csm_ready_for_reading);
  m_connector_hardflow.hardflow(m_udp_client.get());

}
#endif //LWIP
