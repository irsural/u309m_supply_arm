#ifndef cfgh
#define cfgh

#include <irsdefs.h>

#include <armcfg.h>
#include <armspi.h>
#include <armgpio.h>
#include <armeth.h>
#include <armadc.h>

#include <irsnetdefs.h>
#include <irstcpip.h>
#include <hardflowg.h>
#include <armflash.h>
#include <irsdev.h>
#include <irsconfig.h>
#include <irsmem.h>

#include "demux.h"

#include <irsfinal.h>

namespace u309m {

struct plis_pins_t {
  irs::gpio_pin_t& cfg_done;
  irs::gpio_pin_t& cs;
  irs::gpio_pin_t& reset;

  plis_pins_t(
    irs::gpio_pin_t& a_cfg_done,
    irs::gpio_pin_t& a_cs,
    irs::gpio_pin_t& a_reset
  ):
    cfg_done(a_cfg_done),
    cs(a_cs),
    reset(a_reset)
  {
  }
};  //  plis_pins_t

#ifdef OLD_MEAS_COMM

struct meas_comm_pins_t {
  irs::gpio_pin_t* cs;
  irs::gpio_pin_t* reset;
  irs::gpio_pin_t* apply;
  irs::gpio_pin_t* error;

  meas_comm_pins_t(
    irs::gpio_pin_t* ap_cs,
    irs::gpio_pin_t* ap_reset,
    irs::gpio_pin_t* ap_apply,
    irs::gpio_pin_t* ap_error
  ):
    cs(ap_cs),
    reset(ap_reset),
    apply(ap_apply),
    error(ap_error)
  {
  }
}; // meas_comm_pins_t

#endif  //  OLD_MEAS_COMM

struct supply_pins_t {
  irs::gpio_pin_t* termo_sense_base_cs;
  irs::gpio_pin_t* termo_sense_aux_cs;
  irs::gpio_pin_t* adc_cs;
  irs::gpio_pin_t* tc_cs;
  irs::gpio_pin_t* volt_reg_cs;
  irs::gpio_pin_t* temp_reg_cs;

  supply_pins_t(
    irs::gpio_pin_t* ap_termo_sense_base_cs,
    irs::gpio_pin_t* ap_termo_sense_aux_cs,
    irs::gpio_pin_t* ap_adc_cs,
    irs::gpio_pin_t* ap_tc_cs,
    irs::gpio_pin_t* ap_volt_reg_cs,
    irs::gpio_pin_t* ap_temp_reg_cs
  ):
    termo_sense_base_cs(ap_termo_sense_base_cs),
    termo_sense_aux_cs(ap_termo_sense_aux_cs),
    adc_cs(ap_adc_cs),
    tc_cs(ap_tc_cs),
    volt_reg_cs(ap_volt_reg_cs),
    temp_reg_cs(ap_temp_reg_cs)
  {
  }
}; // supply_pins_t

struct command_pins_t {
  supply_pins_t* supply_200V;
  supply_pins_t* supply_20V;
  supply_pins_t* supply_2V;
  supply_pins_t* supply_1A;
  supply_pins_t* supply_17A;

  command_pins_t(
    supply_pins_t* ap_supply_200V_pins,
    supply_pins_t* ap_supply_20V_pins,
    supply_pins_t* ap_supply_2V_pins,
    supply_pins_t* ap_supply_1A_pins,
    supply_pins_t* ap_supply_17A_pins
  ):
    supply_200V(ap_supply_200V_pins),
    supply_20V(ap_supply_20V_pins),
    supply_2V(ap_supply_2V_pins),
    supply_1A(ap_supply_1A_pins),
    supply_17A(ap_supply_17A_pins)
  {
  }
}; // command_pins_t

struct rele_ext_pins_t {
  irs::gpio_pin_t* SYM_2V_on;
  irs::gpio_pin_t* SYM_2V_off;
  irs::gpio_pin_t* SYM_20V_on;
  irs::gpio_pin_t* SYM_20V_off;
  irs::gpio_pin_t* SYM_200V_on;
  irs::gpio_pin_t* SYM_200V_off;
  irs::gpio_pin_t* KZ_2V_on;
  irs::gpio_pin_t* KZ_2V_off;
  irs::gpio_pin_t* KZ_1A;
  irs::gpio_pin_t* KZ_17A;
  irs::gpio_pin_t* REL_220V;
  irs::gpio_pin_t* SYM_OFF;
  irs::gpio_pin_t* SYM_OFF_TEST;

  rele_ext_pins_t(
    irs::gpio_pin_t* ap_SYM_2V_on,
    irs::gpio_pin_t* ap_SYM_2V_off,
    irs::gpio_pin_t* ap_SYM_20V_on,
    irs::gpio_pin_t* ap_SYM_20V_off,
    irs::gpio_pin_t* ap_SYM_200V_on,
    irs::gpio_pin_t* ap_SYM_200V_off,
    irs::gpio_pin_t* ap_KZ_2V_on,
    irs::gpio_pin_t* ap_KZ_2V_off,
    irs::gpio_pin_t* ap_KZ_1A,
    irs::gpio_pin_t* ap_KZ_17A,
    irs::gpio_pin_t* ap_REL_220V,
    irs::gpio_pin_t* ap_SYM_OFF,
    irs::gpio_pin_t* ap_SYM_OFF_TEST
  ):
    SYM_2V_on(ap_SYM_2V_on),
    SYM_2V_off(ap_SYM_2V_off),
    SYM_20V_on(ap_SYM_20V_on),
    SYM_20V_off(ap_SYM_20V_off),
    SYM_200V_on(ap_SYM_200V_on),
    SYM_200V_off(ap_SYM_200V_off),
    KZ_2V_on(ap_KZ_2V_on),
    KZ_2V_off(ap_KZ_2V_off),
    KZ_1A(ap_KZ_1A),
    KZ_17A(ap_KZ_17A),
    REL_220V(ap_REL_220V),
    SYM_OFF(ap_SYM_OFF),
    SYM_OFF_TEST(ap_SYM_OFF_TEST)
  {
  }
}; // rele_ext_pins_t

struct meas_comm_th_pins_t
{
  irs::gpio_pin_t* termo_sense_1;
  irs::gpio_pin_t* termo_sense_2;
  irs::gpio_pin_t* termo_sense_3;
  irs::gpio_pin_t* termo_sense_4;
  irs::gpio_pin_t* termo_sense_5;
  irs::gpio_pin_t* izm_th_enable;

  meas_comm_th_pins_t(
    irs::gpio_pin_t* ap_termo_sense_1,
    irs::gpio_pin_t* ap_termo_sense_2,
    irs::gpio_pin_t* ap_termo_sense_3,
    irs::gpio_pin_t* ap_termo_sense_4,
    irs::gpio_pin_t* ap_termo_sense_5,
    irs::gpio_pin_t* ap_izm_th_enable
  ):
    termo_sense_1(ap_termo_sense_1),
    termo_sense_2(ap_termo_sense_2),
    termo_sense_3(ap_termo_sense_3),
    termo_sense_4(ap_termo_sense_4),
    termo_sense_5(ap_termo_sense_5),
    izm_th_enable(ap_izm_th_enable)
  {
  }
}; // meas_comm_th_pins_t

class cfg_t
{
public:
  cfg_t();
  command_pins_t* command_pins();
  irs::adc_t* adc();
  irs::arm::arm_spi_t* spi_meas_comm_plis();
  irs::arm::arm_spi_t* spi_supply_comm_plis();
  irs::arm::arm_spi_t* spi_general_purpose();
  irs::hardflow::simple_udp_flow_t* hardflow();
  rele_ext_pins_t* rele_ext_pins();
  meas_comm_th_pins_t* meas_comm_th_pins();
  void izm_th_spi_enable_pin_set(bool a_value);
  plis_pins_t& supply_comm_pins();
  irs::pwm_gen_t& supply_tact_gen();
  irs::gpio_pin_t* pins_eeprom();
  #ifdef OLD_MEAS_COMM
  meas_comm_pins_t* meas_pins();
  #else //  OLD_MEAS_COMM
  plis_pins_t& meas_comm_pins();
  irs::pwm_gen_t& meas_tact_gen();
  #endif  //  OLD_MEAS_COMM
private:
  enum {
    CS_TR_3 = 0,
    CS_TR_4 = 1,
    CS_TR_1 = 2,
    CS_TR_2 = 3,
    CS_TR_5 = 4,
    IZM_TH_CS_1 = 5,
    CS_TH1_17A = 6,
    CS_TH2_17A = 7,
    CS_ADC_17A = 8,
    CS_TC_17A = 9,
    CS_TC_1A = 10,
    IZM_TH_CS_3 = 11,
    IZM_TH_CS_2 = 12,
    IZM_TH_CS_5 = 13,
    IZM_TH_CS_4 = 14,
    CS_EE = 15,
    CS_TH1_1A = 16,
    CS_TH2_1A = 17,
    CS_ADC_1A = 18,
    CS_TC_20V = 19,
    CS_TH1_20V = 20,
    CS_TH2_20V = 21,
    CS_ADC_20V = 22,
    CS_TC_200V = 23,
    CS_TH1_200V = 24,
    CS_TH2_200V = 25,
    CS_ADC_200V = 26,
    CS_ADC_2V = 27,
    CS_TH2_2V = 28,
    CS_TH1_2V = 29,
    CS_TC_2V = 30,
    CS_PLIS = 31,

    CS_DAC_17A = 1,
    CS_DAC_1A = 2,
    CS_DAC_2V = 3,
    CS_DAC_20V = 4,
    CS_DAC_200V = 5
  };
  enum {
    PTC_A_channel = 15,
    PTC_LC_channel = 14,
    TR_24V_TEST_channel = 7,
    IZM_3_3V_TEST_channel = 6,
    IZM_6V_TEST_channel = 5,
    IZM_1_2V_TEST_channel = 4,
    TEST_24V_channel = 3,
    TEST_5V_channel = 2,
    PTC_PWR_channel = 1,
    PTC_17A_channel = 0
  };

  irs::arm::io_pin_t m_spi_cs_code_0;
  irs::arm::io_pin_t m_spi_cs_code_1;
  irs::arm::io_pin_t m_spi_cs_code_2;
  irs::arm::io_pin_t m_spi_cs_code_3;
  irs::arm::io_pin_t m_spi_cs_code_4;
  irs::arm::io_pin_t m_spi_cs_enable;
  spi_demux_t::spi_demux_cs_data_t m_spi_demux_cs_data;
  spi_demux_t m_spi_demux;

  irs::arm::io_pin_t m_supply_comm_cfg_done;
  irs::arm::io_pin_t m_supply_comm_reset;
  plis_pins_t m_supply_comm_pins;

  #ifdef OLD_MEAS_COMM
  irs::arm::io_pin_t m_meas_comm_cs;
  irs::arm::io_pin_t m_meas_comm_reset;
  irs::arm::io_pin_t m_meas_comm_apply;
  irs::arm::io_pin_t m_meas_comm_error;
  meas_comm_pins_t m_meas_comm_pins;
  #else //  OLD_MEAS_COMM
  irs::arm::io_pin_t m_meas_comm_cfg_done;
  irs::arm::io_pin_t m_meas_comm_cs;
  irs::arm::io_pin_t m_meas_comm_reset;
  plis_pins_t m_meas_comm_pins;
  #endif  //  OLD_MEAS_COMM

  irs::arm::io_pin_t m_izm_th_enable;
  meas_comm_th_pins_t m_meas_comm_th_pins;

  irs::arm::io_pin_t m_dac_cs_code_0;
  irs::arm::io_pin_t m_dac_cs_code_1;
  irs::arm::io_pin_t m_dac_cs_code_2;
  dac_demux_t::dac_demux_cs_data_t m_dac_demux_cs_data;
  dac_demux_t m_dac_demux;

  supply_pins_t m_supply_200V_pins;
  supply_pins_t m_supply_20V_pins;
  supply_pins_t m_supply_2V_pins;
  supply_pins_t m_supply_1A_pins;
  supply_pins_t m_supply_17A_pins;
  command_pins_t m_command_pins;

  irs::arm::io_pin_t m_SYM_2V_on;
  irs::arm::io_pin_t m_SYM_2V_off;
  irs::arm::io_pin_t m_SYM_20V_on;
  irs::arm::io_pin_t m_SYM_20V_off;
  irs::arm::io_pin_t m_SYM_200V_on;
  irs::arm::io_pin_t m_SYM_200V_off;
  irs::arm::io_pin_t m_KZ_2V_on;
  irs::arm::io_pin_t m_KZ_2V_off;
  irs::arm::io_pin_t m_KZ_1A;
  irs::arm::io_pin_t m_KZ_17A;
  irs::arm::io_pin_t m_REL_220V;
  irs::arm::io_pin_t m_SYM_OFF;
  irs::arm::io_pin_t m_SYM_OFF_TEST;
  rele_ext_pins_t m_rele_ext_pins;

  irs_u32 m_f_osc;
  irs::arm::adc_stellaris_t m_adc;
  irs_u32 m_spi_bitrate;
  irs_u8 m_spi_buf_size;
  irs::arm::arm_spi_t m_spi_meas_comm_plis;
  irs::arm::arm_spi_t m_spi_general_purpose;

  irs::arm::gptm_generator_t m_supply_tact_gen;
  #ifndef OLD_MEAS_COMM
  irs::arm::gptm_generator_t m_meas_tact_gen;
  #endif  //  OLD_MEAS_COMM

  mxmac_t m_local_mac;
  irs::arm::arm_ethernet_t m_arm_eth;
  mxip_t m_local_ip;
  irs_u16 m_local_port;
  mxip_t m_dest_ip;
  irs_u16 m_dest_port;
  irs::simple_tcpip_t m_tcpip;
  irs::hardflow::simple_udp_flow_t m_simple_hardflow;
};

} // namespace u309m

#endif // cfgh
