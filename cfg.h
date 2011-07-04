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
#include <irsmbus.h>
#include <irsmem.h>
#include <armflash.h>

#include "comm.h"
#include "data.h"
#include "demux.h"

#include <irsfinal.h>

#define MEAS_COMM_TEST
#define SUPPLY_COMM_TEST
#define SUPPLY_TEST
#define EEPROM_TEST
#define ARM_ADC_TEST // work
#define USE_FLASH

namespace u309m {

struct rele_ext_eth_data_t {
  irs::bit_data_t SYM_2V;
  irs::bit_data_t SYM_20V;
  irs::bit_data_t SYM_200V;
  irs::bit_data_t KZ_2V;
  irs::bit_data_t KZ_1A;
  irs::bit_data_t KZ_17A;
  irs::bit_data_t REL_220V;
  irs::bit_data_t SYM_OFF;
  irs::bit_data_t SYM_OFF_TEST;

  rele_ext_eth_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    SYM_2V.connect(ap_data, index, 0);
    SYM_20V.connect(ap_data, index, 1);
    SYM_200V.connect(ap_data, index, 2);
    KZ_2V.connect(ap_data, index, 3);
    KZ_1A.connect(ap_data, index, 4);
    KZ_17A.connect(ap_data, index, 5);
    REL_220V.connect(ap_data, index, 6);
    SYM_OFF.connect(ap_data, index, 7);
    index++;
    SYM_OFF_TEST.connect(ap_data, index, 0);
    index++;

    return index;
  }
}; // rele_ext_eth_data_t

struct control_data_t
{
  irs::conn_data_t<irs_u32> alarm;
  irs::bit_data_t alarm_internal_th;
  irs::bit_data_t alarm_ptc_a;
  irs::bit_data_t alarm_ptc_lc;
  irs::bit_data_t alarm_ptc_pwr;
  irs::bit_data_t alarm_ptc_17A;
  irs::bit_data_t alarm_tr_24V;
  irs::bit_data_t alarm_24V;
  irs::bit_data_t alarm_5V;
  irs::bit_data_t alarm_izm_6V;
  irs::bit_data_t alarm_izm_3_3V;
  irs::bit_data_t alarm_izm_1_2V;
  irs::bit_data_t alarm_izm_th1;
  irs::bit_data_t alarm_izm_th2;
  irs::bit_data_t alarm_izm_th3;
  irs::bit_data_t alarm_izm_th4;
  irs::bit_data_t alarm_izm_th5;
  irs::bit_data_t alarm_200V_th_base;
  irs::bit_data_t alarm_200V_th_aux;
  irs::bit_data_t alarm_20V_th_base;
  irs::bit_data_t alarm_20V_th_aux;
  irs::bit_data_t alarm_2V_th_base;
  irs::bit_data_t alarm_2V_th_aux;
  irs::bit_data_t alarm_1A_th_base;
  irs::bit_data_t alarm_1A_th_aux;
  irs::bit_data_t alarm_17A_th_base;
  irs::bit_data_t alarm_17A_th_aux;

  irs::bit_data_t on;

  irs::conn_data_t<irs_u8> unlock;

  irs::bit_data_t ready_200V_prev;
  irs::bit_data_t ready_200V_final;
  irs::bit_data_t ready_20V_prev;
  irs::bit_data_t ready_20V_final;
  irs::bit_data_t ready_2V_prev;
  irs::bit_data_t ready_2V_final;
  irs::bit_data_t ready_1A_prev;
  irs::bit_data_t ready_1A_final;
  irs::bit_data_t ready_17A_prev;
  irs::bit_data_t ready_17A_final;

  control_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    alarm_internal_th.connect(ap_data, index, 0);
    alarm_ptc_a.connect(ap_data, index, 1);
    alarm_ptc_lc.connect(ap_data, index, 2);
    alarm_ptc_pwr.connect(ap_data, index, 3);
    alarm_ptc_17A.connect(ap_data, index, 4);
    alarm_tr_24V.connect(ap_data, index, 5);
    alarm_24V.connect(ap_data, index, 6);
    alarm_5V.connect(ap_data, index, 7);

    alarm_izm_6V.connect(ap_data, index, 0);
    alarm_izm_3_3V.connect(ap_data, index, 1);
    alarm_izm_1_2V.connect(ap_data, index, 2);
    alarm_izm_th1.connect(ap_data, index, 3);
    alarm_izm_th2.connect(ap_data, index, 4);
    alarm_izm_th3.connect(ap_data, index, 5);
    alarm_izm_th4.connect(ap_data, index, 6);
    alarm_izm_th5.connect(ap_data, index, 7);

    alarm_200V_th_base.connect(ap_data, index, 0);
    alarm_200V_th_aux.connect(ap_data, index, 1);
    alarm_20V_th_base.connect(ap_data, index, 2);
    alarm_20V_th_aux.connect(ap_data, index, 3);
    alarm_2V_th_base.connect(ap_data, index, 4);
    alarm_2V_th_aux.connect(ap_data, index, 5);
    alarm_1A_th_base.connect(ap_data, index, 6);
    alarm_1A_th_aux.connect(ap_data, index, 7);

    alarm_17A_th_base.connect(ap_data, index, 0);
    alarm_17A_th_aux.connect(ap_data, index, 1);

    on.connect(ap_data, index, 7);

    index = alarm.connect(ap_data, index);

    index = unlock.connect(ap_data, index);

    ready_200V_prev.connect(ap_data, index, 0);
    ready_200V_final.connect(ap_data, index, 1);
    ready_20V_prev.connect(ap_data, index, 2);
    ready_20V_final.connect(ap_data, index, 3);
    ready_2V_prev.connect(ap_data, index, 4);
    ready_2V_final.connect(ap_data, index, 5);
    ready_1A_prev.connect(ap_data, index, 6);
    ready_1A_final.connect(ap_data, index, 7);

    index++;

    ready_17A_prev.connect(ap_data, index, 0);
    ready_17A_final.connect(ap_data, index, 1);

    index++;
    index++;

    return index;
  }
}; // control_data_t

struct eth_data_t {
  irs::conn_data_t<irs_u8> ip_0;  //  1 byte
  irs::conn_data_t<irs_u8> ip_1;  //  1 byte
  irs::conn_data_t<irs_u8> ip_2;  //  1 byte
  irs::conn_data_t<irs_u8> ip_3;  //  1 byte
  rele_ext_eth_data_t rele_ext;   //  2 bytes
  supply_comm_data_t supply_comm; //  4 bytes
  meas_comm_data_t meas_comm;     //  4 bytes
  arm_adc_data_t arm_adc;         //  44 bytes
  supply_eth_data_t supply_200V;  //  114 bytes
  supply_eth_data_t supply_20V;   //  114 bytes
  supply_eth_data_t supply_2V;    //  114 bytes
  supply_eth_data_t supply_1A;    //  114 bytes
  supply_eth_data_t supply_17A;   //  114 bytes
  control_data_t control;         //  8 bytes
  //---------------------------------------------
  //                          Итого:  636 байт

  eth_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }
  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    index = ip_0.connect(ap_data, index);
    index = ip_1.connect(ap_data, index);
    index = ip_2.connect(ap_data, index);
    index = ip_3.connect(ap_data, index);
    index = rele_ext.connect(ap_data, index);
    index = supply_comm.connect(ap_data, index);
    index = meas_comm.connect(ap_data, index);
    index = arm_adc.connect(ap_data, index);
    index = supply_200V.connect(ap_data, index);
    index = supply_20V.connect(ap_data, index);
    index = supply_2V.connect(ap_data, index);
    index = supply_1A.connect(ap_data, index);
    index = supply_17A.connect(ap_data, index);
    index = control.connect(ap_data, index);

    return index;
  }
  void reset_to_default(supply_type_t a_supply_type)
  {
    ip_0 = 192;
    ip_1 = 168;
    ip_2 = 0;
    ip_3 = 211;
    switch(a_supply_type)
    {
      case sup_200V:
      {
        supply_200V.prev_adc_data.koef = (4.096/1024)*(23.9/3.9)*1.2;
        supply_200V.fin_adc_data.koef = (4.096/1024)*(23.9/3.9)*1.2;
        supply_200V.prev_dac_data.koef = 2633;
        supply_200V.fin_dac_data.koef = 2702;

        supply_200V.base_tr_data.temperature_ref = 60;
        supply_200V.base_tr_data.temp_k = 15000;
        supply_200V.base_tr_data.temp_ki = 0.00075;
        supply_200V.base_tr_data.temp_kd = 200;
        supply_200V.base_tr_data.temp_prop_koef = 0;
        supply_200V.base_tr_data.temp_time_const = 20;

        supply_200V.aux_tr_data.temperature_ref = 60;
        supply_200V.aux_tr_data.temp_k = 15000;
        supply_200V.aux_tr_data.temp_ki = 0.00075;
        supply_200V.aux_tr_data.temp_kd = 200;
        supply_200V.aux_tr_data.temp_prop_koef = 0;
        supply_200V.aux_tr_data.temp_time_const = 20;
        irs::mlog() << irsm(" eeprom supply_200V") << endl;
      } break;
      case sup_20V:
      {
        supply_20V.prev_adc_data.koef = (4.096/1024)*(23.9/3.9)*1.2;
        supply_20V.fin_adc_data.koef = (4.096/1024)*(23.9/3.9)*1.2;
        supply_20V.prev_dac_data.koef = 2633;
        supply_20V.fin_dac_data.koef = 2702;

        supply_20V.base_tr_data.temperature_ref = 60;
        supply_20V.base_tr_data.temp_k = 15000;
        supply_20V.base_tr_data.temp_ki = 0.00075;
        supply_20V.base_tr_data.temp_kd = 200;
        supply_20V.base_tr_data.temp_prop_koef = 0;
        supply_20V.base_tr_data.temp_time_const = 20;

        supply_20V.aux_tr_data.temperature_ref = 60;
        supply_20V.aux_tr_data.temp_k = 15000;
        supply_20V.aux_tr_data.temp_ki = 0.00075;
        supply_20V.aux_tr_data.temp_kd = 200;
        supply_20V.aux_tr_data.temp_prop_koef = 0;
        supply_20V.aux_tr_data.temp_time_const = 20;
        irs::mlog() << irsm(" eeprom supply_20V") << endl;
      } break;
      case sup_2V:
      {
        supply_2V.prev_adc_data.koef = (4.096/1024)*(23.9/3.9)*1.2;
        supply_2V.fin_adc_data.koef = (4.096/1024)*(23.9/3.9)*1.2;
        supply_2V.prev_dac_data.koef = 2633;
        supply_2V.fin_dac_data.koef = 2702;

        supply_2V.base_tr_data.temperature_ref = 60;
        supply_2V.base_tr_data.temp_k = 15000;
        supply_2V.base_tr_data.temp_ki = 0.00075;
        supply_2V.base_tr_data.temp_kd = 200;
        supply_2V.base_tr_data.temp_prop_koef = 0;
        supply_2V.base_tr_data.temp_time_const = 20;

        supply_2V.aux_tr_data.temperature_ref = 60;
        supply_2V.aux_tr_data.temp_k = 15000;
        supply_2V.aux_tr_data.temp_ki = 0.00075;
        supply_2V.aux_tr_data.temp_kd = 200;
        supply_2V.aux_tr_data.temp_prop_koef = 0;
        supply_2V.aux_tr_data.temp_time_const = 20;
        irs::mlog() << irsm(" eeprom supply_2V") << endl;
      } break;
      case sup_1A:
      {
        supply_1A.prev_adc_data.koef = (4.096/1024)*(23.9/3.9)*1.2;
        supply_1A.fin_adc_data.koef = (4.096/1024)*(23.9/3.9)*1.2;
        supply_1A.prev_dac_data.koef = 2633;
        supply_1A.fin_dac_data.koef = 2702;

        supply_1A.base_tr_data.temperature_ref = 60;
        supply_1A.base_tr_data.temp_k = 15000;
        supply_1A.base_tr_data.temp_ki = 0.00075;
        supply_1A.base_tr_data.temp_kd = 200;
        supply_1A.base_tr_data.temp_prop_koef = 0;
        supply_1A.base_tr_data.temp_time_const = 20;

        supply_1A.aux_tr_data.temperature_ref = 60;
        supply_1A.aux_tr_data.temp_k = 15000;
        supply_1A.aux_tr_data.temp_ki = 0.00075;
        supply_1A.aux_tr_data.temp_kd = 200;
        supply_1A.aux_tr_data.temp_prop_koef = 0;
        supply_1A.aux_tr_data.temp_time_const = 20;
        irs::mlog() << irsm(" eeprom supply_1A") << endl;
      } break;
      case sup_17A:
      {
        supply_17A.prev_adc_data.koef = (4.096/1024)*(23.9/3.9)*1.2;
        supply_17A.fin_adc_data.koef = (4.096/1024)*(23.9/3.9)*1.2;
        supply_17A.prev_dac_data.koef = 2633;
        supply_17A.fin_dac_data.koef = 2702;

        supply_17A.base_tr_data.temperature_ref = 60;
        supply_17A.base_tr_data.temp_k = 15000;
        supply_17A.base_tr_data.temp_ki = 0.00075;
        supply_17A.base_tr_data.temp_kd = 200;
        supply_17A.base_tr_data.temp_prop_koef = 0;
        supply_17A.base_tr_data.temp_time_const = 20;

        supply_17A.aux_tr_data.temperature_ref = 60;
        supply_17A.aux_tr_data.temp_k = 15000;
        supply_17A.aux_tr_data.temp_ki = 0.00075;
        supply_17A.aux_tr_data.temp_kd = 200;
        supply_17A.aux_tr_data.temp_prop_koef = 0;
        supply_17A.aux_tr_data.temp_time_const = 20;
        irs::mlog() << irsm(" eeprom supply_17A") << endl;
      } break;
      default:
      {
        IRS_LIB_ASSERT_MSG("неверно указан тип источника");
      }
    }
  }
};

struct command_pins_t {
  meas_comm_pins_t* meas_comm;
  supply_comm_pins_t* supply_comm;
  supply_pins_t* supply_200V;
  supply_pins_t* supply_20V;
  supply_pins_t* supply_2V;
  supply_pins_t* supply_1A;
  supply_pins_t* supply_17A;

  command_pins_t(
    meas_comm_pins_t* ap_meas_comm_pins,
    supply_comm_pins_t* ap_supply_comm_pins,
    supply_pins_t* ap_supply_200V_pins,
    supply_pins_t* ap_supply_20V_pins,
    supply_pins_t* ap_supply_2V_pins,
    supply_pins_t* ap_supply_1A_pins,
    supply_pins_t* ap_supply_17A_pins
  ):
    meas_comm(ap_meas_comm_pins),
    supply_comm(ap_supply_comm_pins),
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

class cfg_t
{
public:
  cfg_t();
  command_pins_t* command_pins();
  irs::adc_t* adc();
  irs::arm::arm_spi_t* spi_meas_comm_plis();
  irs::arm::arm_spi_t* spi_supply_comm_plis();
  irs::arm::arm_spi_t* spi_general_purpose();
  eth_data_t* eth_data();
  eeprom_data_t* eeprom_data();
  irs::hardflow::simple_udp_flow_t* hardflow();
  rele_ext_pins_t* rele_ext_pins();
  meas_comm_t* meas_comm();
  supply_comm_t* supply_comm();
  void tick();

private:
  class plis_ready_t
  {
  public:
    plis_ready_t(irs::gpio_pin_t* ap_cfg_done_pin, counter_t a_counter):
      m_timer(a_counter)
    {
      m_timer.start();
      while(!ap_cfg_done_pin->pin())
      {
        if (m_timer.check()) {
          irs::mlog() << "PLIS not configured" << endl;
          break;
        }
      }
    }
  private:
    irs::timer_t m_timer;
  }; // plis_ready_t

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

  irs::arm::io_pin_t m_supply_comm_cfg_done;
  plis_ready_t m_ipt_plis_ready;
  irs::arm::io_pin_t m_meas_comm_cfg_done;
  plis_ready_t m_meas_plis_ready;
  irs_u8 m_spi_buf_size;
  irs_u32 m_f_osc;
  irs::arm::adc_stellaris_t m_adc;
  irs::arm::arm_spi_t m_spi_meas_comm_plis;
  irs::arm::arm_spi_t m_spi_general_purpose;

  irs::arm::io_pin_t m_spi_cs_code_0;
  irs::arm::io_pin_t m_spi_cs_code_1;
  irs::arm::io_pin_t m_spi_cs_code_2;
  irs::arm::io_pin_t m_spi_cs_code_3;
  irs::arm::io_pin_t m_spi_cs_code_4;
  irs::arm::io_pin_t m_spi_cs_enable;
  spi_demux_t::spi_demux_cs_data_t m_spi_demux_cs_data;
  spi_demux_t m_spi_demux;

  irs::arm::io_pin_t m_dac_cs_code_0;
  irs::arm::io_pin_t m_dac_cs_code_1;
  irs::arm::io_pin_t m_dac_cs_code_2;
  dac_demux_t::dac_demux_cs_data_t m_dac_demux_cs_data;
  dac_demux_t m_dac_demux;

  mxmac_t m_local_mac;
  irs::arm::arm_ethernet_t m_arm_eth;
  mxip_t m_local_ip;
  irs_u16 m_local_port;
  mxip_t m_dest_ip;
  irs_u16 m_dest_port;
  irs::simple_tcpip_t m_tcpip;
  irs::hardflow::simple_udp_flow_t m_simple_hardflow;
  irs::modbus_server_t m_modbus_server;
  eth_data_t m_eth_data;

  irs::arm::io_pin_t m_izm_th_enable;
  irs::arm::io_pin_t m_meas_comm_cs;
  irs::arm::io_pin_t m_meas_comm_reset;
  irs::arm::io_pin_t m_meas_comm_apply;
  irs::arm::io_pin_t m_meas_comm_error;
  meas_comm_pins_t m_meas_comm_pins;

  irs::arm::io_pin_t m_supply_comm_reset;
  supply_comm_pins_t m_supply_comm_pins;

  supply_pins_t m_supply_200V_pins;
  supply_pins_t m_supply_20V_pins;
  supply_pins_t m_supply_2V_pins;
  supply_pins_t m_supply_1A_pins;
  supply_pins_t m_supply_17A_pins;
  command_pins_t m_command_pins;

  #ifdef MEAS_COMM_TEST
  meas_comm_t m_meas_comm;
  #endif // MEAS_COMM_TEST
  #ifdef SUPPLY_COMM_TEST
  supply_comm_t m_supply_comm;
  #endif // SUPPLY_COMM_TEST

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

  #ifdef EEPROM_TEST
  irs::eeprom_command_t::size_type m_eeprom_size;
  #ifdef USE_FLASH
    irs::arm::flash_t m_eeprom;
  #else
    irs::eeprom_command_t m_eeprom_command;
    irs::eeprom_spi_t m_eeprom;
  #endif  //  USE_FLASH
  eeprom_data_t m_eeprom_data;
  #endif // EEPROM_TEST

  irs::loop_timer_t m_timer;
};

} // namespace u309m

#endif // cfgh
