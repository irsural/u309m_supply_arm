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

#include "data.h"
#include "demux.h"

#include <irsfinal.h>

namespace u309m {

struct eth_data_t {
  irs::conn_data_t<irs_u8> ip_0;
  irs::conn_data_t<irs_u8> ip_1;
  irs::conn_data_t<irs_u8> ip_2;
  irs::conn_data_t<irs_u8> ip_3;
  arm_adc_data_t arm_adc;
  supply_comm_data_t supply_comm;
  meas_comm_data_t meas_comm;
  supply_eth_data_t supply_200V;
  supply_eth_data_t supply_20V;
  supply_eth_data_t supply_2V;
  supply_eth_data_t supply_1A;
  supply_eth_data_t supply_17A;

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
    index = arm_adc.connect(ap_data, index);
    index = supply_comm.connect(ap_data, index);
    index = meas_comm.connect(ap_data, index);
    index = supply_200V.connect(ap_data, index);
    index = supply_20V.connect(ap_data, index);
    index = supply_2V.connect(ap_data, index);
    index = supply_1A.connect(ap_data, index);
    index = supply_17A.connect(ap_data, index);
    
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

class cfg_t
{
public:
  cfg_t();
  command_pins_t* command_pins();
  irs::arm::adc_t* adc();
  irs::arm::arm_spi_t* spi_meas_comm_plis();
  irs::arm::arm_spi_t* spi_supply_comm_plis();
  irs::arm::arm_spi_t* spi_general_purpose();
  eth_data_t* eth_data();
  eeprom_data_t* eeprom_data();
  irs::hardflow::simple_udp_flow_t* hardflow();
  void tick();
  
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
    PTC_A_channel = 0,
    PTC_LC_channel = 1,
    TR_24V_TEST_channel = 2,
    IZM_3_3V_TEST_channel = 3,
    IZM_6V_TEST_channel = 4,
    IZM_1_2V_TEST_channel = 5,
    TEST_24V_channel = 6,
    TEST_5V_channel = 7,
    PTC_PWR_channel = 8,
    PTC_17A_channel = 9
  };
  irs_u8 m_spi_buf_size;
  irs_u32 m_f_osc;
  irs::arm::adc_t m_adc;
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
  
  irs::eeprom_command_t::size_type m_eeprom_size;
  irs::eeprom_command_t m_eeprom_command;
  irs::eeprom_spi_t m_eeprom;
  eeprom_data_t m_eeprom_data;
  
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
  
  irs::loop_timer_t m_timer;
};

} // namespace u309m

#endif // cfgh
