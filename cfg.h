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
  supply_comm_data_t supply_comm_data;
  meas_comm_data_t meas_comm_data;
  supply_eth_data_t supply_200V_data;
  supply_eth_data_t supply_20V_data;
  supply_eth_data_t supply_2V_data;
  supply_eth_data_t supply_1A_data;
  supply_eth_data_t supply_17A_data;

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
    
    index = supply_comm_data.connect(ap_data, index);
    index = meas_comm_data.connect(ap_data, index);
    index = supply_200V_data.connect(ap_data, index);
    index = supply_20V_data.connect(ap_data, index);
    index = supply_2V_data.connect(ap_data, index);
    index = supply_1A_data.connect(ap_data, index);
    index = supply_17A_data.connect(ap_data, index);
    
    return index;
  }
};

struct command_pins_t {
  meas_comm_pins_t* meas_comm_pins;
  supply_comm_pins_t* supply_comm_pins;
  supply_pins_t* supply_200V_pins;
  supply_pins_t* supply_20V_pins;
  supply_pins_t* supply_2V_pins;
  supply_pins_t* supply_1A_pins;
  supply_pins_t* supply_17A_pins;
  
  command_pins_t(
    meas_comm_pins_t* ap_meas_comm_pins,
    supply_comm_pins_t* ap_supply_comm_pins,
    supply_pins_t* ap_supply_200V_pins,
    supply_pins_t* ap_supply_20V_pins,
    supply_pins_t* ap_supply_2V_pins,
    supply_pins_t* ap_supply_1A_pins,
    supply_pins_t* ap_supply_17A_pins
  ):
    meas_comm_pins(ap_meas_comm_pins),
    supply_comm_pins(ap_supply_comm_pins),
    supply_200V_pins(ap_supply_200V_pins),
    supply_20V_pins(ap_supply_20V_pins),
    supply_2V_pins(ap_supply_2V_pins),
    supply_1A_pins(ap_supply_1A_pins),
    supply_17A_pins(ap_supply_17A_pins)
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
};

} // namespace u309m

#endif // cfgh
