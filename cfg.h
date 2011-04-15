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

#include "data.h"

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
  irs::arm::arm_spi_t* spi_term();
  eth_data_t* eth_data();
  void tick();
  
private:
  irs_u8 m_spi_buf_size;
  irs_u32 m_f_osc;
  irs::arm::adc_t m_adc;
  irs::arm::arm_spi_t m_spi_meas_comm_plis;
  irs::arm::arm_spi_t m_spi_meas_comm_term;
  irs::arm::io_pin_t m_meas_comm_cs;
  irs::arm::io_pin_t m_meas_comm_reset;
  irs::arm::io_pin_t m_meas_comm_apply;
  irs::arm::io_pin_t m_meas_comm_error;
  irs::arm::io_pin_t m_meas_termo_sense_1_pin;
  irs::arm::io_pin_t m_meas_termo_sense_2_pin;
  irs::arm::io_pin_t m_meas_termo_sense_3_pin;
  irs::arm::io_pin_t m_meas_termo_sense_4_pin;
  irs::arm::io_pin_t m_meas_termo_sense_5_pin;
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
  meas_comm_pins_t m_meas_comm_pins;
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
