#ifndef commh
#define commh

#include <irsdefs.h>

#include <irsadc.h>

#include <armspi.h>
#include <armadc.h>

#include "data.h"

#include <irsfinal.h>

namespace u309m {

class plis_t
{
public:
  enum plis_status_t {
    BUSY = 1,
    COMPLETE = 2,
    ERROR = 3
  };
  
  plis_t(
    irs_u32 a_tact_freq,
    irs::arm::arm_spi_t* ap_spi,
    irs::gpio_pin_t* ap_cs_pin
  );
  ~plis_t();
  void write(const irs_u8 *ap_command);
  void tact_on();
  void tact_off();
  void tick();
private:
  enum {
    CCP5 = 0x4,
    TIMER_16_BIT = 0x4,
    PERIODIC_MODE = 0x2
  };
  enum status_t
  {
    PLIS_SPI_FREE,
    PLIS_SPI_WRITE
  };
  enum {
    m_size = 2
  };
  irs_u32 m_tact_freq;
  irs::arm::arm_spi_t* mp_spi;
  irs::gpio_pin_t* mp_cs_pin;
  irs_u8 mp_buf[m_size];
  status_t m_status;
  bool m_need_write;
}; // plis_t

class meas_comm_t
{
public:
  meas_comm_t(
    irs::arm::adc_t* ap_adc,
    irs::arm::arm_spi_t* ap_spi_term,
    irs::arm::arm_spi_t* ap_spi_meas_comm_plis,
    meas_comm_pins_t* ap_meas_comm_pins,
    meas_comm_data_t* ap_meas_comm_data
  );
  void make_command();
  void init_default();
  void tick();
  
private:
  enum meas_mode_t {
    COILS,
    NORMAL_ELEMENTS,
    RES_BOX
  };
  enum tick_mode_t {
    command_check,
    meas_on_start,
    meas_on_busy,
    meas_on_complete,
    meas_apply_start,
    meas_apply_busy,
    meas_apply_complete,
    meas_reset_start,
    meas_reset_complete
  };

  irs::arm::adc_t* mp_adc;
  meas_comm_pins_t* mp_meas_comm_pins;
  meas_comm_data_t* mp_meas_comm_data;
  irs::th_lm95071_t m_th1;
  irs::th_lm95071_data_t m_th1_data;
  irs::th_lm95071_t m_th2;
  irs::th_lm95071_data_t m_th2_data;
  irs::th_lm95071_t m_th3;
  irs::th_lm95071_data_t m_th3_data;
  irs::th_lm95071_t m_th4;
  irs::th_lm95071_data_t m_th4_data;
  irs::th_lm95071_t m_th5;
  irs::th_lm95071_data_t m_th5_data;
  irs::loop_timer_t m_timer;
  plis_t m_plis;
  bool m_command_apply;
  bool m_comm_on;
  bool m_plis_reset;
  irs_u16 m_command;
  tick_mode_t m_mode;
}; // meas_comm_t

class supply_comm_t
{
public:
  supply_comm_t(
    irs::arm::arm_spi_t* ap_spi,
    supply_comm_pins_t* ap_supply_comm_pins,
    supply_comm_data_t* ap_supply_comm_data
  );
  void make_command();
  void tick();
private:
  enum tick_mode_t {
    command_check
  };
  
  supply_comm_pins_t* mp_supply_comm_pins;
  supply_comm_data_t* mp_supply_comm_data;
  plis_t m_plis;
  bool m_command_apply;
  bool m_comm_on;
  irs_u16 m_command;
  tick_mode_t m_mode;
}; // supply_comm_t

} // namespace u309m

#endif // commh
