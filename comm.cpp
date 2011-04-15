#include <irsdefs.h>

#include <irscpu.h>
#include "comm.h"

#include <irsfinal.h>

u309m::plis_t::plis_t (
  irs_u32 a_tact_freq,
  irs::arm::arm_spi_t* ap_spi,
  irs::gpio_pin_t* ap_cs_pin
):
  m_tact_freq(a_tact_freq),
  mp_spi(ap_spi),
  mp_cs_pin(ap_cs_pin),
  m_status(PLIS_SPI_FREE),
  m_need_write(false)
{
  memset((void*)mp_buf, 0, m_size);
  
  // Timer Initialization
  RCGC1_bit.TIMER2 = 1;
  RCGC2_bit.PORTD = 1;
  //RCGC2_bit.PORTG = 1;
  for (irs_u8 i = 10; i; i--);
  
  GPIODDEN_bit.no2 = 1;
  GPIODAFSEL_bit.no2 = 1;
  GPIODPCTL_bit.PMC2 = CCP5;
  
  GPTM2CTL_bit.TBEN = 0;
  
  GPTM2CFG_bit.GPTMCFG = TIMER_16_BIT;
  GPTM2TBMR_bit.TBAMS = 1;
  GPTM2TBMR_bit.TBCMR = 0;
  GPTM2TBMR_bit.TBMR = PERIODIC_MODE;
  GPTM2CTL_bit.TBPWML = 0;
  GPTM2TBILR = static_cast<irs_u16>(irs::cpu_traits_t::frequency()/a_tact_freq);
  GPTM2TBMATCHR = GPTM2TBILR/2;
  
  GPTM2ICR_bit.TBMCINT = 1;
  GPTM2ICR_bit.TBTOCINT = 1;
  
  for (; mp_spi->get_status() != irs::spi_t::FREE; )
    mp_spi->tick();
  mp_spi->set_order(irs::spi_t::MSB);
  mp_spi->set_polarity(irs::spi_t::RISING_EDGE);
  mp_spi->set_phase(irs::spi_t::LEAD_EDGE);

  mp_cs_pin->set();
}

u309m::plis_t::~plis_t()
{
  
}

void u309m::plis_t::write(const irs_u8* ap_command)
{
  mp_buf[0] = ap_command[1];
  mp_buf[1] = ap_command[0];
  m_need_write = true;
}

void u309m::plis_t::tact_on()
{
  GPTM2CTL_bit.TBEN = 1;
}

void u309m::plis_t::tact_off()
{
  GPTM2CTL_bit.TBEN = 0;
}

void u309m::plis_t::tick()
{
  switch (m_status) {
    case PLIS_SPI_FREE:
    {
      if (m_need_write && (mp_spi->get_status() == irs::spi_t::FREE)) {
        if (!mp_spi->get_lock()) {
          mp_spi->set_order(irs::spi_t::MSB);
          mp_spi->set_polarity(irs::spi_t::RISING_EDGE);
          mp_spi->set_phase(irs::spi_t::LEAD_EDGE);
          mp_spi->lock();
          mp_cs_pin->clear();
          mp_spi->write(mp_buf, m_size);
          m_status = PLIS_SPI_WRITE;
          m_need_write = false;
        }
      }
    } break;
    case PLIS_SPI_WRITE:
    {
      if (mp_spi->get_status() == irs::spi_t::FREE) {
        mp_cs_pin->set();
        mp_spi->unlock();
        m_status = PLIS_SPI_FREE;
      }
    } break;
  }
}

//---------------------------- meas_comm ---------------------------------------

u309m::meas_comm_t::meas_comm_t(
  irs::arm::adc_t* ap_adc,
  irs::arm::arm_spi_t* ap_spi_term,
  irs::arm::arm_spi_t* ap_spi_meas_comm_plis,
  meas_comm_pins_t* ap_meas_comm_pins,
  meas_comm_data_t* ap_meas_comm_data
):
  mp_adc(ap_adc),
  mp_meas_comm_pins(ap_meas_comm_pins),
  mp_meas_comm_data(ap_meas_comm_data),
  m_th1(ap_spi_term, mp_meas_comm_pins->termo_sense_1,
    irs::make_cnt_ms(300)),
  m_th1_data(&m_th1),
  m_th2(ap_spi_term, mp_meas_comm_pins->termo_sense_2,
    irs::make_cnt_ms(300)),
  m_th2_data(&m_th2),
  m_th3(ap_spi_term, mp_meas_comm_pins->termo_sense_3,
    irs::make_cnt_ms(300)),
  m_th3_data(&m_th3),
  m_th4(ap_spi_term, mp_meas_comm_pins->termo_sense_4,
    irs::make_cnt_ms(300)),
  m_th4_data(&m_th4),
  m_th5(ap_spi_term, mp_meas_comm_pins->termo_sense_5,
    irs::make_cnt_ms(300)),
  m_th5_data(&m_th5),
  m_timer(irs::make_cnt_ms(200)),
  m_plis(4000000, ap_spi_meas_comm_plis,
    mp_meas_comm_pins->cs),
  m_command_apply(false),
  m_comm_on(false),
  m_plis_reset(false),
  m_command(0),
  m_mode(command_check)
{
}

void u309m::meas_comm_t::make_command()
{
  m_command = 0;
  switch (mp_meas_comm_data->mode) {
    case COILS:
    {
      irs_u8 position = ((mp_meas_comm_data->etalon_cell) ?
        mp_meas_comm_data->etalon_cell :
        mp_meas_comm_data->calibrated_cell);
      m_command = ((mp_meas_comm_data->mode)|
        (position << 2)|
        (mp_meas_comm_data->load_resistor << 10));
    } break;
    case NORMAL_ELEMENTS:
    {
      m_command = ((mp_meas_comm_data->mode)|
        (mp_meas_comm_data->etalon_cell << 2)|
        (mp_meas_comm_data->calibrated_cell << 6)|
        (mp_meas_comm_data->load_resistor << 10));
    } break;
    case RES_BOX:
    {
      m_command = mp_meas_comm_data->mode;
    } break;
    default:
    {
    } break;
  }
}

void u309m::meas_comm_t::init_default()
{
  m_comm_on = 0;
  mp_meas_comm_data->on = m_comm_on;
  mp_meas_comm_data->mode = COILS;
  mp_meas_comm_data->calibrated_cell = 0;
  mp_meas_comm_data->etalon_cell = 0;
  mp_meas_comm_data->load_resistor = 0;
}

void u309m::meas_comm_t::tick()
{
  m_th1.tick();
  m_th2.tick();
  m_th3.tick();
  m_th4.tick();
  m_th5.tick();
  
  m_plis.tick();

  mp_meas_comm_data->error =
    mp_meas_comm_pins->error->pin();
  
  bool plis_reset_change =
    (m_plis_reset != mp_meas_comm_data->reset);
  if (plis_reset_change) {
    m_plis_reset = mp_meas_comm_data->reset;
    if (m_plis_reset) {
      init_default();
      mp_meas_comm_pins->reset->set();
      m_plis.tact_on();
      m_mode = meas_reset_start;
    }
  }
  
  switch (m_mode) {
    case command_check:
    {
      bool meas_comm_on_change =
        (m_comm_on != mp_meas_comm_data->on);
      if (meas_comm_on_change) {
        m_comm_on = mp_meas_comm_data->on;
        if (!m_comm_on) {
          m_mode = meas_on_start;
          break;
        }
      }
      if (m_comm_on) {
        bool meas_comm_apply_bit_change =
          (m_command_apply != mp_meas_comm_data->apply);
        if (meas_comm_apply_bit_change) {
          m_command_apply = mp_meas_comm_data->apply;
          if (m_command_apply) {
            m_mode = meas_apply_start;
          }
        }
      } else {
        if (mp_meas_comm_data->apply) {
          mp_meas_comm_data->apply = 0;
        }
      }
    } break;
    case meas_on_start:
    {
      /*if (m_comm_on) {
        make_command();
        m_plis.write(reinterpret_cast<irs_u8*>(&m_command));
        m_plis.tact_on();
        m_mode = meas_on_busy;
      } else {
        mp_meas_comm_pins->reset->set();
        m_plis.tact_on();
        m_mode = meas_reset_start;
      }*/
      mp_meas_comm_pins->reset->set();
      m_plis.tact_on();
      m_mode = meas_reset_start;
    } break;
    case meas_on_busy:
    {
      if (mp_meas_comm_pins->apply->pin()) {
        m_mode = meas_on_complete;
      }
    } break;
    case meas_on_complete:
    {
      if (!mp_meas_comm_pins->apply->pin()) {
        m_plis.tact_off();
        m_mode = command_check;
      }
    } break;
    case meas_apply_start:
    {
      make_command();
      m_plis.write(reinterpret_cast<irs_u8*>(&m_command));
      m_plis.tact_on();
      m_mode = meas_apply_busy;
    } break;
    case meas_apply_busy:
    {
      if (mp_meas_comm_pins->apply->pin()) {
        m_mode = meas_apply_complete;
      }
    } break;
    case meas_apply_complete:
    {
      if (!mp_meas_comm_pins->apply->pin()) {
        m_plis.tact_off();
        m_mode = command_check;
      }
    } break;
    case meas_reset_start:
    {
      if (mp_meas_comm_pins->apply->pin()) {
        m_mode = meas_reset_complete;
      }
    } break;
    case meas_reset_complete:
    {
      if (!mp_meas_comm_pins->apply->pin()) {
        mp_meas_comm_data->reset = 0;
        mp_meas_comm_pins->reset->clear();
        m_plis.tact_off();
        m_mode = command_check;
      }
    } break;
    default:
    {
    } break;
  }
  
  if (m_timer.check()) {  
    mp_meas_comm_data->meas_rele_power_voltage =
      (mp_adc->get_data(0)/0.73f);
    mp_meas_comm_data->power_voltage = mp_adc->get_data(1);
    mp_meas_comm_data->meas_comm_plis_voltage =
      (mp_adc->get_data(0)/0.44f);
    mp_meas_comm_data->internal_temp =
      mp_adc->get_temperature();
    mp_meas_comm_data->th1_value = 
      (m_th1_data.temperature_code*m_th1.get_conv_koef());
    mp_meas_comm_data->th2_value = 
      (m_th2_data.temperature_code*m_th2.get_conv_koef());
    mp_meas_comm_data->th3_value = 
      (m_th3_data.temperature_code*m_th3.get_conv_koef());
    mp_meas_comm_data->th4_value = 
      (m_th4_data.temperature_code*m_th4.get_conv_koef());
    mp_meas_comm_data->th5_value = 
      (m_th5_data.temperature_code*m_th5.get_conv_koef());
  }

  mp_meas_comm_data->apply =
    mp_meas_comm_pins->apply->pin();
}

//----------------------------- supply_comm ------------------------------------

u309m::supply_comm_t::supply_comm_t(
  irs::arm::arm_spi_t* ap_spi,
  supply_comm_pins_t* ap_supply_comm_pins,
  supply_comm_data_t* ap_supply_comm_data
):
  mp_supply_comm_pins(ap_supply_comm_pins),
  mp_supply_comm_data(ap_supply_comm_data),
  m_plis(4000000, ap_spi, mp_supply_comm_pins->cs),
  m_command_apply(false),
  m_comm_on(false),
  m_command(0),
  m_mode(command_check)
{
}

void u309m::supply_comm_t::make_command()
{

}

void u309m::supply_comm_t::tick()
{
  m_plis.tick();
}
