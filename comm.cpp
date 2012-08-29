#include <irsdefs.h>

#include <irscpu.h>
#include "comm.h"

#include <irsfinal.h>

//---------------------------------- ПЛИС --------------------------------------

u309m::plis_t::plis_t(plis_pins_t& a_pins, irs::pwm_gen_t& a_gen,
  irs::spi_t& a_spi):
  m_pins(a_pins),
  m_gen(a_gen),
  m_spi(a_spi),
  m_status(RESET),
  m_attempts_counter(0),
  m_reset_vector(),
  m_ready(false),
  m_need_write(false),
  m_timer(irs::make_cnt_ms(m_cfg_done_wait_interval))
{
  clear_buffers();
  m_gen.start();
  //irs::measure_time_t measure_time;
  m_timer.start();
  while (!m_pins.cfg_done.pin()) {
    if (m_timer.check()) {
      irs::mlog() << "Сбой инициализации плис (cfg_done)" << endl;
      m_status = ERROR;
      break;
    }
  }
  //irs::mlog() << "Время инициализации: " << measure_time.get() << endl;
  m_pins.cs.set();
  m_pins.reset.set();
  m_timer.set(irs::make_cnt_ms(m_reset_pulse_interval));
  m_timer.start();
}

u309m::plis_t::~plis_t()
{
}

void u309m::plis_t::reset()
{
  for (size_t i = 0; i < m_reset_vector.size(); i++) {
    *m_reset_vector[i] = true;
  }
  if (m_status == WRITE || m_status == CHECK_READY) {
    m_spi.unlock();
  }
  m_ready = false;
  m_need_write = false;
  clear_buffers();
  m_gen.start();
  m_pins.cs.set();
  m_pins.reset.set();
  m_timer.set(irs::make_cnt_ms(m_reset_pulse_interval));
  m_timer.start();
  m_status = RESET;
}

void u309m::plis_t::add_reset_flag(bool* ap_flag)
{
  if (ap_flag != IRS_NULL) {
    m_reset_vector.push_back(ap_flag);
  }
}

bool u309m::plis_t::ready()
{
  return m_ready;
}

void u309m::plis_t::write(plis_data_t a_data)
{
  plis_data_bytes_t data;
  data.data = a_data;
  mp_write_buf[0] = data.bytes.msb;
  mp_write_buf[1] = data.bytes.lsb;
  m_need_write = true;
  m_ready = false;
}

u309m::plis_data_t u309m::plis_t::read()
{
  plis_data_bytes_t data;
  data.bytes.msb = mp_read_buf[0];
  data.bytes.lsb = mp_read_buf[1];
  return data.data;
}

void u309m::plis_t::tick()
{
  m_spi.tick();
  switch (m_status)
  {
    case RESET: {
      if (m_timer.check()) {
        m_pins.reset.clear();
        m_timer.set(irs::make_cnt_ms(m_transaction_interval));
        m_timer.start();
        m_status = WAIT;
      }
      break;
    }
    case WRITE: {
      if (m_spi.get_status() == irs::spi_t::FREE) {
        m_pins.cs.set();
        m_spi.unlock();
        m_timer.set(irs::make_cnt_ms(m_ack_interval));
        m_timer.start();
        m_status = WAIT;
      }
      break;
    }
    case WAIT: {
      if (m_timer.check()) {
        m_status = ACK;
      }
      break;
    }
    case ACK: {
      if (!m_spi.get_lock() && m_spi.get_status() == irs::spi_t::FREE) {
        spi_prepare();
        command_prepare_check_ready();
        m_spi.read_write(mp_read_buf, mp_write_buf, m_size);
        #ifdef BLOCKING_SPI1
        while (m_spi.get_status() != irs::spi_t::FREE) m_spi.tick();
        #endif  //  BLOCKING_SPI1
        m_status = CHECK_READY;
      }
      break;
    }
    case CHECK_READY: {
      if (m_spi.get_status() == irs::spi_t::FREE) {
        m_pins.cs.set();
        m_spi.unlock();
        if (command_check_ready()) {
          m_gen.stop();
          m_ready = true;
          m_status = FREE;
        } else {
          if (m_attempts_counter < m_max_attempts_count) {
            m_attempts_counter++;
            m_timer.set(irs::make_cnt_ms(m_transaction_interval));
            m_timer.start();
            m_status = WAIT;
          } else {
            m_attempts_counter = 0;
            m_status = ERROR;
          }
        }
      }
      break;
    }
    case FREE: {
      if (m_need_write) {
        if (!m_spi.get_lock() && m_spi.get_status() == irs::spi_t::FREE) {
          m_need_write = false;
          m_gen.start();
          spi_prepare();
          m_spi.write(mp_write_buf, m_size);
          #ifdef BLOCKING_SPI1
          while (m_spi.get_status() != irs::spi_t::FREE) m_spi.tick();
          #endif  //  BLOCKING_SPI1
          m_attempts_counter = 0;
          m_status = WRITE;
        }
      }
      break;
    }
    case ERROR: {
      irs::mlog() << "ПЛИС не отвечает - reset" << endl;
      reset();
      break;
    }
  }
}

void u309m::plis_t::spi_prepare()
{
  m_spi.set_order(irs::spi_t::MSB);
  m_spi.set_polarity(irs::spi_t::NEGATIVE_POLARITY);//RISING_EDGE);
  m_spi.set_phase(irs::spi_t::LEAD_EDGE);
  m_spi.lock();
  m_pins.cs.clear();
}

void u309m::plis_t::command_prepare_check_ready()
{
  mp_write_buf[0] = 0;
  mp_write_buf[1] = (1 << plis_read_only_bit);
}

bool u309m::plis_t::command_check_ready()
{
  return !static_cast<bool>(mp_read_buf[1] & (1 << plis_busy_bit));
}

void u309m::plis_t::clear_buffers()
{
  memset(mp_write_buf, 0, m_size);
  memset(mp_read_buf, 0, m_size);
}

//------------------------------- Коммутатор -----------------------------------

u309m::comm_t::comm_t(plis_t& a_plis, irs::bit_data_t* ap_apply_bit,
  irs::bit_data_t* ap_reset_bit, irs::bit_data_t* ap_alarm_bit):
  m_plis(a_plis),
  m_status(OFF),
  m_byte_vector(),
  m_bit_vector(),
  m_flag_vector(),
  mp_apply_bit(ap_apply_bit),
  mp_reset_bit(ap_reset_bit),
  mp_alarm_bit(ap_alarm_bit),
  m_on_bit_number(0),
  m_need_on(false),
  m_need_off(false),
  m_plis_has_been_reset(false)
{
  m_byte_vector.clear();
  m_bit_vector.clear();
  m_flag_vector.clear();
  m_plis.add_reset_flag(&m_plis_has_been_reset);
}

u309m::comm_t::~comm_t()
{
}

void u309m::comm_t::add_byte(irs::conn_data_t<irs_u8>* ap_byte,
  irs_u8 a_position)
{
  byte_track_t byte_track;
  byte_track.user_byte = ap_byte;
  byte_track.position = a_position;
  m_byte_vector.push_back(byte_track);
  *ap_byte = 0;
}

void u309m::comm_t::add_bit(irs::bit_data_t* ap_bit, irs_u8 a_position,
  bool a_on_bit)
{
  bit_track_t bit_track;
  bit_track.user_bit = ap_bit;
  bit_track.position = a_position;
  if (a_on_bit) m_on_bit_number = m_bit_vector.size();
  m_bit_vector.push_back(bit_track);
  *ap_bit = 0;
}

void u309m::comm_t::add_flag(irs::bit_data_t* ap_bit, irs_u8 a_position)
{
  bit_track_t bit_track;
  bit_track.user_bit = ap_bit;
  bit_track.position = a_position;
  m_flag_vector.push_back(bit_track);
  *ap_bit = 0;
}

void u309m::comm_t::on()
{
  m_need_off = false;
  if (m_status != ON) m_need_on = true;
}

void u309m::comm_t::off()
{
  m_need_on = false;
  if (m_status != OFF) m_need_off = true;
}

bool u309m::comm_t::operated()
{
  return (m_status == ON);
}

void u309m::comm_t::tick()
{
  if (*mp_reset_bit == 1) {
    m_plis.reset();
  } else if (m_plis_has_been_reset && (mp_alarm_bit != IRS_NULL)) {
    *mp_alarm_bit = 1;
  }
  if (m_plis_has_been_reset) {
    if (m_status == ON || m_status == TRANSACTION) {
      m_status = ON;
    }
    clear_user_data();
    m_plis_has_been_reset = false;
  }
  switch (m_status) {
    case ON: {
      if (m_plis.ready()) {
        if (m_need_off) {
          m_need_off = false;
          *mp_apply_bit = 0;
          *m_bit_vector[m_on_bit_number].user_bit = 0;
          prepare_and_write_data();
          m_status = OFF;
        } else if (*mp_apply_bit == 1) {
          prepare_and_write_data();
          m_status = TRANSACTION;
        }
      }
      break;
    }
    case TRANSACTION: {
      if (m_plis.ready()) {
        plis_data_t data = m_plis.read();
        *m_bit_vector[m_on_bit_number].user_bit
          = data & (1 << m_bit_vector[m_on_bit_number].position);
        for (irs_u8 i = 0; i < m_flag_vector.size(); i++) {
          *m_flag_vector[i].user_bit = data & (1 << m_flag_vector[i].position);
        }
        *mp_apply_bit = 0;
        m_status = ON;
      }
      break;
    }
    case OFF: {
      if (m_plis.ready()) {
        if (m_need_on) {
          m_need_on = false;
          m_status = ON;
        } else {
          if (*mp_apply_bit == 1) {
            *mp_apply_bit = 0;
            *m_bit_vector[m_on_bit_number].user_bit = 0;
          }
        }
      }
      break;
    }
  }
}

void u309m::comm_t::prepare_and_write_data()
{
  plis_data_t data = 0;
  for (irs_u8 i = 0; i < m_byte_vector.size(); i++) {
    data |= (*m_byte_vector[i].user_byte << m_byte_vector[i].position);
  }
  for (irs_u8 i = 0; i < m_bit_vector.size(); i++) {
    data |= (*m_bit_vector[i].user_bit << m_bit_vector[i].position);
  }
  m_plis.write(data);
}

void u309m::comm_t::clear_user_data()
{
  *mp_reset_bit = 0;
  *mp_apply_bit = 0;
  for (irs_u8 i = 0; i < m_byte_vector.size(); i++) {
    *m_byte_vector[i].user_byte = 0;
  }
  for (irs_u8 i = 0; i < m_bit_vector.size(); i++) {
    *m_bit_vector[i].user_bit = 0;
  }
  for (irs_u8 i = 0; i < m_flag_vector.size(); i++) {
    *m_flag_vector[i].user_bit = 0;
  }
}

//------------------------------------------------------------------------------
#ifdef OLD_MEAS_COMM

u309m::meas_plis_t::meas_plis_t (
  irs_u32 a_tact_freq,
  irs::arm::arm_spi_t* ap_spi,
  irs::gpio_pin_t* ap_cs_pin
):
  m_tact_freq(a_tact_freq),
  mp_spi(ap_spi),
  mp_cs_pin(ap_cs_pin),
  m_status(PLIS_SPI_FREE),
  m_need_write(false),
  m_tact_gen(PE1, m_tact_freq)
{
  memset((void*)mp_buf, 0, m_size);

  // Timer Initialization
  /*RCGC1_bit.TIMER1 = 1;
  RCGC2_bit.PORTE = 1;
  for (irs_u8 i = 10; i; i--);

  GPIOEDEN_bit.no1 = 1;
  GPIOEAFSEL_bit.no1 = 1;
  GPIOEPCTL_bit.PMC1 = CCP2;

  GPTM1CTL_bit.TBEN = 0;
*/
//  GPTM1CFG_bit.GPTMCFG = TIMER_16_BIT;
//  GPTM1TAMR_bit.TAAMS = 1;
//  GPTM1TAMR_bit.TACMR = 0;
//  GPTM1TAMR_bit.TAMR = PERIODIC_MODE;
//  GPTM1CTL_bit.TAPWML = 0;
//  GPTM1TAILR = static_cast<irs_u16>(irs::cpu_traits_t::frequency()/a_tact_freq);
//  GPTM1TAMATCHR = GPTM1TAILR/2;

  //GPTM1ICR_bit.TAMCINT = 1;
  //GPTM1ICR_bit.TATOCINT = 1;

  for (; mp_spi->get_status() != irs::spi_t::FREE; )
    mp_spi->tick();
  mp_spi->set_order(irs::spi_t::MSB);
  mp_spi->set_polarity(irs::spi_t::NEGATIVE_POLARITY);//RISING_EDGE);
  mp_spi->set_phase(irs::spi_t::LEAD_EDGE);

  mp_cs_pin->set();
}

u309m::meas_plis_t::~meas_plis_t()
{

}

void u309m::meas_plis_t::write(const irs_u8* ap_command)
{
  mp_buf[0] = ap_command[1];
  mp_buf[1] = ap_command[0];
  m_need_write = true;
}

void u309m::meas_plis_t::tact_on()
{
  //GPTM1CTL_bit.TAEN = 1;
  m_tact_gen.start();
}

void u309m::meas_plis_t::tact_off()
{
  //GPTM1CTL_bit.TAEN = 0;
  m_tact_gen.stop();
}

void u309m::meas_plis_t::tick()
{
  switch (m_status) {
    case PLIS_SPI_FREE:
    {
      if (m_need_write && (mp_spi->get_status() == irs::spi_t::FREE)) {
        if (!mp_spi->get_lock()) {
          mp_spi->set_order(irs::spi_t::MSB);
          mp_spi->set_polarity(irs::spi_t::NEGATIVE_POLARITY);//RISING_EDGE);
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

u309m::meas_comm_t::meas_comm_t(
  irs::arm::arm_spi_t* ap_spi_meas_comm_plis,
  meas_comm_pins_t* ap_meas_comm_pins,
  meas_comm_data_t* ap_meas_comm_data
):
  //mp_adc(ap_adc),
  mp_meas_comm_pins(ap_meas_comm_pins),
  mp_meas_comm_data(ap_meas_comm_data),
  m_plis(plis_tact_freq, ap_spi_meas_comm_plis,
    mp_meas_comm_pins->cs),
  m_command_apply(false),
  m_comm_on(false),
  m_plis_reset(false),
  m_command(0),
  //m_mode(command_check)
  m_mode(meas_reset_start),
  m_operate(false),
  m_need_on(false),
  m_need_off(false)
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
  m_plis.tick();

  mp_meas_comm_data->error =
    mp_meas_comm_pins->error->pin();

  /*bool plis_reset_change =
    (m_plis_reset != mp_meas_comm_data->reset);
  if (plis_reset_change) {
    m_plis_reset = mp_meas_comm_data->reset;
    if (m_plis_reset) {
      m_mode = meas_reset_start;
    }
  }*/
  bool reset_mode = (m_mode == meas_reset_start ||
                     m_mode == meas_reset_process ||
                     m_mode == meas_reset_complete);
  if (mp_meas_comm_data->reset && !reset_mode)
  {
    m_mode = meas_reset_start;
  }

  switch (m_mode)
  {
    case command_check:
    {
      if (m_need_on)
      {
        m_operate = true;
        m_need_on = false;
      }
      if (m_need_off)
      {
        m_operate = false;
        m_need_off = false;
        m_comm_on = false;
        mp_meas_comm_data->on = 0;
        m_mode = meas_on_start;
      }
      else
      {
        if (m_comm_on != static_cast<bool>(mp_meas_comm_data->on))
        {
          if (m_operate)
          {
            m_comm_on = mp_meas_comm_data->on;
            if (!m_comm_on)
            {
              m_mode = meas_on_start;
              break;
            }
          }
          else mp_meas_comm_data->on = 0;
        }
        if (m_comm_on & m_operate)
        {
          if (m_command_apply != static_cast<bool>(mp_meas_comm_data->apply))
          {
            m_command_apply = mp_meas_comm_data->apply;
            if (m_command_apply)
            {
              m_mode = meas_apply_start;
            }
          }
        }
        else
        {
          if (mp_meas_comm_data->apply)
          {
            mp_meas_comm_data->apply = 0;
          }
        }
      }
      break;
    }
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
      init_default();
      mp_meas_comm_pins->reset->set();
      m_plis.tact_on();
      m_mode = meas_reset_process;
    } break;
    case meas_reset_process:
    {
      if (mp_meas_comm_pins->apply->pin()) {
        m_mode = meas_reset_complete;
      }
    } break;
    case meas_reset_complete:
    {
      if (!mp_meas_comm_pins->apply->pin())
      {
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

  mp_meas_comm_data->apply =
    mp_meas_comm_pins->apply->pin();
}

#endif  //  OLD_MEAS_COMM
