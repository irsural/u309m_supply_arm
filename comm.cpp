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
  m_timer.start();
  while (!m_pins.cfg_done.pin()) {
    if (m_timer.check()) {
      irs::mlog() << "Сбой инициализации плис (cfg_done)" << endl;
      m_status = ERROR;
      break;
    }
  }
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
  m_spi.set_polarity(irs::spi_t::RISING_EDGE);
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
