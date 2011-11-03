#include <irsdefs.h>

#include <irscpu.h>
#include "comm.h"

#include <irsfinal.h>

u309m::plis_t::plis_t(plis_pins_t& a_pins, irs::pwm_gen_t& a_gen,
  irs::spi_t& a_spi):
  m_pins(a_pins),
  m_gen(a_gen),
  m_spi(a_spi),
  m_status(RESET),
  m_data(0),
  m_attempts_counter(0),
  m_ready(false),
  m_need_transaction(false),
  m_timer(irs::make_cnt_ms(m_cfg_done_wait_interval))
{
  memset(mp_write_buf, 0, m_size);
  memset(mp_read_buf, 0, m_size);
  m_gen.start();
  m_timer.start();
  while (!m_pins.cfg_done.pin()) {
    if (m_timer.check()) {
      IRS_LIB_ERROR(irs::ec_standard, "Сбой инициализации плис (cfg_done)");
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

bool u309m::plis_t::ready()
{
  return m_ready;
}

void u309m::plis_t::write(plis_data_t a_data)
{
  m_data = a_data;
  m_need_transaction = true;
  m_ready = false;
}

u309m::plis_data_t u309m::plis_t::read()
{
  return m_data;
}

void u309m::plis_t::tick()
{
  m_spi.tick();
  switch (m_status)
  {
    case RESET: {
      if (m_timer.check()) {
        m_pins.reset.clear();
        m_timer.set(irs::make_cnt_ms(m_reset_interval));
        m_timer.start();
        m_status = WAIT;
      }
      break;
    }
    case WAIT: {
      if (m_timer.check() && m_spi.get_status() == irs::spi_t::FREE) {
        if (!m_spi.get_lock()) {
          spi_prepare();
          m_pins.cs.clear();
          command_prepare_check_ready(mp_write_buf);
          m_spi.read_write(mp_read_buf, mp_write_buf, m_size);
          m_status = CHECK_READY;
        }
      }
      break;
    }
    case CHECK_READY: {
      if (m_spi.get_status() == irs::spi_t::FREE) {
        m_pins.cs.set();
        m_spi.unlock();
        if (command_check_ready(mp_read_buf)) {
          m_gen.stop();
          m_ready = true;
          m_status = FREE;
        } else if (m_attempts_counter < m_max_attempts_count) {
          m_attempts_counter++;
          m_timer.set(irs::make_cnt_ms(m_reset_interval));
          m_timer.start();
          m_status = WAIT;
        } else {
          m_status = ERROR;
          IRS_LIB_ERROR(irs::ec_standard, "ПЛИС не отвечает");
        }
      }
      break;
    }
    case ERROR: {
      break;
    }
    case FREE: {
      if (m_need_transaction) {
        if (!m_spi.get_lock() && m_spi.get_status() == irs::spi_t::FREE) {
          m_gen.start();
          m_ready = false;
          spi_prepare();
          m_pins.cs.clear();
          m_spi.read_write(mp_read_buf, mp_write_buf, m_size);
          m_status = WAIT;
        }
      }
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
}

void u309m::plis_t::command_prepare_check_ready(irs_u8* ap_buf)
{
  ap_buf[0] = 0;
  ap_buf[1] = (1 << plis_read_only_bit);
}

bool u309m::plis_t::command_check_ready(irs_u8* ap_buf)
{
  return !static_cast<bool>(ap_buf[1] & (1 << plis_busy_bit));
}

//---------------------------- meas_comm ---------------------------------------

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
  mp_spi->set_polarity(irs::spi_t::RISING_EDGE);
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
        if (m_comm_on != mp_meas_comm_data->on)
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
          if (m_command_apply != mp_meas_comm_data->apply)
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

//----------------------------- supply_comm ------------------------------------

u309m::supply_plis_t::supply_plis_t (
  irs_u32 a_tact_freq,
  irs::arm::arm_spi_t* ap_spi,
  irs::gpio_pin_t* ap_cs_pin,
  irs_u8* ap_write_buf,
  irs_u8* ap_read_buf
):
  m_tact_freq(a_tact_freq),
  mp_spi(ap_spi),
  mp_cs_pin(ap_cs_pin),
  mp_write_buf(ap_write_buf),
  mp_read_buf(ap_read_buf),
  m_ready(true),
  m_mode(FREE),
  m_target(NOTHING)
{
  // Timer Initialization
  RCGC1_bit.TIMER0 = 1;
  RCGC2_bit.PORTF = 1;
  for (irs_u8 i = 10; i; i--);

  GPIOFDEN_bit.no4 = 1;
  GPIOFAFSEL_bit.no4 = 1;
  GPIOFPCTL_bit.PMC4 = CCP0;

  GPTM0CTL_bit.TBEN = 0;

  GPTM0CFG_bit.GPTMCFG = TIMER_16_BIT;
  GPTM0TAMR_bit.TAAMS = 1;
  GPTM0TAMR_bit.TACMR = 0;
  GPTM0TAMR_bit.TAMR = PERIODIC_MODE;
  GPTM0CTL_bit.TAPWML = 0;
  GPTM0TAILR = static_cast<irs_u16>(irs::cpu_traits_t::frequency()/a_tact_freq);
  GPTM0TAMATCHR = GPTM0TAILR/2;

  GPTM0ICR_bit.TAMCINT = 1;
  GPTM0ICR_bit.TATOCINT = 1;

  mp_cs_pin->set();
}

u309m::supply_plis_t::~supply_plis_t()
{

}

void u309m::supply_plis_t::read()
{
  m_target = READ;
  m_ready = false;
}

void u309m::supply_plis_t::write()
{
  m_target = WRITE;
  m_ready = false;
}

void u309m::supply_plis_t::read_write()
{
  m_target = READ_WRITE;
  m_ready = false;
}

void u309m::supply_plis_t::tact_on()
{
  GPTM0CTL_bit.TAEN = 1;
}

void u309m::supply_plis_t::tact_off()
{
  GPTM0CTL_bit.TAEN = 0;
}

void u309m::supply_plis_t::tick()
{
  mp_spi->tick();
  switch (m_mode)
  {
    case FREE:
    {
      if ((mp_spi->get_status() == irs::spi_t::FREE) && (!mp_spi->get_lock()))
      {
        if (m_target != NOTHING)
        {
          mp_spi->set_order(irs::spi_t::MSB);
          mp_spi->set_polarity(irs::spi_t::RISING_EDGE);
          mp_spi->set_phase(irs::spi_t::LEAD_EDGE);
          mp_spi->lock();
          mp_cs_pin->clear();
          for (volatile irs_u8 i = 20; i; i--);
          //m_mode = SPI_BUSY;

          switch (m_target)
          {
            case READ: mp_spi->read(mp_read_buf, m_size); break;
            case WRITE:
            {
              irs_u8 write_buf[m_size];
              write_buf[0] = mp_write_buf[1];
              write_buf[1] = mp_write_buf[0];
              mp_spi->write(write_buf, m_size);
              break;
            }
            case READ_WRITE:
            {
              irs_u8 write_buf[m_size];
              write_buf[0] = mp_write_buf[1];
              write_buf[1] = mp_write_buf[0];
              mp_spi->read_write(mp_read_buf, write_buf, m_size);
              break;
            }
          }

          while (mp_spi->get_status() != irs::spi_t::FREE) mp_spi->tick();
          {
            for (volatile irs_u8 i = 20; i; i--);
            mp_cs_pin->set();
            mp_spi->unlock();

            if (m_target == READ || m_target == READ_WRITE)
            {
              irs_u8 byte = mp_read_buf[0];
              mp_read_buf[0] = mp_read_buf[1];
              mp_read_buf[1] = byte;
            }

            m_target = NOTHING;
            m_ready = true;
            m_mode = FREE;
          }
        }
      }
      break;
    }
    case SPI_BUSY:
    {
      if (mp_spi->get_status() == irs::spi_t::FREE)
      {
        for (volatile irs_u8 i = 20; i; i--);
        mp_cs_pin->set();
        mp_spi->unlock();

        if (m_target == READ || m_target == READ_WRITE)
        {
          irs_u8 byte = mp_read_buf[0];
          mp_read_buf[0] = mp_read_buf[1];
          mp_read_buf[1] = byte;
        }

        m_target = NOTHING;
        m_ready = true;
        m_mode = FREE;
      }
    } break;
  }
}

u309m::supply_comm_t::supply_comm_t(
  irs::arm::arm_spi_t* ap_spi,
  supply_comm_pins_t* ap_supply_comm_pins,
  supply_comm_data_t* ap_supply_comm_data
):
  m_reset_interval(irs::make_cnt_ms(1)),
  m_transaction_interval(irs::make_cnt(2 * plis_relay_delay, plis_tact_freq)),
  mp_supply_comm_pins(ap_supply_comm_pins),
  mp_supply_comm_data(ap_supply_comm_data),
  m_plis(plis_tact_freq, ap_spi, mp_supply_comm_pins->cs,
    reinterpret_cast<irs_u8*>(&m_command),
    reinterpret_cast<irs_u8*>(&m_answer)),
  m_command_apply(false),
  m_error(false),
  m_busy(false),
  m_ready(false),
  m_answer(0),
  m_command(0),
  m_mode(mode_reset_start),
  m_plis_reset(false),
  m_timer(m_reset_interval),
  m_transaction_cnt(0),
  m_operate(false),
  m_need_on(false),
  m_need_off(false)
{
  while(m_mode != mode_command_check) tick();
}

void u309m::supply_comm_t::make_command()
{
  m_command = 0;
  m_command = (
    (mp_supply_comm_data->on << 2) |
    (mp_supply_comm_data->polarity_calibrated << 3) |
    (mp_supply_comm_data->polarity_etalon << 4) |
    (mp_supply_comm_data->calibrated_cell << 5) |
    (mp_supply_comm_data->etalon_cell << 9) |
    (mp_supply_comm_data->supply_index << 13)
  );
}

void u309m::supply_comm_t::reset_command()
{
  m_command = 0;
  mp_supply_comm_data->on = 0;
  mp_supply_comm_data->polarity_calibrated = 0;
  mp_supply_comm_data->polarity_etalon = 0;
  mp_supply_comm_data->calibrated_cell = 0;
  mp_supply_comm_data->etalon_cell = 0;
  mp_supply_comm_data->supply_index = 0;
  mp_supply_comm_data->apply = 1;
}

void u309m::supply_comm_t::init_default()
{
  mp_supply_comm_data->supply_index = 0;
  mp_supply_comm_data->calibrated_cell = 0;
  mp_supply_comm_data->etalon_cell = 0;
  mp_supply_comm_data->polarity_etalon = 0;
  mp_supply_comm_data->polarity_calibrated = 0;
  mp_supply_comm_data->apply = 0;
  mp_supply_comm_data->on = 0;
  mp_supply_comm_data->error = 0;
  mp_supply_comm_data->reset = 0;

  m_error = false;
  m_busy = false;
  m_ready = false;

  m_transaction_cnt = 0;
}

void u309m::supply_comm_t::tick()
{
  m_plis.tick();
  mp_supply_comm_data->error = m_error;

  bool plis_reset_change =
    (m_plis_reset != mp_supply_comm_data->reset);
  if (plis_reset_change) {
    m_plis_reset = mp_supply_comm_data->reset;
    if (m_plis_reset) {
      m_mode = mode_reset_start;
    }
  }

  switch (m_mode) {
    case mode_reset_start:
    {
      init_default();
      m_timer.set(m_reset_interval);
      m_timer.start();
      m_plis.tact_on();
      mp_supply_comm_pins->reset->set();
      mp_supply_comm_data->reset = 1;
      m_mode = mode_reset_clear;
      break;
    }
    case mode_reset_clear:
    {
      if (m_timer.check() && m_plis.ready())
      {
        mp_supply_comm_pins->reset->clear();
        m_timer.set(m_transaction_interval);
        m_timer.start();
        m_mode = mode_reset;
//        m_command = (PLIS|SUPPLY_17A|SUPPLY_1A|SUPPLY_2V|SUPPLY_20V|
//          SUPPLY_200V|IZM_TH|EEPROM|MISO_MASK_EN);
        m_command = (PLIS|IZM_TH|MISO_MASK_EN);
        m_plis.write();
      }
      break;
    }
    case mode_reset:
    {
      if (m_plis.ready() && m_timer.check())
      {
        m_plis.tact_off();
        m_plis_reset = false;
        mp_supply_comm_data->reset = m_plis_reset;
        m_mode = mode_command_check;
      }
      break;
    }
    case mode_command_check:
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
        m_command_apply = false;
        reset_command();
        m_mode = mode_send_command;
      }
      else
      {
        if (m_command_apply != mp_supply_comm_data->apply)
        {
          if (m_operate)
          {
            m_command_apply = mp_supply_comm_data->apply;
            if (m_command_apply)
            {
              make_command();
              m_mode = mode_send_command;
            }
          }
          else mp_supply_comm_data->apply = 0;
        }
      }
      break;
    }
    case mode_send_command:
    {
      if (m_plis.ready())
      {
        m_plis.tact_on();
        m_plis.write();
        m_mode = mode_send_request;
      }
      break;
    }
    case mode_send_request:
    {
      if (m_plis.ready())
      {
        m_command = (1 << READ_ONLY_POS);
        m_plis.read_write();
        m_mode = mode_recieve_request;
      }
      break;
    }
    case mode_recieve_request:
    {
      if (m_plis.ready())
      {
        m_error = m_answer & (1 << ERROR_POS);
        m_busy = m_answer & (1 << BUSY_POS);
        m_ready = m_answer & (1 << READY_POS);
        if (m_busy)
        {
          if (m_transaction_cnt <= transaction_cnt_limit)
          {
            m_timer.set(m_transaction_interval);
            m_timer.start();
            m_transaction_cnt++;
            m_mode = mode_wait;
          }
          else
          {
            m_mode = mode_reset_start;
          }
        }
        else
        {
          m_plis.tact_off();
          m_transaction_cnt = 0;
          m_command_apply = 0;
          mp_supply_comm_data->apply = 0;
          m_mode = mode_command_check;
        }
      }
      break;
    }
    case mode_wait:
    {
      if (m_timer.check())
      {
        m_mode = mode_send_request;
      }
      break;
    }
  }
}
