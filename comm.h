#ifndef commh
#define commh

#include <irsdefs.h>

#include <irsspi.h>
#include <irsdev.h>
#include <mxdata.h>
#include "cfg.h"

#ifdef OLD_MEAS_COMM
#include "data.h"
#endif  //  OLD_MEAS_COMM

#include <irsfinal.h>

namespace u309m {

//---------------------------------- ПЛИС --------------------------------------

typedef irs_u16 plis_data_t;
typedef struct {
  irs_u8 lsb  : 8;
  irs_u8 msb  : 8;
} plis_bytes_t;
typedef union {
  plis_data_t data;
  plis_bytes_t bytes;
} plis_data_bytes_t;

enum plis_bit_t {
  //  Входящие
  plis_miso_mask_bit = 0,
  plis_read_only_bit = 1,
  plis_on_bit = 2,
  //  Исходящие
  plis_error_bit = 0,
  plis_busy_bit = 1,
  plis_ready_bit = 2
};

class plis_t {
public:
  plis_t(plis_pins_t& a_pins, irs::pwm_gen_t& a_gen, irs::spi_t& a_spi);
  ~plis_t();
  void reset();
  void add_reset_flag(bool* ap_flag);
  bool ready();
  void write(plis_data_t a_data);
  plis_data_t read();
  void tick();
private:
  enum {
    m_reset_pulse_interval = 10,
    m_transaction_interval = 500,
    m_ack_interval = 1,
    m_cfg_done_wait_interval = 3000,
    m_size = sizeof(plis_data_t),
    m_max_attempts_count = 20
  };
  enum status_t {
    RESET,
    WRITE,
    WAIT,
    ACK,
    CHECK_READY,
    FREE,
    ERROR
  };
  plis_pins_t& m_pins;
  irs::pwm_gen_t& m_gen;
  irs::spi_t& m_spi;
  status_t m_status;
  irs_u8 mp_write_buf[m_size];
  irs_u8 mp_read_buf[m_size];
  irs_u8 m_attempts_counter;
  vector<bool*> m_reset_vector;
  bool m_ready;
  bool m_need_write;
  irs::timer_t m_timer;
  //
  void spi_prepare();
  void command_prepare_check_ready();
  bool command_check_ready();
  void clear_buffers();
};

//------------------------------- Коммутатор -----------------------------------

class comm_t {
public:
  comm_t(plis_t& a_plis, irs::bit_data_t* ap_apply_bit,
    irs::bit_data_t* ap_reset_bit, irs::bit_data_t* ap_alarm_bit = IRS_NULL);
  ~comm_t();
  void add_byte(irs::conn_data_t<irs_u8>* ap_byte, irs_u8 a_position);
  void add_bit(irs::bit_data_t* ap_bit, irs_u8 a_position,
    bool a_on_bit = false);
  void add_flag(irs::bit_data_t* ap_bit, irs_u8 a_position);
  void on();
  void off();
  bool operated();
  void tick();
private:
  enum status_t {
    ON,
    TRANSACTION,
    OFF
  };
  typedef struct {
    irs::conn_data_t<irs_u8>* user_byte;
    irs_u8 position;
  } byte_track_t;
  typedef struct {
    irs::bit_data_t* user_bit;
    irs_u8 position;
  } bit_track_t;
  plis_t& m_plis;
  status_t m_status;
  vector<byte_track_t> m_byte_vector;
  vector<bit_track_t> m_bit_vector;
  vector<bit_track_t> m_flag_vector;
  irs::bit_data_t* mp_apply_bit;
  irs::bit_data_t* mp_reset_bit;
  irs::bit_data_t* mp_alarm_bit;
  irs_u8 m_on_bit_number;
  bool m_need_on;
  bool m_need_off;
  bool m_plis_has_been_reset;
  //
  void prepare_and_write_data();
  void clear_user_data();
};

//------------------------------------------------------------------------------

#ifdef OLD_MEAS_COMM

enum
{
  plis_tact_freq = 7000000,
  plis_relay_delay = 4000000//50000
};


class meas_plis_t
{
public:
  enum plis_status_t {
    BUSY = 1,
    COMPLETE = 2,
    ERROR = 3
  };

  meas_plis_t(
    irs_u32 a_tact_freq,
    irs::arm::arm_spi_t* ap_spi,
    irs::gpio_pin_t* ap_cs_pin
  );
  ~meas_plis_t();
  void write(const irs_u8 *ap_command);
  void tact_on();
  void tact_off();
  void tick();
private:
  enum {
    CCP2 = 0x4,
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
  irs::arm::gptm_generator_t m_tact_gen;
}; // meas_plis_t

class meas_comm_t
{
public:
  meas_comm_t(
    irs::arm::arm_spi_t* ap_spi_meas_comm_plis,
    meas_comm_pins_t* ap_meas_comm_pins,
    meas_comm_data_t* ap_meas_comm_data
  );
  inline void on() { m_need_on = true; };
  inline void off() { m_need_off = true; };
  inline bool operated() { return m_operate; };
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
    meas_reset_process,
    meas_reset_complete
  };

  meas_comm_pins_t* mp_meas_comm_pins;
  meas_comm_data_t* mp_meas_comm_data;
  meas_plis_t m_plis;
  bool m_command_apply;
  bool m_comm_on;
  bool m_plis_reset;
  irs_u16 m_command;
  tick_mode_t m_mode;
  bool m_operate;
  bool m_need_on;
  bool m_need_off;

  void make_command();
  void init_default();
}; // meas_comm_t

#endif  //  OLD_MEAS_COMM

} // namespace u309m

#endif // commh
