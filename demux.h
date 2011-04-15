#ifndef demuxh
#define demuxh

#include <irsdefs.h>

#include <irsgpio.h>

#include <irsfinal.h>

class demux_t
{
public:
  struct cs_data_t {
    irs::gpio_pin_t* cs_code_0;
    irs::gpio_pin_t* cs_code_1;
    irs::gpio_pin_t* cs_code_2;
    irs::gpio_pin_t* cs_code_3;
    irs::gpio_pin_t* cs_code_4;
    irs::gpio_pin_t* cs_enable;
    
    cs_data_t(
      irs::gpio_pin_t* ap_cs_code_0,
      irs::gpio_pin_t* ap_cs_code_1,
      irs::gpio_pin_t* ap_cs_code_2,
      irs::gpio_pin_t* ap_cs_code_3,
      irs::gpio_pin_t* ap_cs_code_4,
      irs::gpio_pin_t* ap_cs_enable
    ):
      cs_code_0(ap_cs_code_0),
      cs_code_1(ap_cs_code_1),
      cs_code_2(ap_cs_code_2),
      cs_code_3(ap_cs_code_3),
      cs_code_4(ap_cs_code_4),
      cs_enable(ap_cs_enable)
    {
    }
  };
  
  demux_t(cs_data_t* ap_cs_data):
    m_demux_cs_vec()
  {
    for (int i = 0; i < number_of_spi_devices; i++) {
      demux_cs_t demux_cs(ap_cs_data, i);
      m_demux_cs_vec.push_back(demux_cs);
    }
  }
  
  irs::gpio_pin_t* cs_code(irs_u8 a_device_cs_num)
  {
    return& m_demux_cs_vec[a_device_cs_num];
  }
private:
  class demux_cs_t: public irs::gpio_pin_t
  {
  public:
    demux_cs_t(cs_data_t* ap_cs_data_t, irs_u8 a_index):
      mp_cs_data(ap_cs_data_t),
      m_index(a_index)
    {
    }
    virtual bool pin()
    {
      return false;
    }
    virtual void set()
    {
      if (m_index <= first_demux_end) {
        mp_cs_data->cs_code_4->clear();
        set_or_clear_pin(mask_gen_demux_1(0), mp_cs_data->cs_code_0);
        set_or_clear_pin(mask_gen_demux_1(1), mp_cs_data->cs_code_1);
        set_or_clear_pin(mask_gen_demux_1(2), mp_cs_data->cs_code_2);
        set_or_clear_pin(mask_gen_demux_1(3), mp_cs_data->cs_code_3);
      } else if ((m_index >= second_demux_begin) &&
        (m_index <= second_demux_end))
      {
        mp_cs_data->cs_code_4->set();
        set_or_clear_pin(mask_gen_demux_2(0), mp_cs_data->cs_code_0);
        set_or_clear_pin(mask_gen_demux_2(1), mp_cs_data->cs_code_1);
        set_or_clear_pin(mask_gen_demux_2(2), mp_cs_data->cs_code_2);
        set_or_clear_pin(mask_gen_demux_2(3), mp_cs_data->cs_code_3);
      }
    }
    virtual void clear()
    {
      mp_cs_data->cs_enable->clear();
    }
    virtual void set_dir(gpio_pin_t::dir_t /*a_dir*/)
    {
    }
  private:
    enum {
      first_demux_end = 15,
      second_demux_begin = first_demux_end + 1,
      second_demux_end = 31
    };
  
    cs_data_t* mp_cs_data;
    irs_u8 m_index;
    
    irs_u8 mask_gen_demux_1(irs_u8 a_bit)
    {
      irs_u8 left_shift = 7 - a_bit;
      irs_u8 right_shift = a_bit;
      irs_u8 mask = m_index;
      mask <<= left_shift;
      mask >>= left_shift;
      mask >>= right_shift;
      mask <<= right_shift;
      return mask;
    }
    irs_u8 mask_gen_demux_2(irs_u8 a_bit)
    {
      irs_u8 left_shift = 7 - a_bit;
      irs_u8 right_shift = a_bit;
      irs_u8 mask = m_index - second_demux_begin;
      mask <<= left_shift;
      mask >>= left_shift;
      mask >>= right_shift;
      mask <<= right_shift;
      return mask;
    }
    void set_or_clear_pin(bool a_bit, irs::gpio_pin_t* a_pin)
    {
      if (a_bit) {
        a_pin->set();
      } else {
        a_pin->clear();
      }
    }
  };
  enum {
    number_of_spi_devices = 32
  };
  
  vector<demux_cs_t> m_demux_cs_vec;
};

#endif // demuxh
