#include <irsdefs.h>

#include <irsstrm.h>
#include "app.h"

#include <irsfinal.h>

void app_start(u309m::cfg_t* ap_cfg);

class interrupt_event_t: public mxfact_event_t
{
public:
  interrupt_event_t()
  {
    irs::arm::interrupt_array()->int_event_gen(irs::arm::gpio_portj_int)->
      add(this);
  }
  
  virtual void exec()
  {
    mxfact_event_t::exec();
    irs::mlog() << irsm("test") << endl;
  }

};
int main()
{
  pll_on();

  static hard_fault_event_t hard_fault_event(GPIO_PORTJ, 5);

  static irs::arm::com_buf log_buf(1, 10, 1000000);
  irs::mlog().rdbuf(&log_buf);
  irs::mlog() << endl;
  irs::mlog() << endl;
  irs::mlog() << irsm("--------- INITIALIZATION --------") << endl;
  
  interrupt_event_t interrupt_event_gpio();
  
  static u309m::cfg_t cfg;
  app_start(&cfg);
}

void app_start(u309m::cfg_t* ap_cfg)
{
  static u309m::app_t app(ap_cfg);

  irs::mlog() << irsm("------------- START -------------") << endl;

  while(true) {
    app.tick();
    static irs::blink_t F0_blink(GPIO_PORTF, 0, irs::make_cnt_ms(100));
    F0_blink(); // Мигание светодиодом на плате arm
  }
}
