#include <irsdefs.h>

#include <irsstrm.h>
#include <irsmcutil.h>
#include <irsinit.h>

#include "app.h"

#include <irsfinal.h>

// Команды для переменной unlock (смещение 652):
// m_unlock_command = 116,
// m_clear_alarm_command = 207,

void app_start(u309m::cfg_t* ap_cfg);


int main()
{
  pll_on();
  irs::init();

  static hard_fault_event_t hard_fault_event(GPIO_PORTJ, 5);

  static irs::arm::com_buf log_buf(1, 10, 1000000);
  irs::mlog().rdbuf(&log_buf);
  irs::mlog() << endl;
  irs::mlog() << endl;
  irs::mlog() << irsm("--------- INITIALIZATION --------") << endl;

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
