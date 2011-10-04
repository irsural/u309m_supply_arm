#include <irsdefs.h>

#include <armadc.h>
#include <irsadc.h>

#include "app.h"
#include "demux.h"

#include <irsstrm.h>

#include <irsfinal.h>

void app_start(u309m::cfg_t* ap_cfg);

int main()
{
  pll_on();

  /*SHCSR_bit.MEMFAULTENA = 1;
  SHCSR_bit.BUSFAULTENA = 1;
  SHCSR_bit.USGFAULTENA = 1;
  CCR_bit.DIV_0_TRP = 1;*/
  //CCR_bit.UNALIGN_TRP = 1;

  static hard_fault_event_t hard_fault_event(GPIO_PORTJ, 5);
  
  static irs::arm::com_buf log_buf(1, 10, 1000000);
  irs::mlog().rdbuf(&log_buf);
  //irs::mlog().rdbuf(cout.rdbuf());
  //static irs::mc_error_handler_t error_handler(GPIO_PORTC, 7, &irs::mlog());
    
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
    //A3_blink(); // Мигание светодиодом на плате измерительного коммутатора
    //irs::mlog() << "divide by zero: " << result_test << endl;
  }
}
