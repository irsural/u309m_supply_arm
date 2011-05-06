#include <irsdefs.h>

#include <armadc.h>
#include <irsadc.h>

#include "app.h"
#include "demux.h"

#include <irsstrm.h>
#include <mxdata.h>

#include <irsfinal.h>

int main()
{
  pll_on();

  /*SHCSR_bit.MEMFAULTENA = 1;
  SHCSR_bit.BUSFAULTENA = 1;
  SHCSR_bit.USGFAULTENA = 1;
  CCR_bit.DIV_0_TRP = 1;*/
  //CCR_bit.UNALIGN_TRP = 1;

  static hard_fault_event_t hard_fault_event(GPIO_PORTC, 5);
  
  static irs::arm::com_buf log_buf(1, 10, 1000000);
  irs::mlog().rdbuf(&log_buf);
  static irs::mc_error_handler_t error_handler(GPIO_PORTC, 7, &irs::mlog());
    
  static irs::blink_t F0_blink(GPIO_PORTF, 0, irs::make_cnt_ms(100));

  static u309m::cfg_t cfg;
  static u309m::app_t app(&cfg);
  
  irs::mlog() << irsm("------------- START -------------") << endl;
  
  while(true) {
    app.tick();
    F0_blink(); // Мигание светодиодом на плате arm
    //A3_blink(); // Мигание светодиодом на плате измерительного коммутатора
    //irs::mlog() << "divide by zero: " << result_test << endl;
  }
}
