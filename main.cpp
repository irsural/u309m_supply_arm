#include <irsdefs.h>

#include <armadc.h>
#include <irsadc.h>

#include "app.h"
#include "demux.h"

#include <irsstrm.h>
#include <mxdata.h>

#include <armflash.h>

#include <irsfinal.h>

struct flash_test_data_t {
  irs::conn_data_t<float> var_1;
  irs::conn_data_t<float> var_2;
  irs::conn_data_t<float> var_3;
  
  flash_test_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if(ap_size != IRS_NULL){
      *ap_size = size;
    }
  }
  
  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;
    
    index = var_1.connect(ap_data, index);
    index = var_2.connect(ap_data, index);
    index = var_3.connect(ap_data, index);
    
    return index;
  }
};

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

  /*static irs::arm::flash_protected_t flash(12);
  static flash_test_data_t flash_data(&flash);*/
  
  irs::mlog() << endl;
  irs::mlog() << endl;
  irs::mlog() << irsm("--------- INITIALIZATION --------") << endl;
  
  static u309m::cfg_t cfg;
  static u309m::app_t app(&cfg);
  
  irs::mlog() << irsm("------------- START -------------") << endl;
  
  /*if (!flash.double_error()) {
    irs::mlog() << irsm("before write") << endl;
    irs::mlog() << irsm("flash var_1 = ") << flash_data.var_1 << endl;
    irs::mlog() << irsm("flash var_2 = ") << flash_data.var_2 << endl;
    irs::mlog() << irsm("flash var_3 = ") << flash_data.var_3 << endl;
    
    flash_data.var_1 = 5;
    flash_data.var_2 = 3;
    flash_data.var_3 = 177;
    
    irs::mlog() << irsm("after write") << endl;
    irs::mlog() << irsm("flash var_1 = ") << flash_data.var_1 << endl;
    irs::mlog() << irsm("flash var_2 = ") << flash_data.var_2 << endl;
    irs::mlog() << irsm("flash var_3 = ") << flash_data.var_3 << endl;
  } else {
    irs::mlog() << "Achtung!!! FLASH ERROR" << endl; 
  }*/
  
  while(true) {
    app.tick();
    F0_blink(); // Мигание светодиодом на плате arm
    //A3_blink(); // Мигание светодиодом на плате измерительного коммутатора
    //irs::mlog() << "divide by zero: " << result_test << endl;
  }
}
