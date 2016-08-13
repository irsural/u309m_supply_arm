// 13.08.2016 16:32:10
//    Исправлена ошибка в перменной MODBUS control.spi_enable
//    В MODBUS добавлены версии программы
enum { program_rev = 100, mxsrclib_rev = 1261, common_rev = 12 };
// 05.08.2016 14:10:20 rev. 99 mxsrclib rev. 1261 u309m_common rev. 10
//    Процессор изменен на TexasInstruments LM3S9B95 (раньше был LM3S9B96.
//    Это не правильно, но работало)
// 16.02.2015 12:06 rev. 97 mxsrclib rev. 1261 u309m_common rev. 10

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
  //irs::mlog().rdbuf(cout.rdbuf());

  irs::mlog() << endl;
  irs::mlog() << endl;
  irs::mlog() << irsm("--------- INITIALIZATION --------") << endl;

  static u309m::main_info_t main_info;
  main_info.program_rev = program_rev;
  main_info.mxsrclib_rev = mxsrclib_rev;
  main_info.common_rev = common_rev;
  static u309m::cfg_t cfg;
  cfg.main_info(&main_info);
  app_start(&cfg);
}

void app_start(u309m::cfg_t* ap_cfg)
{
  static u309m::app_t app(ap_cfg);

  irs::mlog() << irsm("------------- START -------------") << endl;
  irs::mlog() << irsm("program_rev = ") << program_rev << endl;
  irs::mlog() << irsm("mxsrclib_rev = ") << mxsrclib_rev << endl;
  irs::mlog() << irsm("common_rev = ") << common_rev << endl;
  while(true) {
    app.tick();
    static irs::blink_t F0_blink(GPIO_PORTF, 0, irs::make_cnt_ms(100));
    F0_blink(); // Мигание светодиодом на плате arm
  }
}
