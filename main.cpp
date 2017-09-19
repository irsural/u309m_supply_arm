// 24.11.2016 18:05:01 Крашенинников:
enum { program_rev = 110, mxsrclib_rev = 1321, common_rev = 13, 
  lwip_rev = 21 };
// 24.11.2016 18:05:01 Крашенинников:
//  Убран вывод в лог параметров изодромного звена терморегулятора
//enum { program_rev = 109, mxsrclib_rev = 1321, common_rev = 13, 
//  lwip_rev = 21 };
// 17.11.2016 14:07:20 Крашенинников:
//  Добавлен LWIP. Маска, шлюз и ip-адрес теперь применяются по
//    биту ip_params_apply
//enum { program_rev = 108, mxsrclib_rev = 1320, common_rev = 13, 
//  lwip_rev = 21 };
// 17.11.2016 10:59:37 Крашенинников:
// Сделал IP по умолчанию 192.168.1.6 (был 192.168.0.211)
//enum { program_rev = 107, mxsrclib_rev = 1319, common_rev = 12 };
// 17.11.2016 Крашенинников:
// Программа скомпилированна в IAR EW ARM 6.30.1.3142, т. к. на IAR 7.50
//   не работает программатор J-Link 8 (пишет что не оригинальный)
// Начато добавление lwip, но пока отключено
// Все что написано в коментарии к ревизии 103 для функции arm_spi_t::write
//   внесено в библиотеку только в mxsrclib_rev = 1318
// Была проведена минимальная проверка на МК.
//enum { program_rev = 106, mxsrclib_rev = 1319, common_rev = 12 };
// 28.09.2016 Крашенинников:
//   Программа скомпилированна в IAR EW ARM 7.50.2.10505
//   В arm_spi_t::write (define __LM3SxBxx__) был вывод в на консоль COM-порта
//     информации о том, что бит on (m_eth_data.meas_comm.on) сброшен в 0.
//     Для этого события выводилось время. Когда к ИПТ подключена программа
//     У309М, то она задает текущее время для счетчика библиотеки mxsrclib.
//     При этом время, в этом событии, выводится с помощью этого счетчика.
//     Если программа У309М не подключена к ИПТ (через Ethernet), то
//     время берется с помощью функции time из стандартной библиотеки.
//     Т. к. к этой функции не был реализован интерфейс для чтения
//     времени из внешних источников, то при вызове этой функции происходит
//     зависание МК. При включенном сторожевом таймере МК перезагружается.
//     Т. е. при попытке выключить коммутатор происходит перезагрузка МК.
//     Причем при подключенном отладчике IAR зависания не происходит, т. к.
//     он подключает свою реализацию чтения времени. Он берет время с ПК.
//     Это ошибка была исправлена путем удаления кода для вывода информации
//     о выключении бита on.
//enum { program_rev = 103, mxsrclib_rev = 1261, common_rev = 12 };
// 13.08.2016 16:32:10 Крашенинников:
//    Исправлена ошибка в перменной MODBUS control.spi_enable
//    В MODBUS добавлены версии программы
//enum { program_rev = 100, mxsrclib_rev = 1261, common_rev = 12 };
// 05.08.2016 14:10:20 rev. 99 mxsrclib rev. 1261 u309m_common rev. 10
//    Процессор изменен на TexasInstruments LM3S9B95 (раньше был LM3S9B96.
//    Это не правильно, но работало)
// 16.02.2015 12:06 rev. 97 mxsrclib rev. 1261 u309m_common rev. 10

#include <irsdefs.h>

#include <irsstrm.h>
#include <irsmcutil.h>
#include <irsinit.h>

#include "app.h"
#include "config.h"

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
  #ifdef U309M_LWIP
  main_info.lwip_rev = lwip_rev;
  #else //U309M_LWIP
  main_info.lwip_rev = 0;
  #endif //U309M_LWIP
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
  #ifdef U309M_LWIP
  irs::mlog() << irsm("lwip_rev = ") << lwip_rev << endl;
  #endif //U309M_LWIP
  while(true) {
    app.tick();
    static irs::blink_t F0_blink(GPIO_PORTF, 0, irs::make_cnt_ms(100));
    F0_blink(); // Мигание светодиодом на плате arm
  }
}
