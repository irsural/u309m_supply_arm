#ifndef datah
#define datah

#include <irsdefs.h>

#include <mxdata.h>
#include "privatecfg.h"
#include "..\u309m_common\common_data.h"

#include <irsfinal.h>

struct eeprom_supply_data_t
{
  irs::conn_data_t<irs_u16> resistance_code;
  irs::conn_data_t<float> koef_reg_prev;
  irs::conn_data_t<float> koef_adc_volt_prev;
  irs::conn_data_t<float> koef_reg_fin;
  irs::conn_data_t<float> koef_adc_volt_fin;
  irs::conn_data_t<float> temp_base_ref;
  irs::conn_data_t<float> temp_base_k;
  irs::conn_data_t<float> temp_base_ki;
  irs::conn_data_t<float> temp_base_kd;
  irs::conn_data_t<float> temp_base_prop_koef;
  irs::conn_data_t<float> temp_base_time_const;
  irs::conn_data_t<float> temp_aux_ref;
  irs::conn_data_t<float> temp_aux_k;
  irs::conn_data_t<float> temp_aux_ki;
  irs::conn_data_t<float> temp_aux_kd;
  irs::conn_data_t<float> temp_aux_prop_koef;
  irs::conn_data_t<float> temp_aux_time_const;

  eeprom_supply_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if(ap_size != IRS_NULL){
      *ap_size = size;
    }
  }
  irs_uarc connect(irs::mxdata_t* ap_data = 0, irs_uarc a_index = 0)
  {
    irs_uarc index = a_index;

    index = resistance_code.connect(ap_data, index);
    index = koef_reg_prev.connect(ap_data, index);
    index = koef_adc_volt_prev.connect(ap_data, index);
    index = koef_reg_fin.connect(ap_data, index);
    index = koef_adc_volt_fin.connect(ap_data, index);
    index = temp_base_ref.connect(ap_data, index);
    index = temp_base_k.connect(ap_data, index);
    index = temp_base_ki.connect(ap_data, index);
    index = temp_base_kd.connect(ap_data, index);
    index = temp_base_prop_koef.connect(ap_data, index);
    index = temp_base_time_const.connect(ap_data, index);
    index = temp_aux_ref.connect(ap_data, index);
    index = temp_aux_k.connect(ap_data, index);
    index = temp_aux_ki.connect(ap_data, index);
    index = temp_aux_kd.connect(ap_data, index);
    index = temp_aux_prop_koef.connect(ap_data, index);
    index = temp_aux_time_const.connect(ap_data, index);

    return index;
  }
}; // supply_eeprom_data

struct eeprom_data_t
{
  irs::conn_data_t<irs_u8> ip_0;
  irs::conn_data_t<irs_u8> ip_1;
  irs::conn_data_t<irs_u8> ip_2;
  irs::conn_data_t<irs_u8> ip_3;
  eeprom_supply_data_t supply_200V;
  eeprom_supply_data_t supply_20V;
  eeprom_supply_data_t supply_2V;
  eeprom_supply_data_t supply_1A;
  eeprom_supply_data_t supply_17A;
  irs::conn_data_t<irs_u32> options;
  irs::bit_data_t upper_level_check;
  irs::bit_data_t izm_th_spi_enable;
  irs::bit_data_t supply_comm_debug;
  irs::bit_data_t meas_comm_debug;

  eeprom_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
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

    index = ip_0.connect(ap_data, index);
    index = ip_1.connect(ap_data, index);
    index = ip_2.connect(ap_data, index);
    index = ip_3.connect(ap_data, index);
    index = supply_200V.connect(ap_data, index);
    index = supply_20V.connect(ap_data, index);
    index = supply_2V.connect(ap_data, index);
    index = supply_1A.connect(ap_data, index);
    index = supply_17A.connect(ap_data, index);

    upper_level_check.connect(ap_data, index, 0);
    izm_th_spi_enable.connect(ap_data, index, 1);
    supply_comm_debug.connect(ap_data, index, 2);
    meas_comm_debug.connect(ap_data, index, 3);
    index = options.connect(ap_data, index);

    return index;
  }

  inline void reset_to_default()
  {
    ip_0 = IP_0;
    ip_1 = IP_1;
    ip_2 = IP_2;
    ip_3 = IP_3;

    supply_200V.resistance_code = 980;
    supply_200V.koef_adc_volt_prev = 0.259343;
    supply_200V.koef_adc_volt_fin = 0.291522;
    supply_200V.koef_reg_prev = 297.572;
    supply_200V.koef_reg_fin = 264.399;

    supply_200V.temp_base_ref = 70.;
    supply_200V.temp_base_k = 15000.;
    supply_200V.temp_base_ki = 0.005;
    supply_200V.temp_base_kd = 50.;
    supply_200V.temp_base_prop_koef = 0.;
    supply_200V.temp_base_time_const = 20.;

    supply_200V.temp_aux_ref = 70.;
    supply_200V.temp_aux_k = 15000.;
    supply_200V.temp_aux_ki = 0.005;
    supply_200V.temp_aux_kd = 50.;
    supply_200V.temp_aux_prop_koef = 0.;
    supply_200V.temp_aux_time_const = 20.;
    irs::mlog() << irsm(" eeprom supply_200V") << endl;

    supply_20V.resistance_code = 521;
    supply_20V.koef_adc_volt_prev = 0.0295382;
    supply_20V.koef_adc_volt_fin = 0.0285352;
    supply_20V.koef_reg_prev = 2697.6;
    supply_20V.koef_reg_fin = 2697.6;

    supply_20V.temp_base_ref = 70.;
    supply_20V.temp_base_k = 15000.;
    supply_20V.temp_base_ki = 0.005;
    supply_20V.temp_base_kd = 50.;
    supply_20V.temp_base_prop_koef = 0.;
    supply_20V.temp_base_time_const = 20.;

    supply_20V.temp_aux_ref = 70.;
    supply_20V.temp_aux_k = 15000.;
    supply_20V.temp_aux_ki = 0.005;
    supply_20V.temp_aux_kd = 50.;
    supply_20V.temp_aux_prop_koef = 0.;
    supply_20V.temp_aux_time_const = 20.;
    irs::mlog() << irsm(" eeprom supply_20V") << endl;

    supply_2V.resistance_code = 501;
    supply_2V.koef_adc_volt_prev = 0.00962346;
    supply_2V.koef_adc_volt_fin = 0.00716917;
    supply_2V.koef_reg_prev = 15999.2;
    supply_2V.koef_reg_fin = 21520;

    supply_2V.temp_base_ref = 70.;
    supply_2V.temp_base_k = 15000.;
    supply_2V.temp_base_ki = 0.005;
    supply_2V.temp_base_kd = 50.;
    supply_2V.temp_base_prop_koef = 0.;
    supply_2V.temp_base_time_const = 20.;

    supply_2V.temp_aux_ref = 70.;
    supply_2V.temp_aux_k = 15000.;
    supply_2V.temp_aux_ki = 0.005;
    supply_2V.temp_aux_kd = 50.;
    supply_2V.temp_aux_prop_koef = 0.;
    supply_2V.temp_aux_time_const = 20.;
    irs::mlog() << irsm(" eeprom supply_2V") << endl;

    supply_1A.resistance_code = 426;
    supply_1A.koef_adc_volt_prev = 0.00482014;
    supply_1A.koef_adc_volt_fin = 0.00135671;
    supply_1A.koef_reg_prev = 15998.8;
    supply_1A.koef_reg_fin = 56847.5;

    supply_1A.temp_base_ref = 70.;
    supply_1A.temp_base_k = 15000.;
    supply_1A.temp_base_ki = 0.005;
    supply_1A.temp_base_kd = 50.;
    supply_1A.temp_base_prop_koef = 0.;
    supply_1A.temp_base_time_const = 20.;

    supply_1A.temp_aux_ref = 70.;
    supply_1A.temp_aux_k = 15000.;
    supply_1A.temp_aux_ki = 0.005;
    supply_1A.temp_aux_kd = 50.;
    supply_1A.temp_aux_prop_koef = 0.;
    supply_1A.temp_aux_time_const = 20.;
    irs::mlog() << irsm(" eeprom supply_1A") << endl;

    supply_17A.resistance_code = 875;
    supply_17A.koef_adc_volt_prev = 0.00479873;
    supply_17A.koef_adc_volt_fin = 0.0185929;
    supply_17A.koef_reg_prev = 15996.5;
    supply_17A.koef_reg_fin = 4144.1;

    supply_17A.temp_base_ref = 70.;
    supply_17A.temp_base_k = 15000.;
    supply_17A.temp_base_ki = 0.005;
    supply_17A.temp_base_kd = 50.;
    supply_17A.temp_base_prop_koef = 0.;
    supply_17A.temp_base_time_const = 20.;

    supply_17A.temp_aux_ref = 0.;
    supply_17A.temp_aux_k = 15000.;
    supply_17A.temp_aux_ki = 0.005;
    supply_17A.temp_aux_kd = 50.;
    supply_17A.temp_aux_prop_koef = 0.;
    supply_17A.temp_aux_time_const = 20.;
    irs::mlog() << irsm(" eeprom supply_17A") << endl;

    upper_level_check = 1;
    izm_th_spi_enable = 1;
    supply_comm_debug = 0;
    meas_comm_debug = 0;
  }
}; // eeprom_data_t

#endif // datah
