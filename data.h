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


enum { new_params_marker = 7 };
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
  irs::conn_data_t<irs_u8> mask_0;
  irs::conn_data_t<irs_u8> mask_1;
  irs::conn_data_t<irs_u8> mask_2;
  irs::conn_data_t<irs_u8> mask_3;
  irs::conn_data_t<irs_u8> gateway_0;
  irs::conn_data_t<irs_u8> gateway_1;
  irs::conn_data_t<irs_u8> gateway_2;
  irs::conn_data_t<irs_u8> gateway_3;
  irs::conn_data_t<irs_u8> mask_gateway_mark;

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

    irs_uarc options_index = index;
    index = options.connect(ap_data, index);
    
    upper_level_check.connect(ap_data, options_index, 0);
    izm_th_spi_enable.connect(ap_data, options_index, 1);
    supply_comm_debug.connect(ap_data, options_index, 2);
    meas_comm_debug.connect(ap_data, options_index, 3);
    
    index = mask_0.connect(ap_data, index);
    index = mask_1.connect(ap_data, index);
    index = mask_2.connect(ap_data, index);
    index = mask_3.connect(ap_data, index);
    index = gateway_0.connect(ap_data, index);
    index = gateway_1.connect(ap_data, index);
    index = gateway_2.connect(ap_data, index);
    index = gateway_3.connect(ap_data, index);
    index = mask_gateway_mark.connect(ap_data, index);
  
    return index;
  }

  inline void mask_gateway_reset_to_default()
  {
    mask_0 = 255;
    mask_1 = 255;
    mask_2 = 252;
    mask_3 = 0;

    gateway_0 = 192;
    gateway_1 = 168;
    gateway_2 = 0;
    gateway_3 = 1;

    mask_gateway_mark = new_params_marker;
  }
  
  inline void reset_to_default()
  {
    ip_0 = IP_0;
    ip_1 = IP_1;
    ip_2 = IP_2;
    ip_3 = IP_3;
    
    mask_gateway_reset_to_default();
    
    supply_200V.resistance_code = 488;
    supply_200V.koef_adc_volt_prev = 0.259343;
    supply_200V.koef_adc_volt_fin = 0.299369;
    supply_200V.koef_reg_prev = 297.572;
    supply_200V.koef_reg_fin = 257.774;

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

    supply_20V.resistance_code = 415;
    supply_20V.koef_adc_volt_prev = 0.0295382;
    supply_20V.koef_adc_volt_fin = 0.0259706;
    supply_20V.koef_reg_prev = 2697.6;
    supply_20V.koef_reg_fin = 2961.12;

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

    supply_2V.resistance_code = 499;
    supply_2V.koef_adc_volt_prev = 0.00962346;
    supply_2V.koef_adc_volt_fin = 0.00704233;
    supply_2V.koef_reg_prev = 15999.2;
    supply_2V.koef_reg_fin = 21791.8;

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

    supply_1A.resistance_code = 333;
    supply_1A.koef_adc_volt_prev = 0.00482014;
    supply_1A.koef_adc_volt_fin = 0.00134612;
    supply_1A.koef_reg_prev = 15998.8;
    supply_1A.koef_reg_fin = 57060;

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

    supply_17A.resistance_code = 900;
    supply_17A.koef_adc_volt_prev = 0.00479873;
    supply_17A.koef_adc_volt_fin = 0.0206957;
    supply_17A.koef_reg_prev = 15996.5;
    supply_17A.koef_reg_fin = 3719.02;

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

    upper_level_check = 0;
    izm_th_spi_enable = 0;
    supply_comm_debug = 0;
    meas_comm_debug = 0;
  }
}; // eeprom_data_t

#endif // datah
