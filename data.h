#ifndef datah
#define datah

#include <irsdefs.h>

#include <irsnetdefs.h>

#include "privatecfg.h"

#include <irsfinal.h>

namespace u309m {

struct rele_ext_eth_data_t {
  irs::bit_data_t SYM_2V;
  irs::bit_data_t SYM_20V;
  irs::bit_data_t SYM_200V;
  irs::bit_data_t KZ_2V;
  irs::bit_data_t KZ_1A;
  irs::bit_data_t KZ_17A;
  irs::bit_data_t REL_220V;
  irs::bit_data_t SYM_OFF;
  irs::bit_data_t SYM_OFF_TEST;

  rele_ext_eth_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    SYM_2V.connect(ap_data, index, 0);
    SYM_20V.connect(ap_data, index, 1);
    SYM_200V.connect(ap_data, index, 2);
    KZ_2V.connect(ap_data, index, 3);
    KZ_1A.connect(ap_data, index, 4);
    KZ_17A.connect(ap_data, index, 5);
    REL_220V.connect(ap_data, index, 6);
    SYM_OFF.connect(ap_data, index, 7);
    index++;
    SYM_OFF_TEST.connect(ap_data, index, 0);
    index++;

    return index;
  }
}; // rele_ext_eth_data_t

enum supply_type_t{
  sup_200V,
  sup_20V,
  sup_2V,
  sup_1A,
  sup_17A
};

struct supply_comm_data_t
{
  irs::conn_data_t<irs_u8> supply_index;
  irs::conn_data_t<irs_u8> etalon_cell;
  irs::conn_data_t<irs_u8> calibrated_cell;
  irs::bit_data_t polarity_etalon;
  irs::bit_data_t polarity_calibrated;
  irs::bit_data_t apply;
  irs::bit_data_t on;
  irs::bit_data_t error;
  irs::bit_data_t reset;

  supply_comm_data_t(irs::mxdata_t *ap_data = IRS_NULL,
    irs_uarc a_index = 0, irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t* ap_data = 0, irs_uarc a_start_index = 0)
  {
    irs_uarc index = a_start_index;

    index = supply_index.connect(ap_data, index);
    index = etalon_cell.connect(ap_data, index);
    index = calibrated_cell.connect(ap_data, index);
    polarity_etalon.connect(ap_data, index, 0);
    polarity_calibrated.connect(ap_data, index, 1);
    apply.connect(ap_data, index, 2);
    on.connect(ap_data, index, 3);
    error.connect(ap_data, index, 4);
    reset.connect(ap_data, index, 5);
    index++;

    return index;
  }
}; // supply_comm_data_t

struct meas_comm_data_t
{
  irs::conn_data_t<irs_u8> mode;
  irs::conn_data_t<irs_u8> etalon_cell;
  irs::conn_data_t<irs_u8> calibrated_cell;
  irs::bit_data_t load_resistor;
  irs::bit_data_t apply;
  irs::bit_data_t on;
  irs::bit_data_t error;
  irs::bit_data_t reset;

  meas_comm_data_t(irs::mxdata_t *ap_data = IRS_NULL,
    irs_uarc a_index = 0, irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t* ap_data = 0, irs_uarc a_index = 0)
  {
    irs_uarc index = a_index;

    index = mode.connect(ap_data, index);
    index = etalon_cell.connect(ap_data, index);
    index = calibrated_cell.connect(ap_data, index);
    load_resistor.connect(ap_data, index, 0);
    apply.connect(ap_data, index, 1);
    on.connect(ap_data, index, 2);
    error.connect(ap_data, index, 3);
    reset.connect(ap_data, index, 4);
    index++;

    return index;
  }
}; // meas_comm_data_t

struct meas_comm_th_data_t
{
  irs::conn_data_t<float> th1_value;
  irs::conn_data_t<float> th2_value;
  irs::conn_data_t<float> th3_value;
  irs::conn_data_t<float> th4_value;
  irs::conn_data_t<float> th5_value;

  meas_comm_th_data_t(irs::mxdata_t *ap_data = IRS_NULL,
    irs_uarc a_index = 0, irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t* ap_data = 0, irs_uarc a_index = 0)
  {
    irs_uarc index = a_index;

    index = th1_value.connect(ap_data, index);
    index = th2_value.connect(ap_data, index);
    index = th3_value.connect(ap_data, index);
    index = th4_value.connect(ap_data, index);
    index = th5_value.connect(ap_data, index);

    return index;
  }
}; // meas_comm_th_data_t

struct arm_adc_data_t {
  irs::conn_data_t<float> PTC_A;
  irs::conn_data_t<float> PTC_LC;
  irs::conn_data_t<float> TR_24V_TEST;
  irs::conn_data_t<float> IZM_3_3V_TEST;
  irs::conn_data_t<float> IZM_6V_TEST;
  irs::conn_data_t<float> IZM_1_2V_TEST;
  irs::conn_data_t<float> TEST_24V;
  irs::conn_data_t<float> TEST_5V;
  irs::conn_data_t<float> PTC_PWR;
  irs::conn_data_t<float> PTC_17A;
  irs::conn_data_t<float> internal_temp;

  arm_adc_data_t(irs::mxdata_t *ap_data = IRS_NULL,
    irs_uarc a_index = 0, irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t* ap_data = 0, irs_uarc a_index = 0)
  {
    irs_uarc index = a_index;

    index = PTC_A.connect(ap_data, index);
    index = PTC_LC.connect(ap_data, index);
    index = TR_24V_TEST.connect(ap_data, index);
    index = IZM_3_3V_TEST.connect(ap_data, index);
    index = IZM_6V_TEST.connect(ap_data, index);
    index = IZM_1_2V_TEST.connect(ap_data, index);
    index = TEST_24V.connect(ap_data, index);
    index = TEST_5V.connect(ap_data, index);
    index = PTC_PWR.connect(ap_data, index);
    index = PTC_17A.connect(ap_data, index);
    index = internal_temp.connect(ap_data, index);

    return index;
  }
}; // arm_adc_data_t

struct temp_data_t {
  irs::conn_data_t<float> value;
  irs::conn_data_t<float> filtered_value;

  temp_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
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

    index = value.connect(ap_data, index);
    index = filtered_value.connect(ap_data, index);

    return index;
  }
};

struct dac_data_t {
  irs::conn_data_t<float> voltage_code;
  irs::conn_data_t<float> koef;

  dac_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
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

    index = voltage_code.connect(ap_data, index);
    index = koef.connect(ap_data, index);

    return index;
  }
};

struct adc_data_t {
  irs::conn_data_t<float> voltage_code;
  irs::conn_data_t<float> koef;

  adc_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
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

    index = voltage_code.connect(ap_data, index);
    index = koef.connect(ap_data, index);

    return index;
  }
};

struct tr_data_t {
  irs::conn_data_t<float> temperature_ref;
  irs::conn_data_t<float> temp_k;
  irs::conn_data_t<float> temp_ki;
  irs::conn_data_t<float> temp_kd;
  irs::conn_data_t<float> dac_value;
  irs::conn_data_t<float> temp_prop_koef;
  irs::conn_data_t<float> temp_time_const;
  irs::conn_data_t<irs_i32> int_val;

  tr_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
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

    index = temperature_ref.connect(ap_data, index);
    index = temp_k.connect(ap_data, index);
    index = temp_ki.connect(ap_data, index);
    index = temp_kd.connect(ap_data, index);
    index = dac_value.connect(ap_data, index);
    index = temp_prop_koef.connect(ap_data, index);
    index = temp_time_const.connect(ap_data, index);
    index = int_val.connect(ap_data, index);

    return index;
  }
};

struct supply_eth_data_t {
  irs::conn_data_t<irs_u16> resistance_code;
  dac_data_t prev_dac_data; // 8
  adc_data_t prev_adc_data; // 8
  dac_data_t fin_dac_data; // 8
  adc_data_t fin_adc_data; // 8
  tr_data_t base_tr_data; // 32
  temp_data_t base_temp_data; // 8
  tr_data_t aux_tr_data; // 32
  temp_data_t aux_temp_data; // 8

  supply_eth_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
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

    index = resistance_code.connect(ap_data, index);

    index = prev_dac_data.connect(ap_data, index);
    index = prev_adc_data.connect(ap_data, index);
    index = fin_dac_data.connect(ap_data, index);
    index = fin_adc_data.connect(ap_data, index);

    index = base_tr_data.connect(ap_data, index);
    index = base_temp_data.connect(ap_data, index);

    index = aux_tr_data.connect(ap_data, index);
    index = aux_temp_data.connect(ap_data, index);

    return index;
  }
}; // supply_eth_data_t

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
    index = options.connect(ap_data, index);

    return index;
  }

  void reset_to_default()
  {
    ip_0 = IP_0;
    ip_1 = IP_1;
    ip_2 = IP_2;
    ip_3 = IP_3;
    
    supply_200V.resistance_code = 976;
    supply_200V.koef_adc_volt_prev = 0.259343;
    supply_200V.koef_adc_volt_fin = 0.292788;
    supply_200V.koef_reg_prev = 297.572;
    supply_200V.koef_reg_fin = 263.678;

    supply_200V.temp_base_ref = 70.;
    supply_200V.temp_base_k = 1500.;
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

    supply_20V.resistance_code = 463;
    supply_20V.koef_adc_volt_prev = 0.029538188;
    supply_20V.koef_adc_volt_fin = 0.025330844;
    supply_20V.koef_reg_prev = 2610.304968;
    supply_20V.koef_reg_fin = 3043.865432;

    supply_20V.temp_base_ref = 60.;
    supply_20V.temp_base_k = 7500.;
    supply_20V.temp_base_ki = 0.0015;
    supply_20V.temp_base_kd = 5.;
    supply_20V.temp_base_prop_koef = 0.3;
    supply_20V.temp_base_time_const = 5.;

    supply_20V.temp_aux_ref = 60.;
    supply_20V.temp_aux_k = 15000.;
    supply_20V.temp_aux_ki = 0.0015;
    supply_20V.temp_aux_kd = 5.;
    supply_20V.temp_aux_prop_koef = 0.3;
    supply_20V.temp_aux_time_const = 5.;
    irs::mlog() << irsm(" eeprom supply_20V") << endl;

    supply_2V.resistance_code = 463;
    supply_2V.koef_adc_volt_prev = 0.00962346;
    supply_2V.koef_adc_volt_fin = 0.006859275;
    supply_2V.koef_reg_prev = 15999.2186;
    supply_2V.koef_reg_fin = 22446.6649;

    supply_2V.temp_base_ref = 60.;
    supply_2V.temp_base_k = 7500.;
    supply_2V.temp_base_ki = 0.0015;
    supply_2V.temp_base_kd = 5.;
    supply_2V.temp_base_prop_koef = 0.3;
    supply_2V.temp_base_time_const = 5.;

    supply_2V.temp_aux_ref = 60.;
    supply_2V.temp_aux_k = 15000.;
    supply_2V.temp_aux_ki = 0.0015;
    supply_2V.temp_aux_kd = 5.;
    supply_2V.temp_aux_prop_koef = 0.3;
    supply_2V.temp_aux_time_const = 5.;
    irs::mlog() << irsm(" eeprom supply_2V") << endl;

    supply_1A.resistance_code = 512;
    supply_1A.koef_adc_volt_prev = 0.004820137;
    supply_1A.koef_adc_volt_fin = 0.001420473;
    supply_1A.koef_reg_prev = 15998.82792;
    supply_1A.koef_reg_fin = 54289.35609;

    supply_1A.temp_base_ref = 60.;
    supply_1A.temp_base_k = 7500.;
    supply_1A.temp_base_ki = 0.0015;
    supply_1A.temp_base_kd = 5.;
    supply_1A.temp_base_prop_koef = 0.3;
    supply_1A.temp_base_time_const = 5.;

    supply_1A.temp_aux_ref = 60.;
    supply_1A.temp_aux_k = 15000.;
    supply_1A.temp_aux_ki = 0.0015;
    supply_1A.temp_aux_kd = 5.;
    supply_1A.temp_aux_prop_koef = 0.3;
    supply_1A.temp_aux_time_const = 5.;
    irs::mlog() << irsm(" eeprom supply_1A") << endl;

    supply_17A.resistance_code = 512;
    supply_17A.koef_adc_volt_prev = 0.004798729;
    supply_17A.koef_adc_volt_fin = 0.020719409;
    supply_17A.koef_reg_prev = 15996.48429;
    supply_17A.koef_reg_fin = 3704.873815;

    supply_17A.temp_base_ref = 60.;
    supply_17A.temp_base_k = 7500.;
    supply_17A.temp_base_ki = 0.0015;
    supply_17A.temp_base_kd = 5.;
    supply_17A.temp_base_prop_koef = 0.3;
    supply_17A.temp_base_time_const = 5.;

    supply_17A.temp_aux_ref = 60.;
    supply_17A.temp_aux_k = 15000.;
    supply_17A.temp_aux_ki = 0.0015;
    supply_17A.temp_aux_kd = 5.;
    supply_17A.temp_aux_prop_koef = 0.3;
    supply_17A.temp_aux_time_const = 5.;
    irs::mlog() << irsm(" eeprom supply_17A") << endl;

    upper_level_check = 1;
    izm_th_spi_enable = 1;
  }
}; // eeprom_data_t

struct control_data_t
{
  irs::conn_data_t<irs_u32> alarm;
  irs::bit_data_t alarm_internal_th;
  irs::bit_data_t alarm_ptc_a;
  irs::bit_data_t alarm_ptc_lc;
  irs::bit_data_t alarm_ptc_pwr;
  irs::bit_data_t alarm_ptc_17A;
  irs::bit_data_t alarm_tr_24V;
  irs::bit_data_t alarm_24V;
  irs::bit_data_t alarm_5V;
  irs::bit_data_t alarm_izm_6V;
  irs::bit_data_t alarm_izm_3_3V;
  irs::bit_data_t alarm_izm_1_2V;
  irs::bit_data_t alarm_izm_th1;
  irs::bit_data_t alarm_izm_th2;
  irs::bit_data_t alarm_izm_th3;
  irs::bit_data_t alarm_izm_th4;
  irs::bit_data_t alarm_izm_th5;
  irs::bit_data_t alarm_200V_th_base;
  irs::bit_data_t alarm_200V_th_aux;
  irs::bit_data_t alarm_20V_th_base;
  irs::bit_data_t alarm_20V_th_aux;
  irs::bit_data_t alarm_2V_th_base;
  irs::bit_data_t alarm_2V_th_aux;
  irs::bit_data_t alarm_1A_th_base;
  irs::bit_data_t alarm_1A_th_aux;
  irs::bit_data_t alarm_17A_th_base;
  irs::bit_data_t alarm_17A_th_aux;
  irs::bit_data_t alarm_upper_level;

  irs::bit_data_t on;

  irs::conn_data_t<irs_u8> unlock;

  irs::bit_data_t ready_200V_prev;
  irs::bit_data_t ready_200V_final;
  irs::bit_data_t ready_20V_prev;
  irs::bit_data_t ready_20V_final;
  irs::bit_data_t ready_2V_prev;
  irs::bit_data_t ready_2V_final;
  irs::bit_data_t ready_1A_prev;
  irs::bit_data_t ready_1A_final;
  irs::bit_data_t ready_17A_prev;
  irs::bit_data_t ready_17A_final;

  irs::bit_data_t upper_level_check;
  irs::bit_data_t refresh_all_sources;
  irs::bit_data_t watchdog_reset_cause;
  irs::bit_data_t watchdog_test;
  irs::bit_data_t izm_th_spi_enable;

  irs::conn_data_t<irs_u32> connect_counter;

  control_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
      *ap_size = size;
    }
  }

  irs_uarc connect(irs::mxdata_t *ap_data, irs_uarc a_index)
  {
    irs_uarc index = a_index;

    alarm_internal_th.connect(ap_data, index, 0);
    alarm_ptc_a.connect(ap_data, index, 1);
    alarm_ptc_lc.connect(ap_data, index, 2);
    alarm_ptc_pwr.connect(ap_data, index, 3);
    alarm_ptc_17A.connect(ap_data, index, 4);
    alarm_tr_24V.connect(ap_data, index, 5);
    alarm_24V.connect(ap_data, index, 6);
    alarm_5V.connect(ap_data, index, 7);

    alarm_izm_6V.connect(ap_data, index + 1, 0);
    alarm_izm_3_3V.connect(ap_data, index + 1, 1);
    alarm_izm_1_2V.connect(ap_data, index + 1, 2);
    alarm_izm_th1.connect(ap_data, index + 1, 3);
    alarm_izm_th2.connect(ap_data, index + 1, 4);
    alarm_izm_th3.connect(ap_data, index + 1, 5);
    alarm_izm_th4.connect(ap_data, index + 1, 6);
    alarm_izm_th5.connect(ap_data, index + 1, 7);

    alarm_200V_th_base.connect(ap_data, index + 2, 0);
    alarm_200V_th_aux.connect(ap_data, index + 2, 1);
    alarm_20V_th_base.connect(ap_data, index + 2, 2);
    alarm_20V_th_aux.connect(ap_data, index + 2, 3);
    alarm_2V_th_base.connect(ap_data, index + 2, 4);
    alarm_2V_th_aux.connect(ap_data, index + 2, 5);
    alarm_1A_th_base.connect(ap_data, index + 2, 6);
    alarm_1A_th_aux.connect(ap_data, index + 2, 7);

    alarm_17A_th_base.connect(ap_data, index + 3, 0);
    alarm_17A_th_aux.connect(ap_data, index + 3, 1);
    alarm_upper_level.connect(ap_data, index + 3, 2);

    on.connect(ap_data, index + 3, 7);

    index = alarm.connect(ap_data, index);

    index = unlock.connect(ap_data, index);

    ready_200V_prev.connect(ap_data, index, 0);
    ready_200V_final.connect(ap_data, index, 1);
    ready_20V_prev.connect(ap_data, index, 2);
    ready_20V_final.connect(ap_data, index, 3);
    ready_2V_prev.connect(ap_data, index, 4);
    ready_2V_final.connect(ap_data, index, 5);
    ready_1A_prev.connect(ap_data, index, 6);
    ready_1A_final.connect(ap_data, index, 7);

    index++;

    ready_17A_prev.connect(ap_data, index, 0);
    ready_17A_final.connect(ap_data, index, 1);

    upper_level_check.connect(ap_data, index, 2);
    refresh_all_sources.connect(ap_data, index, 3);
    watchdog_reset_cause.connect(ap_data, index, 4);
    watchdog_test.connect(ap_data, index, 5);
    izm_th_spi_enable.connect(ap_data, index, 6);

    index++;
    index++;

    index = connect_counter.connect(ap_data, index);

    return index;
  }
}; // control_data_t

struct eth_data_t {
  irs::conn_data_t<irs_u8> ip_0;  //  1 byte
  irs::conn_data_t<irs_u8> ip_1;  //  1 byte
  irs::conn_data_t<irs_u8> ip_2;  //  1 byte
  irs::conn_data_t<irs_u8> ip_3;  //  1 byte
  rele_ext_eth_data_t rele_ext;   //  2 bytes
  supply_comm_data_t supply_comm; //  4 bytes
  meas_comm_data_t meas_comm;     //  4 bytes
  meas_comm_th_data_t meas_comm_th;// 20 bytes
  arm_adc_data_t arm_adc;         //  44 bytes
  supply_eth_data_t supply_200V;  //  114 bytes
  supply_eth_data_t supply_20V;   //  114 bytes
  supply_eth_data_t supply_2V;    //  114 bytes
  supply_eth_data_t supply_1A;    //  114 bytes
  supply_eth_data_t supply_17A;   //  114 bytes
  control_data_t control;         //  12 bytes
  //---------------------------------------------
  //                          Итого:  660 байт
 
  eth_data_t(irs::mxdata_t *ap_data = IRS_NULL, irs_uarc a_index = 0,
    irs_uarc* ap_size = IRS_NULL)
  {
    irs_uarc size = connect(ap_data, a_index);
    if (ap_size != IRS_NULL) {
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
    index = rele_ext.connect(ap_data, index);
    index = supply_comm.connect(ap_data, index);
    index = meas_comm.connect(ap_data, index);
    index = meas_comm_th.connect(ap_data, index);
    index = arm_adc.connect(ap_data, index);
    index = supply_200V.connect(ap_data, index);
    index = supply_20V.connect(ap_data, index);
    index = supply_2V.connect(ap_data, index);
    index = supply_1A.connect(ap_data, index);
    index = supply_17A.connect(ap_data, index);
    index = control.connect(ap_data, index);   
   
    return index;
  }
};

} // namespace u309m

#endif // datah
