#ifndef datah
#define datah

#include <irsnetdefs.h>

namespace u309m {

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
  
  /*irs::conn_data_t<float> meas_rele_power_voltage;
  irs::conn_data_t<float> power_voltage;
  irs::conn_data_t<float> meas_comm_plis_voltage;
  irs::conn_data_t<float> internal_temp;*/
  irs::conn_data_t<float> th1_value;
  irs::conn_data_t<float> th2_value;
  irs::conn_data_t<float> th3_value;
  irs::conn_data_t<float> th4_value;
  irs::conn_data_t<float> th5_value;
  
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
    
    /*index = meas_rele_power_voltage.connect(ap_data, index);
    index = power_voltage.connect(ap_data, index);
    index = meas_comm_plis_voltage.connect(ap_data, index);
    index = internal_temp.connect(ap_data, index);*/
    index = th1_value.connect(ap_data, index);
    index = th2_value.connect(ap_data, index);
    index = th3_value.connect(ap_data, index);
    index = th4_value.connect(ap_data, index);
    index = th5_value.connect(ap_data, index);
    
    return index;
  }
}; // meas_comm_data_t

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

struct meas_comm_pins_t {
  irs::gpio_pin_t* cs;
  irs::gpio_pin_t* reset;
  irs::gpio_pin_t* apply;
  irs::gpio_pin_t* error;
  irs::gpio_pin_t* termo_sense_1;
  irs::gpio_pin_t* termo_sense_2;
  irs::gpio_pin_t* termo_sense_3;
  irs::gpio_pin_t* termo_sense_4;
  irs::gpio_pin_t* termo_sense_5;
  
  meas_comm_pins_t(
    irs::gpio_pin_t* ap_cs,
    irs::gpio_pin_t* ap_reset,
    irs::gpio_pin_t* ap_apply,
    irs::gpio_pin_t* ap_error,
    irs::gpio_pin_t* ap_termo_sense_1,
    irs::gpio_pin_t* ap_termo_sense_2,
    irs::gpio_pin_t* ap_termo_sense_3,
    irs::gpio_pin_t* ap_termo_sense_4,
    irs::gpio_pin_t* ap_termo_sense_5
  ):
    cs(ap_cs),
    reset(ap_reset),
    apply(ap_apply),
    error(ap_error),
    termo_sense_1(ap_termo_sense_1),
    termo_sense_2(ap_termo_sense_2),
    termo_sense_3(ap_termo_sense_3),
    termo_sense_4(ap_termo_sense_4),
    termo_sense_5(ap_termo_sense_5)
  {
  }
}; // meas_comm_pins_t

struct supply_comm_pins_t {
  irs::gpio_pin_t* cs;
  irs::gpio_pin_t* reset;
  
  supply_comm_pins_t(
    irs::gpio_pin_t* ap_cs,
    irs::gpio_pin_t* ap_reset
  ):
    cs(ap_cs),
    reset(ap_reset)
  {
  }
}; // supply_comm_pins_t

struct supply_pins_t {
  irs::gpio_pin_t* termo_sense_base_cs;
  irs::gpio_pin_t* termo_sense_aux_cs;
  irs::gpio_pin_t* adc_cs;
  irs::gpio_pin_t* tc_cs;
  irs::gpio_pin_t* volt_reg_cs;
  irs::gpio_pin_t* temp_reg_cs;
  
  supply_pins_t(
    irs::gpio_pin_t* ap_termo_sense_base_cs,
    irs::gpio_pin_t* ap_termo_sense_aux_cs,
    irs::gpio_pin_t* ap_adc_cs,
    irs::gpio_pin_t* ap_tc_cs,
    irs::gpio_pin_t* ap_volt_reg_cs,
    irs::gpio_pin_t* ap_temp_reg_cs
  ):
    termo_sense_base_cs(ap_termo_sense_base_cs),
    termo_sense_aux_cs(ap_termo_sense_aux_cs),
    adc_cs(ap_adc_cs),
    tc_cs(ap_tc_cs),
    volt_reg_cs(ap_volt_reg_cs),
    temp_reg_cs(ap_temp_reg_cs)
  {
  }
}; // supply_pins_t

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
    
    return index;
  }
    
  void reset_to_default(supply_type_t a_supply_type)
  {
    ip_0 = 192;
    ip_1 = 168;
    ip_2 = 0;
    ip_3 = 211;
    switch(a_supply_type)
    {
      case sup_200V:
      {
        supply_200V.koef_adc_volt_prev = (4.096/1024)*(23.9/3.9)*1.2;
        supply_200V.koef_adc_volt_fin = (4.096/1024)*(23.9/3.9)*1.2;
        supply_200V.koef_reg_prev = 2633;
        supply_200V.koef_reg_fin = 2702;
        
        supply_200V.temp_base_ref = 60;
        supply_200V.temp_base_k = 15000;
        supply_200V.temp_base_ki = 0.00075;
        supply_200V.temp_base_kd = 200;
        supply_200V.temp_base_prop_koef = 0;
        supply_200V.temp_base_time_const = 20;
        
        supply_200V.temp_aux_ref = 60;
        supply_200V.temp_aux_k = 15000;
        supply_200V.temp_aux_ki = 0.00075;
        supply_200V.temp_aux_kd = 200;
        supply_200V.temp_aux_prop_koef = 0;
        supply_200V.temp_aux_time_const = 20;
        irs::mlog() << irsm(" eeprom supply_200V") << endl;
      } break;
      case sup_20V:
      {
        supply_20V.koef_adc_volt_prev = (4.096/1024)*(23.9/3.9)*1.2;
        supply_20V.koef_adc_volt_fin = (4.096/1024)*(23.9/3.9)*1.2;
        supply_20V.koef_reg_prev = 2633;
        supply_20V.koef_reg_fin = 2702;
        
        supply_20V.temp_base_ref = 60;
        supply_20V.temp_base_k = 15000;
        supply_20V.temp_base_ki = 0.00075;
        supply_20V.temp_base_kd = 200;
        supply_20V.temp_base_prop_koef = 0;
        supply_20V.temp_base_time_const = 20;
        
        supply_20V.temp_aux_ref = 60;
        supply_20V.temp_aux_k = 15000;
        supply_20V.temp_aux_ki = 0.00075;
        supply_20V.temp_aux_kd = 200;
        supply_20V.temp_aux_prop_koef = 0;
        supply_20V.temp_aux_time_const = 20;
        irs::mlog() << irsm(" eeprom supply_20V") << endl;
      } break;
      case sup_2V:
      {
        supply_2V.koef_adc_volt_prev = (4.096/1024)*(23.9/3.9)*1.2;
        supply_2V.koef_adc_volt_fin = (4.096/1024)*(23.9/3.9)*1.2;
        supply_2V.koef_reg_prev = 2633;
        supply_2V.koef_reg_fin = 2702;
        
        supply_2V.temp_base_ref = 60;
        supply_2V.temp_base_k = 15000;
        supply_2V.temp_base_ki = 0.00075;
        supply_2V.temp_base_kd = 200;
        supply_2V.temp_base_prop_koef = 0;
        supply_2V.temp_base_time_const = 20;
        
        supply_2V.temp_aux_ref = 60;
        supply_2V.temp_aux_k = 15000;
        supply_2V.temp_aux_ki = 0.00075;
        supply_2V.temp_aux_kd = 200;
        supply_2V.temp_aux_prop_koef = 0;
        supply_2V.temp_aux_time_const = 20;
        irs::mlog() << irsm(" eeprom supply_2V") << endl;
      } break;
      case sup_1A:
      {
        supply_1A.koef_adc_volt_prev = (4.096/1024)*(23.9/3.9)*1.2;
        supply_1A.koef_adc_volt_fin = (4.096/1024)*(23.9/3.9)*1.2;
        supply_1A.koef_reg_prev = 2633;
        supply_1A.koef_reg_fin = 2702;
        
        supply_1A.temp_base_ref = 60;
        supply_1A.temp_base_k = 15000;
        supply_1A.temp_base_ki = 0.00075;
        supply_1A.temp_base_kd = 200;
        supply_1A.temp_base_prop_koef = 0;
        supply_1A.temp_base_time_const = 20;
        
        supply_1A.temp_aux_ref = 60;
        supply_1A.temp_aux_k = 15000;
        supply_1A.temp_aux_ki = 0.00075;
        supply_1A.temp_aux_kd = 200;
        supply_1A.temp_aux_prop_koef = 0;
        supply_1A.temp_aux_time_const = 20;
        irs::mlog() << irsm(" eeprom supply_1A") << endl;
      } break;
      case sup_17A:
      {
        supply_17A.koef_adc_volt_prev = (4.096/1024)*(23.9/3.9)*1.2;
        supply_17A.koef_adc_volt_fin = (4.096/1024)*(23.9/3.9)*1.2;
        supply_17A.koef_reg_prev = 2633;
        supply_17A.koef_reg_fin = 2702;
        
        supply_17A.temp_base_ref = 60;
        supply_17A.temp_base_k = 15000;
        supply_17A.temp_base_ki = 0.00075;
        supply_17A.temp_base_kd = 200;
        supply_17A.temp_base_prop_koef = 0;
        supply_17A.temp_base_time_const = 20;
        
        supply_17A.temp_aux_ref = 60;
        supply_17A.temp_aux_k = 15000;
        supply_17A.temp_aux_ki = 0.00075;
        supply_17A.temp_aux_kd = 200;
        supply_17A.temp_aux_prop_koef = 0;
        supply_17A.temp_aux_time_const = 20;
        irs::mlog() << irsm(" eeprom supply_17A") << endl;
      } break;
      default:
      {
        IRS_LIB_ASSERT_MSG("неверно указан тип источника");
      }
    }
  }
}; // eeprom_data_t

} // namespace u309m

#endif // datah
