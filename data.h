#ifndef datah
#define datah

namespace u309m {

struct supply_comm_data_t
{
  irs::conn_data_t<irs_u8> etalon_cell;
  irs::conn_data_t<irs_u8> calibrated_cell;
  irs::bit_data_t polarity_etalon;
  irs::bit_data_t polarity_calibrated;
  irs::bit_data_t apply;
  irs::bit_data_t on;
  irs::bit_data_t error;
  
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
    
    index = etalon_cell.connect(ap_data, index);
    index = calibrated_cell.connect(ap_data, index);
    polarity_etalon.connect(ap_data, index, 0);
    polarity_calibrated.connect(ap_data, index, 1);
    apply.connect(ap_data, index, 2);
    on.connect(ap_data, index, 3);
    error.connect(ap_data, index, 4);
    index++;
    
    return index;
  }
};

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
  
  irs::conn_data_t<float> meas_rele_power_voltage;
  irs::conn_data_t<float> power_voltage;
  irs::conn_data_t<float> meas_comm_plis_voltage;
  irs::conn_data_t<float> internal_temp;
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
    
    index = meas_rele_power_voltage.connect(ap_data, index);
    index = power_voltage.connect(ap_data, index);
    index = meas_comm_plis_voltage.connect(ap_data, index);
    index = internal_temp.connect(ap_data, index);
    index = th1_value.connect(ap_data, index);
    index = th2_value.connect(ap_data, index);
    index = th3_value.connect(ap_data, index);
    index = th4_value.connect(ap_data, index);
    index = th5_value.connect(ap_data, index);
    
    return index;
  }
};

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
  // Commutator
  irs::bit_data_t meas_rele_1;
  irs::bit_data_t meas_rele_2;
  irs::bit_data_t polarity;
  irs::bit_data_t dir;
  irs::bit_data_t load;
  irs::bit_data_t sr_200;

  // Supply
  irs::conn_data_t<irs_u16> resistance_code;
  dac_data_t prev_dac_data;
  adc_data_t prev_adc_data;
  dac_data_t fin_dac_data;
  adc_data_t fin_adc_data;
  tr_data_t base_tr_data;
  temp_data_t base_temp_data;
  tr_data_t aux_tr_data;
  temp_data_t aux_temp_data;

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
    
    // Commutator
    meas_rele_1.connect(ap_data, index, 0);
    meas_rele_2.connect(ap_data, index, 1);
    polarity.connect(ap_data, index, 2);
    dir.connect(ap_data, index, 3);
    load.connect(ap_data, index, 4);
    sr_200.connect(ap_data, index, 5);
    index++;
    
    // Supply
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
  irs::gpio_pin_t* apply;
  irs::gpio_pin_t* error;
  irs::gpio_pin_t* termo_sense_1;
  irs::gpio_pin_t* termo_sense_2;
  irs::gpio_pin_t* termo_sense_3;
  irs::gpio_pin_t* termo_sense_4;
  irs::gpio_pin_t* termo_sense_5;
  
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

struct eeprom_data_t {

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
    
    return index;
  }
}; // eeprom_data_t

} // namespace u309m

#endif // datah
