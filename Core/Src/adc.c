#include "variables.h"

//// input voltage         -600     --   0    -- 600
//// transformer            -1      --   0    --  1
//// off set introduced    1.65     --  1.65  -- 1.65
//
//// adc input             0.65     --  1.65  --  2.65
//// adc value            806.5909  -- 2047.5 -- 3288.4090
//
//// adc value - offset   -1240.909091 -- 0   -- 1240.909091
//
//// multiplication factor -600/-1240.90  0/0    600/1240.9090
//// recalculated           (adc-off) * multiplication_factor
//// voltage recalculated given to Vab , Vbc, Vca , ia, ib, ic, vdc, idc
//float a = adc[0]; // Vab  --> 806.59
//a = a - offset;
//multiplication_factor = input/a;
//vab = multiplication_factor * a;

//	float voltage_offset = 2047.5;
//	float current_offset = 2047.5;
//	float vdc_offset = 2047.5;
//	float idc_offset = 2047.5;
//
//	float multiplication_factor = 0.4835164835;


void adcReadings(void){

	// OffSet and Multiplication factor
	adc_read.offSet              = 2306.05802;       // 2047.5f;
	adc_read.voltage_gain_factor = 0.4293040293f;
	adc_read.current_gain_factor = 0.4293040293f;  //0.30835164835f;
	adc_read.Vdc_gain_factor     = 0.4293040293f;
	adc_read.Idc_gain_factor     = 0.4293040293f;
	adc_read.bat_i_gain_factor   = 0.4293040293f;
	adc_read.bat_volt_gain_factor= 0.4293040293f;
	adc_read.inter_i_gain_factor = 0.4293040293f;
	adc_read.inter_volt_gain_factor = 0.4293040293f;
	adc_read.inv_temp_gain_factor   = 0.4293040293f;
	adc_read.v_ref_gain_factor      = 0.4293040293f;
	adc_read.earth_fault_gain_factor= 0.4293040293f;

	// raw adc values
	adc_read.ia_in  = adc_buffer[0];  //PA0
	adc_read.ib_in  = adc_buffer[1];  //PA1
	adc_read.ic_in  = adc_buffer[2];  //PA2
	adc_read.Vab_in = adc_buffer[3];  //PA3
	adc_read.Vbc_in = adc_buffer[4];  //PA4
	adc_read.Vca_in = adc_buffer[5];  //PA5
	adc_read.idc_in = adc_buffer[6];  //PA6
	adc_read.Vdc_in = adc_buffer[7];  //PB0
	adc_read.battery_i_in    = adc_buffer[8]; //PB1
	adc_read.battery_volt_in = adc_buffer[9]; //PC0
	adc_read.inter_i_in      = adc_buffer[10]; //PC1
	adc_read.inter_volt_in   = adc_buffer[11]; //PC2
	adc_read.inv_temp_in     = adc_buffer[12]; //PC3
	adc_read.v_ref_in        = adc_buffer[13]; //PC4
	adc_read.earth_fault_in  = adc_buffer[14]; //PC5

	// adc - offset
	adc_read.Vab_corrected = (float)adc_read.Vab_in - adc_read.offSet;
	adc_read.Vbc_corrected = (float)adc_read.Vbc_in - adc_read.offSet;
	adc_read.Vca_corrected = (float)adc_read.Vca_in - adc_read.offSet;
	adc_read.ia_corrected  = (float)adc_read.ia_in  - adc_read.offSet;
	adc_read.ib_corrected  = (float)adc_read.ib_in  - adc_read.offSet;
	adc_read.ic_corrected  = (float)adc_read.ic_in  - adc_read.offSet;
	adc_read.Vdc_corrected = (float)adc_read.Vdc_in - adc_read.offSet;
	adc_read.Idc_corrected = (float)adc_read.idc_in - adc_read.offSet;
	adc_read.bat_i_corrected       = (float)adc_read.battery_i_in    - adc_read.offSet;
	adc_read.bat_volt_corrected    = (float)adc_read.battery_volt_in - adc_read.offSet;
	adc_read.inter_i_corrected     = (float)adc_read.inter_i_in      - adc_read.offSet;
	adc_read.inter_volt_corrected  = (float)adc_read.inter_volt_in   - adc_read.offSet;
	adc_read.inv_temp_corrected    = (float)adc_read.inv_temp_in     - adc_read.offSet;
	adc_read.v_ref_corrected       = (float)adc_read.v_ref_in        - adc_read.offSet;
	adc_read.earth_fault_corrected = (float)adc_read.earth_fault_in  - adc_read.offSet;


	// multiplication factor
	// multiplication_factor = input_voltage_reference/adc_read.Vab_corrected;
	// voltage recalculated

	adc_read.Vab_recalculated = adc_read.voltage_gain_factor * adc_read.Vab_corrected;
	adc_read.Vbc_recalculated = adc_read.voltage_gain_factor * adc_read.Vbc_corrected;
	adc_read.Vca_recalculated = adc_read.voltage_gain_factor * adc_read.Vca_corrected;
	adc_read.ia_recalculated  = adc_read.current_gain_factor * adc_read.ia_corrected;
	adc_read.ib_recalculated  = adc_read.current_gain_factor * adc_read.ib_corrected;
	adc_read.ic_recalculated  = adc_read.current_gain_factor * adc_read.ic_corrected;
	adc_read.Vdc_recalculated = adc_read.Vdc_gain_factor     * adc_read.Vdc_corrected;
	adc_read.Idc_recalculated = adc_read.Idc_gain_factor     * adc_read.Idc_corrected;
	adc_read.bat_i_recalculated       = adc_read.bat_i_gain_factor       * adc_read.bat_i_corrected;
	adc_read.bat_volt_recalculated    = adc_read.bat_volt_gain_factor    * adc_read.bat_volt_corrected;
	adc_read.inter_i_recalculated     = adc_read.inter_i_gain_factor     * adc_read.inter_i_corrected;
	adc_read.inter_volt_recalculated  = adc_read.inter_volt_gain_factor  * adc_read.inter_volt_corrected;
	adc_read.inv_temp_recalculated    = adc_read.inv_temp_gain_factor    * adc_read.inv_temp_corrected;
	adc_read.v_ref_recalculated       = adc_read.v_ref_gain_factor       * adc_read.inv_temp_corrected;
	adc_read.earth_fault_recalculated = adc_read.earth_fault_gain_factor * adc_read.earth_fault_corrected;

	// Assign recalculated values
	adc_read.Va = ((2.0f * adc_read.Vab_recalculated) + adc_read.Vbc_recalculated) / 3.0f; // instVal.Va_inst // grid.Va  //rms_calculations.Va
	adc_read.Vb = -((2.0f * adc_read.Vab_recalculated) + adc_read.Vbc_recalculated)/ 3.0f; // instVal.Vb_inst // grid.Vb  //rms_calculations.Vb
	adc_read.Vc = -(adc_read.Vab_recalculated + (2.0f * adc_read.Vbc_recalculated))/ 3.0f; // instVal.Vc_inst // grid.Vc  //rms_calculations.Vc

	adc_read.ia = adc_read.ia_recalculated;  // instVal.Ia_inst // grid.Ia  //rms_calculations.ia
	adc_read.ib = adc_read.ib_recalculated;  // instVal.Ia_inst // grid.Ib  //rms_calculations.ib
	adc_read.ic = adc_read.ic_recalculated;  // instVal.Ia_inst // grid.Ic  //rms_calculations.ic
	adc_read.Vdc= adc_read.Vdc_recalculated; // Vdc_inst //avg_calculations.Vdc
	adc_read.Idc= adc_read.Idc_recalculated; // idc_inst //avg_calculations.Idc
	adc_read.battery_i         = adc_read.bat_i_recalculated;
	adc_read.battery_volt      = adc_read.bat_volt_recalculated;
	adc_read.intermediate_i    = adc_read.inter_i_recalculated;
	adc_read.intermediate_volt = adc_read.inter_volt_recalculated;
	adc_read.inv_temp          = adc_read.inv_temp_recalculated;
	adc_read.v_ref             = adc_read.inv_temp_recalculated;
	adc_read.earth_fault       = adc_read.earth_fault_recalculated;




//	grid.Va = adc_read.Va;
//	grid.Vb = adc_read.Vb;
//	grid.Vc = adc_read.Vc;
//
//	instVal.Va_inst = adc_read.Va;
//	instVal.Vb_inst = adc_read.Vb;
//	instVal.Vc_inst = adc_read.Vc;
//
//	rms_calculations.Va = adc_read.Va;
//	rms_calculations.Vb = adc_read.Vb;
//	rms_calculations.Vc = adc_read.Vc;
//
//	grid.Ia = adc_read.ia;
//	grid.Ib = adc_read.ib;
//	grid.Ic = adc_read.ic;
//
//	instVal.Ia_inst = adc_read.ia;
//	instVal.Ib_inst = adc_read.ib;
//	instVal.Ic_inst = adc_read.ic;
//
//	rms_calculations.Ia = adc_read.ia;
//	rms_calculations.Ib = adc_read.ib;
//	rms_calculations.Ic = adc_read.ic;
//
//	instVal.Vdc_inst = adc_read.Vdc;
//	instVal.Idc_inst = adc_read.Idc;
//
//	avg_calculations.Vdc = adc_read.Vdc;
//	avg_calculations.Idc = adc_read.Idc;

}



