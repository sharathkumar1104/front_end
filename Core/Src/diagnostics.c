#include "variables.h"
#include <stdbool.h>


// Instantaneous Diagnostics
void instantaneousDiagnostics(void)
{
	if(INST_GRID_VOLTAGE_OUT_OF_LIMITS)
	{
		error_flags.inst_grid_voltage = 1;
		inverterOff();
		relayOff_counter_enable = 1;
		state = state0;
		first_time_state_entry = 1;

	}
	if(INST_GRID_CURRENT_OUT_OF_LIMITS)
	{
		error_flags.inst_grid_voltage = 1;
		inverterOff();
		relayOff_counter_enable = 1;
		state = state0;
		first_time_state_entry = 1;
	}
	if(INST_DC_VOLT_OUT_OF_LIMITS)
	{
		error_flags.Vdc_inst = 1;
		inverterOff();
		relayOff_counter_enable = 1;
		state = state0;
		first_time_state_entry = 1;
	}
	if(INST_DC_CURRENT_OUT_OF_LIMITS)
	{
        error_flags.Idc_inst = 1;
		inverterOff();
		relayOff_counter_enable = 1;
		state = state0;
		first_time_state_entry = 1;
	}
}

// RMS Diagnostics
void rmsDiagnostics(void){
	if(RMS_GRID_VOLT_OUT_OF_LIMITS)
	{
		error_flags.rms_grid_voltage = 1;
		inverterOff();
		relayOff_counter_enable = 1;
		state = state0;
		first_time_state_entry = 1;
	}
	if(RMS_GRID_CURRENT_OUT_OF_LIMITS)
	{
		error_flags.rms_grid_current = 1;
		inverterOff();
		relayOff_counter_enable = 1;
		state = state0;
		first_time_state_entry = 1;
	}
}

// Average Diagnostics
void averageDiagnostics(void){
	if(AVERAGE_DC_VOLT_OUT_OF_LIMITS)
	{
		error_flags.Vdc_avg = 1;
		inverterOff();
		relayOff_counter_enable = 1;
		state = state0;
		first_time_state_entry = 1;
	}
	if(AVERAGE_DC_CURRENT_OUT_OF_LIMITS)
	{
		error_flags.Idc_avg = 1;
		inverterOff();
		relayOff_counter_enable = 1;
		state = state0;
		first_time_state_entry = 1;
	}
}



