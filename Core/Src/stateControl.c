#include "variables.h"


/***********************************************************************/
// State Control Function
/***********************************************************************/
void stateControl(void)
{
	switch(state)
	{
	    case state0 :
	    {
	    	state0_Control_Code();
	    	break;
	    }
	    case state1 :
	    {

	    	break;
	    }
	    case state2:
	    {

	    	break;
	    }
	    default :
	    {

	    	break;
	    }
	}
}



// State0 Control Code
void state0_Control_Code(void){
	if(first_time_state_entry == 1)
	{

		//clears the flags------>
		counters.state_transistion_timer = 0;
		first_time_state_entry  = 0;
	}
	if(RMS_GRID_VOLT_WITHIN_LIMITS)
	{
		error_flags.rms_grid_voltage  = 0;
		error_flags.inst_grid_voltage = 0;
	}
	if(RMS_GRID_CURRENT_WITHIN_LIMITS)
	{
		error_flags.rms_grid_current  = 0;
		error_flags.inst_grid_current = 0;
	}
	if(AVERAGE_DC_VOLT_WITHIN_LIMITS)
	{
		error_flags.Vdc_avg  = 0;
		error_flags.Vdc_inst = 0;
	}
	if(AVERAGE_DC_CURRENT_WITHIN_LIMITS)
	{
		error_flags.Idc_avg  = 0;
		error_flags.Idc_inst = 0;
	}

	if(error_flags.rms_grid_voltage == 0 && error_flags.Vdc_avg == 0 &&
	   error_flags.inst_grid_voltage== 0 && error_flags.Vdc_inst == 0 &&
	   error_flags.rms_grid_current == 0 && error_flags.Idc_avg == 0 &&
	   error_flags.inst_grid_current ==0 && error_flags.Idc_inst == 0 &&
	   error_flags.pll_lock_status == 0 )
	{
		counters.state_transistion_timer++;
		if(counters.state_transistion_timer >= 100000)
		{
			state = state1;
			first_time_state_entry = 1;
		}
	}
	else
	{
		counters.state_transistion_timer = 0;
	}
}



// State1 Control Code
void state1_Control_Code(void){
	if(first_time_state_entry == 1)
	{

//		relayOn------>

		counters.state1_timer++;

		if(counters.state1_timer++ >= 50000)
		{
			first_time_state_entry  = 0;
			counters.state1_timer   = 0;
		}
	}
	else
	{

//		voltageController();
//		currentControlTriggered();
//		dq_to_abc();

	}
}



