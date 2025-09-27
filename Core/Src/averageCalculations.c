#include "math.h"
#include "variables.h"


void average_calculation_to_be_placed_in_interrupt(void)
{
	avg_calculations.theta      = avg_calculations.theta_prev - pll.Theta;
	avg_calculations.theta_prev = pll.Theta;

		if((avg_calculations.theta > 4) && (avg_calculations.busy_flag == 0))
		{

			final_sum.Vdc_sum = avg_calculations.Vdc_sum;
			final_sum.Idc_sum = avg_calculations.Idc_sum;

			avg_calculations.final_N = avg_calculations.n;
			avg_calculations.request = 1;
			avg_calculations.busy_flag = 1;
			avg_calculations.Vdc_sum = 0;
			avg_calculations.Idc_sum = 0;
			avg_calculations.n = 0;
		}

		avg_calculations.Vdc_sum = avg_calculations.Vdc_sum + avg_calculations.Vdc;
		avg_calculations.Idc_sum = avg_calculations.Idc_sum + avg_calculations.Idc;

		avg_calculations.n = avg_calculations.n+1;
}

void avg_calculation_to_be_placed_in_while_loop(void)
{
	if(avg_calculations.request == 1)
	{
		if(avg_calculations.final_N == 0)
		{
			avg_calculations.final_N = 1;
		}
	    avgVal.Vdc_avg = final_sum.Vdc_sum/avg_calculations.final_N;
	    avgVal.Idc_avg = final_sum.Idc_sum/avg_calculations.final_N;

	    avg_calculations.request = 0;
	    avg_calculations.busy_flag = 0;
	}

}
