#include "math.h"
#include "variables.h"


void rms_calculation_to_be_placed_in_interrupt(void)
{
	rms_calculations.theta      = rms_calculations.theta_prev - pll.Theta;
	rms_calculations.theta_prev = pll.Theta;

		if((rms_calculations.theta > 4) && (rms_calculations.busy_flag == 0))
		{

			final_squared_sum.Ia_sum = rms_calculations.Ia_sum;
			final_squared_sum.Ib_sum = rms_calculations.Ib_sum;
			final_squared_sum.Ic_sum = rms_calculations.Ic_sum;
			final_squared_sum.Va_sum = rms_calculations.Va_sum;
			final_squared_sum.Vb_sum = rms_calculations.Vb_sum;
			final_squared_sum.Vc_sum = rms_calculations.Vc_sum;

			rms_calculations.final_N   = rms_calculations.n;
			rms_calculations.request   = 1;
			rms_calculations.busy_flag = 1;

			rms_calculations.Ia_sum = 0;
			rms_calculations.Ib_sum = 0;
			rms_calculations.Ic_sum = 0;
			rms_calculations.Va_sum = 0;
			rms_calculations.Vb_sum = 0;
			rms_calculations.Vc_sum = 0;

			rms_calculations.n = 0;
		}

		rms_calculations.Ia_sum = rms_calculations.Ia_sum + (rms_calculations.Ia * rms_calculations.Ia);
		rms_calculations.Ib_sum = rms_calculations.Ib_sum + (rms_calculations.Ib * rms_calculations.Ib);
		rms_calculations.Ic_sum = rms_calculations.Ic_sum + (rms_calculations.Ic * rms_calculations.Ic);

		rms_calculations.Va_sum = rms_calculations.Va_sum + (rms_calculations.Va * rms_calculations.Va);
		rms_calculations.Vb_sum = rms_calculations.Vb_sum + (rms_calculations.Vb * rms_calculations.Vb);
		rms_calculations.Vc_sum = rms_calculations.Vc_sum + (rms_calculations.Vc * rms_calculations.Vc);

		rms_calculations.n = rms_calculations.n+1;
}

void rms_calculation_to_be_placed_in_while_loop(void)
{
	if(rms_calculations.request == 1)
	{
		if(rms_calculations.final_N == 0)
		{
			rms_calculations.final_N = 1;
		}
	    avg_of_mean_square.Ia = final_squared_sum.Ia_sum/rms_calculations.final_N;
	    avg_of_mean_square.Ib = final_squared_sum.Ib_sum/rms_calculations.final_N;
	    avg_of_mean_square.Ic = final_squared_sum.Ic_sum/rms_calculations.final_N;

	    avg_of_mean_square.Va = final_squared_sum.Va_sum/rms_calculations.final_N;
	    avg_of_mean_square.Vb = final_squared_sum.Vb_sum/rms_calculations.final_N;
	    avg_of_mean_square.Vc = final_squared_sum.Vc_sum/rms_calculations.final_N;

	    rmsVal.Ia_rms = sqrtf(avg_of_mean_square.Ia);
	    rmsVal.Ib_rms = sqrtf(avg_of_mean_square.Ib);
	    rmsVal.Ic_rms = sqrtf(avg_of_mean_square.Ic);

	    rmsVal.Va_rms = sqrtf(avg_of_mean_square.Va);
	    rmsVal.Vb_rms = sqrtf(avg_of_mean_square.Vb);
	    rmsVal.Vc_rms = sqrtf(avg_of_mean_square.Vc);

		rms_calculations.request   = 0;
		rms_calculations.busy_flag = 0;
	}

}


