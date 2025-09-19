#include "variables.h"
#include <stdbool.h>

void check_if_PLL_locked(void){

	if(pll.Vq_out_filtered > -10.0f && pll.Vq_out_filtered < 10.0f)
	{
		pll.lockCount++;
		if(pll.lockCount >= pll.treshold_time)
		{
			pll.lockCount = pll.treshold_time;
			error_flags.pll_lock_status = 0;
		}

	}
	else
	{
	     pll.lockCount = 0;
		 error_flags.pll_lock_status = 1;
	}
}
