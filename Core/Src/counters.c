#include "variables.h"

void Counters(void){

	/**** counter of 10 seconds for relay ****/
	if(relayOff_counter_enable == 1)
	{
		counters.relayOffDelay++;

		if(counters.relayOffDelay >= 10000)
		{
			//relayOff()--->GPIO off
			counters.relayOffDelay = 0;
			relayOff_counter_enable = 0;
		}
	}
	else
	{
		counters.relayOffDelay = 0;
	}
}




void inverterOff(void){
	//pulses_off()
}
