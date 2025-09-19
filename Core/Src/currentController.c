#include "math.h"
#include "variables.h"


void currentControlTriggered(void){

	// --- Current Errors ---
	current_ctrl.err_Id = voltage_ctrl.Id_ref - grid.Id;
	current_ctrl.err_Iq = current_ctrl.Iq_ref - grid.Iq;

	// --- Integrate Error ---
	current_ctrl.int_Id = current_ctrl.int_Id + current_ctrl.Ki * pll.Ts * current_ctrl.err_Id;
	current_ctrl.int_Iq = current_ctrl.int_Iq + current_ctrl.Ki * pll.Ts * current_ctrl.err_Iq;

	// --- Clamp Integrators using Vlimit ---
	if (current_ctrl.int_Id > current_ctrl.Vlimit){
		current_ctrl.int_Id = current_ctrl.Vlimit;
	}
	if (current_ctrl.int_Id < -current_ctrl.Vlimit){
		current_ctrl.int_Id = -current_ctrl.Vlimit;
	}
	if (current_ctrl.int_Iq > current_ctrl.Vlimit){
		current_ctrl.int_Iq = current_ctrl.Vlimit;
	}
	if (current_ctrl.int_Iq < -current_ctrl.Vlimit){
		current_ctrl.int_Iq = -current_ctrl.Vlimit;
	}

	// --- PI Controller Output (delta voltage) ---
	current_ctrl.deltaV_d = current_ctrl.Kp * current_ctrl.err_Id + current_ctrl.int_Id;
	current_ctrl.deltaV_q = current_ctrl.Kp * current_ctrl.err_Iq + current_ctrl.int_Iq;

    // Clamp controller output
    if (current_ctrl.deltaV_d > current_ctrl.Vlimit){
    	current_ctrl.deltaV_d = current_ctrl.Vlimit;
    }
    if (current_ctrl.deltaV_d < -current_ctrl.Vlimit){
    	current_ctrl.deltaV_d = -current_ctrl.Vlimit;
    }
    if (current_ctrl.deltaV_q > current_ctrl.Vlimit){
    	current_ctrl.deltaV_q = current_ctrl.Vlimit;
    }
    if (current_ctrl.deltaV_q < -current_ctrl.Vlimit){
    	current_ctrl.deltaV_q = -current_ctrl.Vlimit;
    }

    // --- Final Output: Feedforward + PI Output ---
    current_ctrl.Vd_ff = grid.Vd;
    current_ctrl.Vq_ff = pll.Vq_out_filtered;
    current_ctrl.Vd_out = current_ctrl.Vd_ff + current_ctrl.deltaV_d;
    current_ctrl.Vq_out = current_ctrl.Vq_ff + current_ctrl.deltaV_q;


}
