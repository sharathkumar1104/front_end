#include "math.h"
#include "variables.h"
#include <stdbool.h>


void voltageController(void){

	// Voltage error
	voltage_ctrl.error = voltage_ctrl.Vdc_ref - voltage_ctrl.Vdc_actual;

	// Proportional term
	voltage_ctrl.P_term = voltage_ctrl.Kp * voltage_ctrl.error;

    // Compute integral term candidate
	voltage_ctrl.I_term_candidate = voltage_ctrl.Ki * (voltage_ctrl.integral_error + voltage_ctrl.error * pll.Ts);

	// Clamp the integral term to prevent output from exceeding ±Id_ref_max
//	if (voltage.I_term_candidate > (voltage.Id_ref_max - voltage.P_term)) {
//		voltage.I_term_clamped = voltage.Id_ref_max - voltage.P_term;
//	 } else if (voltage.I_term_candidate < (-(voltage.Id_ref_max) - voltage.P_term)) {
//		voltage.I_term_clamped = -(voltage.Id_ref_max) - voltage.P_term;
//	 } else {
//		voltage.I_term_clamped = voltage.I_term_candidate;
//	 }

	if(voltage_ctrl.I_term_candidate > 10){
		voltage_ctrl.I_term_candidate = 10;
	}else if(voltage_ctrl.I_term_candidate < -10){
		voltage_ctrl.I_term_candidate = -10;
	}


	// update integral state from clamped I_term
	voltage_ctrl.integral_error = voltage_ctrl.I_term_candidate / voltage_ctrl.Ki;


	// Final PI controller output
    voltage_ctrl.Id_ref = voltage_ctrl.P_term + voltage_ctrl.I_term_candidate;

	// Final output safety clamp (should already be within range)
	if (voltage_ctrl.Id_ref > voltage_ctrl.Id_ref_max) {
		voltage_ctrl.Id_ref = voltage_ctrl.Id_ref_max;
	 } else if (voltage_ctrl.Id_ref < -(voltage_ctrl.Id_ref_max)) {
	    voltage_ctrl.Id_ref = -(voltage_ctrl.Id_ref_max);
	 }

	 //   return Id_ref;
}




