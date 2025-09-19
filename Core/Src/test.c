#include "variables.h"
#include "math.h"

void test_abc_to_dq(void){
	grid.Ia = 0.0f;
	grid.Ib = 0.0f;
	grid.Ic = 0.0f;

	pll.Theta = 1.5708f;

	abc_to_dq();


//	Ia = 10, Ib = -5, Ic = -5
//	V_alpha ≈ 10
//	V_beta ≈ 0
//	Id ≈ 10
//	Iq ≈ 0


//	Ia = 10, Ib = -5, Ic = -5
//	V_alpha ≈ 10
//	V_beta ≈ 0
//	Id ≈ 0
//	Iq ≈ -10
//	Theta = 1.5708 //90 degrees


//	Ia = Ib = Ic = 0
//	→ V_alpha, V_beta, Id, Iq should all be 0

}


void test_voltageController(void){
	  tick_counter++;
	  if (tick_counter < 20000) {
	    voltage_ctrl.Vdc_actual = 650.0f;
	   } else if (tick_counter < 40000) {
	    voltage_ctrl.Vdc_actual = 700.0f;
	   } else if (tick_counter < 60000) {
	    voltage_ctrl.Vdc_actual = 750.0f;
	   } else {
	    tick_counter = 0;
	    }

	   // Run controller
	   voltageController();
}


//void test_voltageController(void) {
//    voltage.Vdc_actual = 700.0f;       // Measured DC Link Voltage
//    voltage.Kp = 0.05f;                // Example proportional gain
//    voltage.Ki = 1.0f;                 // Example integral gain
//    voltage.integral_error = 0.0f;     // Initial integral state
//    voltage.Id_ref_max = 12.5f;        // Max current setpoint
//
//    voltageController();
//}

void test_currentControlTriggered(void) {
    // Set test input values
    voltage_ctrl.Id_ref = 10.0f;
    current_ctrl.Iq_ref = 1.0f;
    grid.Id = 1.5f;
    grid.Iq = 0.5f;
    grid.Vd = 299.0f;
    pll.Vq_out_filtered = 0.1f;

    current_ctrl.Vlimit = 10.0f;


    // Reset integrators before starting test
    current_ctrl.int_Id = 0.0f;
    current_ctrl.int_Iq = 0.0f;

    // Call the controller function
    currentControlTriggered();

}

void test_dq_to_abc(void){
	pll.Theta = 0.0f;
	current_ctrl.Vd_out = 399.0f;
	current_ctrl.Vq_out = 0.1f;
	voltage_ctrl.Vdc_actual = 700.0f;
	for(int i=0; i<5; i++){
		dq_to_abc();
		pll.Theta += (float)M_PI/6.0f;
	}
}

void test_dq_to_pwm(void){
	current_ctrl.Vd_out = 400.0f;
	current_ctrl.Vq_out = 0.1f;
	for(int i=0 ; i<6 ; i++){
		pll.Theta += (float)M_PI/6.0f;
		dq_to_abc();
	}
}



//void test_currentControlTriggered(void){
//
//    current.Vlimit = 200.0f;
//    grid.Vd = 300.0f;
//    pll.Vq_out_filtered = 0.0f;
//
//
//	tick_counter++;
//	if(tick_counter < 20000){
//		voltage.Id_ref = 10.0f;
//		grid.Id = 5.0f;
//	}else if(tick_counter < 40000){
//		voltage.Id_ref = 0.0f;
//		grid.Id = 0.0f;
//	}else if(tick_counter < 60000){
//		voltage.Id_ref = -10.0f;
//		grid.Id = -5.0f;
//	}else {
//        tick_counter = 0;              // Reset test cycle
//    }
//
//	currentControlTriggered();
//}




