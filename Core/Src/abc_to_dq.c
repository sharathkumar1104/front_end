#include "math.h"
#include "variables.h"

void abc_to_dq(void){

	// Clarke Transformation
	grid.V_alpha = (2.0f/3.0f)*(grid.Ia - 0.5f * grid.Ib - 0.5f * grid.Ic);
	grid.V_beta =  (2.0f/3.0f)*((sqrtf(3.0f)/2.0f)*(grid.Ib - grid.Ic));

	// Park Transformation
	pll.cos_theta = cosf(pll.Theta);
	pll.sin_theta = sinf(pll.Theta);

	grid.Id = grid.V_alpha * pll.cos_theta + grid.V_beta * pll.sin_theta;
	grid.Iq = -grid.V_alpha * pll.sin_theta + grid.V_beta * pll.cos_theta;

}

//void abc_to_dqV(void){
//	// Clarke and Park Transform in one block
//	// Va, Vb, Vc: Three-phase voltages
//	// Theta: Rotor or PLL angle in radians
//    // Clarke Transformation
//
//    grid.V_alpha = (2/3)*(grid.Va - 0.5*grid.Vb - 0.5*grid.Vc); // Valpha = (2/3)*(Va - 0.5*Vb - 0.5*Vc);
//	grid.V_beta = (2/3)*(sqrt(3)/2)*(grid.Vb - grid.Vc);  //	Vbeta  = (2/3)*(sqrt(3)/2)*(Vb - Vc);
//
//	// Park Transformation
//    pll.cos_theta = cosf(pll.Theta);
//	pll.sin_theta = sinf(pll.Theta);
//
//	grid.Vd =  grid.V_alpha * pll.cos_theta + grid.V_beta * pll.sin_theta;
//	grid.Vq = -grid.V_alpha * pll.sin_theta + grid.V_beta * pll.cos_theta;
//
//}

