#include "math.h"
#include "variables.h"

void dq_to_abc(void){

	 // Park inverse transform
	 grid.V_alpha = current_ctrl.Vd_out * cosf(pll.Theta) - current_ctrl.Vq_out * sinf(pll.Theta);
	 grid.V_beta  = current_ctrl.Vd_out * sinf(pll.Theta) + current_ctrl.Vq_out * cosf(pll.Theta);

	 // Clarke inverse transform
	 abc_form.Va = grid.V_alpha;
	 abc_form.Vb = -0.5f * grid.V_alpha + 0.8660254f * grid.V_beta;   // √3/2 ≈ 0.8660254
	 abc_form.Vc = -0.5f * grid.V_alpha - 0.8660254f * grid.V_beta;

	 // Normalize with respect to Vdc/2 → modulation index
	 abc_form.m_a = abc_form.Va / (voltage_ctrl.Vdc_actual / 2.0f);
	 abc_form.m_b = abc_form.Vb / (voltage_ctrl.Vdc_actual / 2.0f);
	 abc_form.m_c = abc_form.Vc / (voltage_ctrl.Vdc_actual / 2.0f);

	 if(abc_form.m_a > 1.0f){
		abc_form.m_a = 1.0f;
	 }else if (abc_form.m_a < -1.0f){
	    abc_form.m_a = -1.0f;
	 }

	 if(abc_form.m_b > 1.0f){
		abc_form.m_b = 1.0f;
	 }else if(abc_form.m_b < -1.0f){
		 abc_form.m_b = -1.0f;
	 }

	  if (abc_form.m_c > 1.0f){
		  abc_form.m_c = 1.0f;
	  } else if (abc_form.m_c < -1.0f){
	    	abc_form.m_c = -1.0f;
	  }

//	  abc_form.m_out_a = 650.0f + 624.0f * abc_form.m_a;
//	  abc_form.m_out_b= 650.0f + 624.0f * abc_form.m_b;
//	  abc_form.m_out_c= 650.0f + 624.0f * abc_form.m_c;

	  // Normalize with +ve values
	  abc_form.m_out_a = 525.0f + 524.0f * abc_form.m_a; //1049
	  abc_form.m_out_b = 525.0f + 524.0f * abc_form.m_b;
	  abc_form.m_out_c = 525.0f + 524.0f * abc_form.m_c;


	  // Update PWM outputs
	  TIM8->CCR1 = (uint16_t)abc_form.m_out_a;
	  TIM8->CCR2 = (uint16_t)abc_form.m_out_b;
	  TIM8->CCR3 = (uint16_t)abc_form.m_out_c;

}
