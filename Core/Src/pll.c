#include <math.h>
#include "variables.h"

//function [theta, freq, Vq_out_filtered, Vd,Vq] = srf_pll(Va, Vb, Vc, Ts)
void PLL(void){

     //	SRF-PLL with feedforward frequency, anti-windup, low-pass filter
     //	All computations are in single precision
     // Clarke Transformation
     grid.V_alpha = grid.Va;
     grid.V_beta = (grid.Va + (2 * grid.Vb)) / sqrtf(3.0f);

     // Park Transformation using estimated theta
     // This transformation converts AC signals in stationary coordinates into DC-like signals in a rotating frame,

      pll.cos_theta = cosf(pll.Theta);
      pll.sin_theta = sinf(pll.Theta);

      grid.Vd =  grid.V_alpha * pll.cos_theta + grid.V_beta * pll.sin_theta;  // voltage along x axis
      grid.Vq = -grid.V_alpha * pll.sin_theta + grid.V_beta * pll.cos_theta;  // voltage along y axis

      // Low-pass filter on Vq (1st order IIR)
      // pll.alpha = 0.005f;  // LPF coefficient
      // Vq is filtered to reduce Ac ripples so to observe smooth phase dc error
      pll.Vq_filtered = pll.alpha * grid.Vq + (1.0f - pll.alpha) * pll.Vq_prev;
      pll.Vq_prev = pll.Vq_filtered;

      // Deadband (optional for noise suppression)
      pll.error = pll.Vq_filtered;
      if (fabsf(pll.error) < 1e-4f) {
          pll.error = 0.0f;
      }

      // PI Controller
      pll.integrator = pll.integrator + pll.error * pll.Ts;

      // Anti-windup limits
      pll.max_corr_rad = 2.0f *(float)M_PI * 5.0f;  // ±5 Hz in rad/s    maximum correction angular velocity
      pll.w_corr = pll.Kp * pll.error + pll.Ki * pll.integrator;   // angular frequency

      if(pll.w_corr > pll.max_corr_rad){
    	  pll.w_corr = pll.max_corr_rad;
      }else if(pll.w_corr < -(pll.max_corr_rad)){
          pll.w_corr = -(pll.max_corr_rad);
        }

      //float w_corr = min(max(w_corr, -max_corr_rad), max_corr_rad);
      // Limit integrator accordingly
      //integrator = min(max(integrator, -max_corr_rad / Ki), max_corr_rad / Ki);

      if(pll.integrator > pll.max_corr_rad / pll.Ki){
    	  pll.integrator = pll.max_corr_rad / pll.Ki;
      }else if(pll.integrator < -(pll.max_corr_rad) / pll.Ki){
    	  pll.integrator = - (pll.max_corr_rad) / pll.Ki;
      }

      // Total estimated frequency (with feedforward)   //should it be pll.freq_ff?
       pll.w_nom = 2.0f * (float)M_PI * pll.freq_ff; // rad/s
       pll.w = pll.w_nom + pll.w_corr;

      // Clamp final frequency to [45, 55] Hz in rad/s

       pll.w_min = 2.0f * (float)M_PI * 45.0f;
       pll.w_max = 2.0f * (float)M_PI * 55.0f;

       if(pll.w > pll.w_max){
    	   pll.w = pll.w_max;
       }else if(pll.w < pll.w_min){
    	   pll.w = pll.w_min;
       }

       // Integrate frequency to get theta
       // theta = mod(theta, 2.0f * (float)M_PI);  // wrap to [0, 2]

      pll.Theta = pll.theta_prev + pll.w * pll.Ts;
      if (pll.Theta >= 2.0f * M_PI)
    	  pll.Theta -= 2.0f * M_PI;
      else if (pll.Theta < 0.0f)
    	  pll.Theta += 2.0f * M_PI;

      // Update state
       pll.theta_prev = pll.Theta;

      // Outputs
       pll.freq = pll.w / (2.0f * (float)M_PI);  // Hz
       pll.Vq_out_filtered = pll.Vq_filtered;

}



