#include "stm32f4xx.h"
#include <stdbool.h>

#ifndef VARIABLES_H
#define VARIABLES_H

/***********************************************************************/
//                          Definitions
/***********************************************************************/

#define ADC_LENGHT      8                        // 8 samples
#define DELAY_10_SEC    100000                   // 10s at 100us handler

// USART COMMUNICATION
#define DATA_TYPE  uint8_t
#define SIZE_OF_DATA_TYPE sizeof(DATA_TYPE)
#define TXN_ARRAY_SIZE  ((200/SIZE_OF_DATA_TYPE) + (SIZE_OF_DATA_TYPE * 2))
#define BASE64_SIZE  ((SIZE_OF_DATA_TYPE * 2) + (((TXN_ARRAY_SIZE - 2) * SIZE_OF_DATA_TYPE + 2) / 3 * 4))
/*------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------------------------------------------*/

#define INST_GRID_VOLTAGE_OUT_OF_LIMITS        (instVal.Va_inst > UCoffLimit.Va_inst || instVal.Va_inst < LCoffLimit.Va_inst ||     \
	                                            instVal.Vb_inst > UCoffLimit.Vb_inst || instVal.Vb_inst < LCoffLimit.Vb_inst ||     \
	                                            instVal.Vc_inst > UCoffLimit.Vc_inst || instVal.Vc_inst < LCoffLimit.Vc_inst)

#define INST_GRID_CURRENT_OUT_OF_LIMITS         (instVal.Ia_inst > UCoffLimit.Ia_inst || instVal.Ia_inst < LCoffLimit.Ia_inst ||    \
			                                     instVal.Ib_inst > UCoffLimit.Ib_inst || instVal.Ib_inst < LCoffLimit.Ib_inst ||    \
		                                     	 instVal.Ic_inst > UCoffLimit.Ic_inst || instVal.Ic_inst < LCoffLimit.Ic_inst)

#define INST_DC_VOLT_OUT_OF_LIMITS              (instVal.Vdc_inst > UCoffLimit.Vdc_inst || instVal.Vdc_inst < LCoffLimit.Vdc_inst)

#define INST_DC_CURRENT_OUT_OF_LIMITS           (instVal.Idc_inst > UCoffLimit.Idc_inst || instVal.Idc_inst < LCoffLimit.Idc_inst )


#define RMS_GRID_VOLT_OUT_OF_LIMITS             (rmsVal.Va_rms > UCoffLimit.Va_rms || rmsVal.Va_rms < LCoffLimit.Va_rms ||       \
	                                             rmsVal.Vb_rms > UCoffLimit.Vb_rms || rmsVal.Vb_rms < LCoffLimit.Vb_rms ||       \
	                                             rmsVal.Vc_rms > UCoffLimit.Vc_rms || rmsVal.Vc_rms < LCoffLimit.Vc_rms )

#define RMS_GRID_CURRENT_OUT_OF_LIMITS           (rmsVal.Ia_rms > UCoffLimit.Ia_rms || rmsVal.Ia_rms < LCoffLimit.Ia_rms ||      \
			                                      rmsVal.Ib_rms > UCoffLimit.Ib_rms || rmsVal.Ib_rms < LCoffLimit.Ib_rms ||      \
	                                              rmsVal.Ic_rms > UCoffLimit.Ic_rms || rmsVal.Ic_rms < LCoffLimit.Ic_rms )

#define RMS_GRID_VOLT_WITHIN_LIMITS              ((rmsVal.Va_rms < UCinLimit.Va_rms && rmsVal.Va_rms > LCinLimit.Va_rms) &&      \
	                                              (rmsVal.Vb_rms < UCinLimit.Vb_rms && rmsVal.Vb_rms > LCinLimit.Vb_rms) &&      \
	                                              (rmsVal.Vc_rms < UCinLimit.Vc_rms && rmsVal.Vc_rms > LCinLimit.Vc_rms))

#define RMS_GRID_CURRENT_WITHIN_LIMITS           ((rmsVal.Ia_rms < UCinLimit.Ia_rms && rmsVal.Ia_rms > LCinLimit.Ia_rms) &&      \
	                                              (rmsVal.Ib_rms < UCinLimit.Ib_rms && rmsVal.Ib_rms > LCinLimit.Ib_rms) &&      \
	                                              (rmsVal.Ic_rms < UCinLimit.Ic_rms && rmsVal.Ic_rms > LCinLimit.Ic_rms) )

#define AVERAGE_DC_VOLT_OUT_OF_LIMITS             (avgVal.Vdc_avg > UCoffLimit.Vdc_avg || avgVal.Vdc_avg < LCoffLimit.Vdc_avg)

#define AVERAGE_DC_CURRENT_OUT_OF_LIMITS          (avgVal.Idc_avg > UCoffLimit.Idc_avg || avgVal.Idc_avg < LCoffLimit.Idc_avg )

#define AVERAGE_DC_VOLT_WITHIN_LIMITS             (avgVal.Vdc_avg < UCinLimit.Vdc_avg && avgVal.Vdc_avg > LCinLimit.Vdc_avg)

#define AVERAGE_DC_CURRENT_WITHIN_LIMITS          (avgVal.Idc_avg < UCinLimit.Idc_avg && avgVal.Idc_avg > LCinLimit.Idc_avg)



/***********************************************************************/
//                            Types
/***********************************************************************/

typedef enum {
    state0,     // off state
    state1,     // intermediate state
    state2      // on state
} StateControl;




/***********************************************************************/
//                          Structures
/***********************************************************************/

struct PLL_Variables
{
    float Ts;
    volatile float Theta;
    volatile float freq;
    volatile float Vq_out_filtered;
    volatile float theta_prev;
    volatile float integrator;
    volatile float Vq_prev;
    volatile float Vq_filtered;
    volatile float cos_theta;
    volatile float sin_theta;
    volatile float error;
    volatile float max_corr_rad;
    volatile float w_corr;
    volatile float w_nom;
    volatile float w;
    volatile float w_min;
    volatile float w_max;
    volatile float freq_ff;
    volatile float alpha;
    volatile float Kp;
    volatile float Ki;
    volatile float debugVariable1;
    volatile float debugVariable2;
    volatile int intialized;
    volatile int lockCount;
    volatile int treshold_time;

};

// Structure for V_grid Parameters
struct V_Grid_Variables
{
	volatile float Ia, Ib, Ic;
    volatile float Va, Vb, Vc;
    volatile float V_alpha, V_beta;
    volatile float Id, Iq;
    volatile float Vd, Vq;
};

// Structure for Voltage Control
struct Voltage_Controller_Variables
{
	float Vdc_ref;
	float Vdc_actual;
	float Id_ref_max;
	float Id_ref;
	float I_term_clamped;
	float integral_error;
	float I_term_candidate;
	float P_term;
	float error;
	float Kp;
	float Ki;
};

// Structure for Current Controlled Triggered
struct Current_Controller_Variables
{
	float L;
	float C;
	float Vd_ff;
	float Vq_ff;
	float Iq_ref;
	float Id;
	float Iq;
	float Vd_out;
	float Vq_out;
	float deltaV_d;
	float deltaV_q;
	float Vlimit;
	float err_Id;
	float err_Iq;
	float Kp;
    float Ki;
	volatile float int_Id;
	volatile float int_Iq;
};

// Structure for dq-to-abc
struct dq_to_abc
{
	float Va, Vb, Vc;
	float m_a, m_b, m_c;
	float m_out_a, m_out_b, m_out_c;
};


// Structure for Tim3 Values
struct Timer3_Variables
{
	volatile int sample_index;
	int Tim3_ARR_Value;
};


// Structure for Limit Values
struct Limit_Parameters
{
	float Va_rms;
	float Vb_rms;
	float Vc_rms;
	float Va_inst;
	float Vb_inst;
	float Vc_inst;
	float Ia_rms;
	float Ib_rms;
	float Ic_rms;
	float Ia_inst;
	float Ib_inst;
	float Ic_inst;
	float Vdc_avg;
	float Vdc_inst;
	float Idc_avg;
	float Idc_inst;
};

// Structure for Error Flags
struct ErrorFlags
{
	volatile bool inst_grid_voltage;
	volatile bool inst_grid_current;
	volatile bool rms_grid_voltage;
	volatile bool rms_grid_current;
	volatile bool Vdc_inst;
	volatile bool Idc_inst;
	volatile bool Vdc_avg;
	volatile bool Idc_avg;
	volatile bool pll_lock_status;
};

// Structure for RMS Values
struct RMS_Variables
{
	volatile float Va_rms;
	volatile float Vb_rms;
	volatile float Vc_rms;
	volatile float Ia_rms;
	volatile float Ib_rms;
	volatile float Ic_rms;
};

// Structure for Instantaneous Values
struct Instantaneous_Variables
{
	volatile float Va_inst;
	volatile float Vb_inst;
	volatile float Vc_inst;
	volatile float Ia_inst;
	volatile float Ib_inst;
	volatile float Ic_inst;
	volatile float Vdc_inst;
	volatile float Idc_inst;
};

// Structure for Average Values
struct Average_Variables
{
	volatile float Vdc_avg;
	volatile float Idc_avg;
};

// Structure for Counter Values
struct Counters
{
	volatile float relayOffDelay;
	volatile float state_transistion_timer;
	volatile float state1_timer;
};


// Structure for Rms Values
struct Rms_Calculations
{
	volatile float Ia, Ib, Ic;
	volatile float Va, Vb, Vc;
	volatile float Ia_sum;
	volatile float Ib_sum;
	volatile float Ic_sum;
	volatile float Va_sum;
	volatile float Vb_sum;
	volatile float Vc_sum;
	volatile float theta;
	volatile float theta_prev;
	volatile bool busy_flag;
	volatile bool request;
	volatile int n;
	volatile int final_N;
};

struct Avg_of_mean_square
{
	volatile float Ia, Ib, Ic;
	volatile float Va, Vb, Vc;
};

// Structure for Rms Values
struct Avg_Calculations
{
	volatile float Vdc, Idc;
	volatile float Vdc_sum, Idc_sum;
	volatile float theta;
    volatile float theta_prev;
	volatile bool busy_flag;
	volatile bool request;
	volatile int n;
	volatile int final_N;
};

struct Avg_Sum
{
	volatile float Vdc_sum;
	volatile float Idc_sum;
};

// Structure for Adc Readings
struct Adc_Readings
{
	float offSet;
	float voltage_gain_factor;
	float current_gain_factor;
	float Vdc_gain_factor;
	float Idc_gain_factor;
	int Vab_in, Vbc_in, Vca_in;
	int ia_in, ib_in, ic_in;
    float Vdc_in, idc_in;
    float Vab_corrected;
    float Vbc_corrected;
    float Vca_corrected;
    float ia_corrected;
    float ib_corrected;
    float ic_corrected;
    float Vdc_corrected;
    float Idc_corrected;
    float Vab_recalculated;
    float Vbc_recalculated;
    float Vca_recalculated;
    float ia_recalculated;
    float ib_recalculated;
    float ic_recalculated;
    float Vdc_recalculated;
    float Idc_recalculated;
    float Va, Vb, Vc, ia, ib, ic, Vdc, Idc;
};


// USART
struct USART_COMM
{
   volatile bool transmission_enable;
   volatile int array_population_index;
   volatile int array_transmission_index;
   volatile int adc_value;
   volatile bool start_of_transmission;
   volatile bool convert_txn_array_to_base64;
   volatile float adc_scaled_value;
};

// BASE64
struct base64
{
	uint16_t output_index;
	uint32_t octet_a;
	uint32_t octet_b;
	uint32_t octet_c;
	uint32_t triple;
	uint16_t input_len;
};

extern volatile uint8_t txn_buffer;
extern volatile DATA_TYPE txn_array[TXN_ARRAY_SIZE];
extern volatile uint8_t *uart_send_data_pointer;
extern volatile struct base64 base64_variables;
extern const uint8_t *data;
extern char base64_txn[BASE64_SIZE];

/***********************************************************************/
//                     Global Variables
/***********************************************************************/

// ADC
extern volatile struct Adc_Readings adc_read;
extern volatile uint16_t adc_buffer[ADC_LENGHT];

// COUNTERS
extern volatile struct Counters counters;
extern volatile bool relayOff_counter_enable;
extern volatile uint32_t delay_time;
extern volatile uint32_t tick_counter;

// STATE MACHINE
extern volatile StateControl state;
extern volatile uint32_t first_time_state_entry;

// LIMIT PARAMETERS
extern volatile struct Limit_Parameters UCoffLimit;
extern volatile struct Limit_Parameters UCinLimit;
extern volatile struct Limit_Parameters LCoffLimit;
extern volatile struct Limit_Parameters LCinLimit;

// FLAGS AND COUNTERS FOR RMS AND AVG CALCULATIONS
extern volatile struct Rms_Calculations rms_calculations;
extern volatile struct Rms_Calculations final_squared_sum;
extern volatile struct Avg_of_mean_square avg_of_mean_square;


extern volatile struct Avg_Calculations avg_calculations;
extern volatile struct Avg_Calculations final_sum;

// DIAGNOSTICS VARIABLES
extern volatile struct RMS_Variables rmsVal;
extern volatile struct Instantaneous_Variables instVal;
extern volatile struct Average_Variables avgVal;
extern volatile struct ErrorFlags error_flags;

// GRID AND PLL
extern volatile struct PLL_Variables pll;
extern volatile struct V_Grid_Variables grid;
extern const float va_array[];
extern const float vb_array[];
extern const float vc_array[];

extern const float ia_array[];
extern const float ib_array[];
extern const float ic_array[];

extern const float Vdc_array[];
extern const float idc_array[];

// CONTROL LOOPS
extern volatile struct Voltage_Controller_Variables voltage_ctrl;
extern volatile struct Current_Controller_Variables current_ctrl;
extern volatile struct dq_to_abc abc_form;

// TIMERS
extern volatile struct Timer3_Variables timer3_variables;

//
extern volatile struct USART_COMM usart_comm;
extern const char base64_chars[];


/***********************************************************************/
//                      FUNCTION PROTOTYPES
/***********************************************************************/

// Called inside interrupt of 100us
void Counters(void);

// Called inside interrupt of 100us
void adcReadings(void);

// Called inside interrupt of 100us
void PLL(void);                        // PLL routine
void abc_to_dq(void);                  // abc → dq transformation
void check_if_PLL_locked(void);        // PLL lock status check

// Called inside interrupt of 100us
void rms_calculation_to_be_placed_in_interrupt(void);
void average_calculation_to_be_placed_in_interrupt(void);

// called inside main_while_loop
void rms_calculation_to_be_placed_in_while_loop(void);
void avg_calculation_to_be_placed_in_while_loop(void);

void instantaneousDiagnostics(void);   // Calculate instantaneous values
void rmsDiagnostics(void);             // Calculate RMS values
void averageDiagnostics(void);         // Calculate averages
void stateControl(void);

// Should be placed in State Control Code
void inverterOff(void);
void state0_Control_Code(void);
void state1_Control_Code(void);

// Should be called inside State1
void voltageController(void);          // Voltage control loop
void currentControlTriggered(void);    // Current control routine
void dq_to_abc(void);                  // dq → abc transformation

void usart_communication(void);
void convert_txn_array_to_base64(void);

#endif
