#include "variables.h"


void usart_communication(void)
{

	if(usart_comm.transmission_enable == 0)
	{
		if(usart_comm.array_population_index <= TXN_ARRAY_SIZE)
		{
			if (usart_comm.array_population_index == 0 )
			{
				txn_array[usart_comm.array_population_index] = 0.0f;
				usart_comm.array_population_index++;
			}
			else if (usart_comm.array_population_index > 0 && usart_comm.array_population_index < TXN_ARRAY_SIZE-1)
			{
				usart_comm.adc_value = adc_buffer[1];
				usart_comm.adc_scaled_value = 102.36f * usart_comm.adc_value;
				txn_array[usart_comm.array_population_index] = usart_comm.adc_scaled_value;
				usart_comm.array_population_index++;
			}
			else if(usart_comm.array_population_index == TXN_ARRAY_SIZE-1)
			{
				txn_array[usart_comm.array_population_index] = 2.0f;
				usart_comm.array_population_index = usart_comm.array_population_index+2;
			}

		}
		else
		{
			usart_comm.convert_txn_array_to_base64 = 1;
			usart_comm.transmission_enable         = 1;
			uart_send_data_pointer                 = (uint8_t *)base64_txn;
			usart_comm.array_population_index      = 0;
			usart_comm.array_transmission_index    = 0;
		}
	}



	if(usart_comm.transmission_enable == 1 && usart_comm.convert_txn_array_to_base64 == 0)
	{
		if (usart_comm.array_transmission_index <= (BASE64_SIZE))
		{
			USART2->DR = *uart_send_data_pointer;
			uart_send_data_pointer++;
			usart_comm.array_transmission_index++;
		}
		else
		{
			usart_comm.transmission_enable = 0;
			usart_comm.array_transmission_index = 0;
			usart_comm.array_population_index = 0;

		}

	}

}



void convert_txn_array_to_base64(void) {

	if (usart_comm.convert_txn_array_to_base64 == 1)
	{
		*(DATA_TYPE*) &base64_txn[0] = txn_array[0];                              // Copy the first value as header
		base64_variables.output_index = SIZE_OF_DATA_TYPE ;                       // 1st value occupy space upto the size of data_type

		for (uint16_t i = 0; i < base64_variables.input_len; i += 3)
		{
			base64_variables.octet_a = data[i];
			base64_variables.octet_b = (i + 1 < base64_variables.input_len) ? data[i + 1] : 0;
			base64_variables.octet_c = (i + 2 < base64_variables.input_len) ? data[i + 2] : 0;
			base64_variables.triple  =
					(base64_variables.octet_a << 16) | (base64_variables.octet_b << 8) | base64_variables.octet_c;

			base64_txn[base64_variables.output_index++] = base64_chars[(base64_variables.triple >> 18) & 0x3F];
			base64_txn[base64_variables.output_index++] = base64_chars[(base64_variables.triple >> 12) & 0x3F];

			base64_txn[base64_variables.output_index++] =
					(i + 1 < base64_variables.input_len) ? base64_chars[(base64_variables.triple >> 6) & 0x3F] : '=';

			base64_txn[base64_variables.output_index++] =
					(i + 2 < base64_variables.input_len) ? base64_chars[base64_variables.triple & 0x3F] : '=';
		}

		*(DATA_TYPE*) &base64_txn[base64_variables.output_index] = txn_array[TXN_ARRAY_SIZE-1];    // Copy the last value as header
		base64_variables.output_index += SIZE_OF_DATA_TYPE ;
		usart_comm.convert_txn_array_to_base64 = 0;
	}

}



