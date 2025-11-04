#include "variables.h"
#include "stm32f4xx.h"
#include "main.h"

void digitalReadings(void)
{
  digital_read.switch_in        = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_11);
  if(digital_read.switch_in == 1){

  }
  digital_read.inv_temp_fail    = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_12);
  digital_read.driver_supply_ok = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_13);
  digital_read.extra_digi_1     = LL_GPIO_IsInputPinSet(GPIOD, LL_GPIO_PIN_8);
  digital_read.extra_digi_2     = LL_GPIO_IsInputPinSet(GPIOD, LL_GPIO_PIN_9);
  digital_read.extra_digi_3     = LL_GPIO_IsInputPinSet(GPIOD, LL_GPIO_PIN_10);
  digital_read.extra_digi_4     = LL_GPIO_IsInputPinSet(GPIOD, LL_GPIO_PIN_11);
  digital_read.zc               = LL_GPIO_IsInputPinSet(GPIOD, LL_GPIO_PIN_12);
  // mains relay
  // fan on
}

