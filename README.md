# MS51_ADC_PWM

update @ 2020/01/06

update MS51 driver to Keil_V1.00.003 , by test in MS51 TSSOP20 EVM

Add 2 define : ENABLE_ADC_MMA , ENABLE_ADC_DROP_AVG

- ENABLE_ADC_MMA : original adc convert data , with modified moving average algorithm 

- ENABLE_ADC_DROP_AVG : drop the first few adc convert data , and start average , in a loop

Simply the driver to main.c , reduce code size (HIRC , TIMER0 , UART , and print message)

Revies adc_ref_voltage and adc_convert_target , to calculate dynamically , instead of using DEFINE

----------------------------------------

update @ 2019/09/03

Add 2 function : ADC_To_Duty , ADC_To_Voltage

Fix get data sequence : convert adc_data at ADC_ISR , and use at ADC_ModifiedMovingAverage

----------------------------------------

update @ 2019/06/03

Add 2 define (ENABLE_LED_DIMMING_WITH_PWM , ENABLE_CONVERT_ADC_TO_DUTY_DEMO) for demo purpose

- ENABLE_LED_DIMMING_WITH_PWM : original demo , LED dimming with PWM function

- ENABLE_CONVERT_ADC_TO_DUTY_DEMO : add ADC convert to PWM duty , check CUSTOM_INPUT_VOLT_MIN for customize define lower limit voltage

- ADC reference voltage calculate dynamically (follow on MCU voltage)

----------------------------------------

update @ 2019/04/30

Add ADC (interrupt trigger) sample code for MS51 (P0.4)

- Use Modified Moving Average algorithm to calculate smooth ADC data

- Get ADC internal voltage (band-gap voltage) to calculate actual input voltage

- Convert ADC data per 20ms

Add LED dimming with PWM function (P1.2)

- increase duty per 18 ms ,

- when reach 100 % duty , change to decrase duty per 18ms

- when reach 0 % duty , change to increase duty again

Add PWM with fix duty function (P1.1)

Add GPIO Toggle by timer (500ms)(P0.5)