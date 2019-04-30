# MS51_ADC_PWM

update @ 2019/04/30

Add ADC (interrupt trigger) sample code for MS51 (P0.4)

- Use Modified Moving Average algorithm to calculate smooth ADC data

- Get ADC internal voltage (band-gap voltage) to calculate actual input voltage

Add LED dimming with PWM function (P1.2)

- increase duty per 18 ms ,

- when reach 100 % duty , change to decrase duty per 18ms

- when reach 0 % duty , change to increase duty again