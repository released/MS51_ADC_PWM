/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2019 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/***********************************************************************************************************/
/* Website: http://www.nuvoton.com                                                                         */
/*  E-Mail : MicroC-8bit@nuvoton.com                                                                       */
/*  Date   : Jan/21/2019                                                                                   */
/***********************************************************************************************************/

/************************************************************************************************************/
/*  File Function: MS51 DEMO project                                                                        */
/************************************************************************************************************/

#include "MS51.h"

enum{
	ADC_CH0 = 0 ,
	ADC_CH1 ,
	ADC_CH2 ,
	ADC_CH3 ,	
	
	ADC_CH4 ,
	ADC_CH5 ,
	ADC_CH6 ,
	ADC_CH7 ,

	ADC_CH_DEFAULT	
}ADC_CH_TypeDef;

typedef enum
{
	ADC_DataState_AVERAGE = 0 ,
	ADC_DataState_MMA , 
	
	ADC_DataState_DEFAULT 	
}ADC_DataState_TypeDef;

#define SYS_CLOCK 								(24000000ul)
#define PWM_FREQ 								(15000ul)

#define LED_REVERSE(x)							(100-x)			// because lED in EVM schematic , need to reverse level

#define TIMER_LOG_MS							(1000ul)
#define ADC_SAMPLETIME_MS						(20ul)
#define GPIO_TOGGLE_MS							(500ul)

#define ADC_RESOLUTION							(4096ul)
#define ADC_REF_VOLTAGE							(3300ul)	//(float)(3.3f)

#define ADC_MAX_TARGET							(4095ul)	//(float)(2.612f)
#define ADC_MIN_TARGET							(0ul)	//(float)(0.423f)

#define DUTY_MAX								(100ul)
#define DUTY_MIN								(0ul)
#define ADC_CONVERT_TARGET						(float)(ADC_MIN_TARGET*ADC_RESOLUTION/ADC_REF_VOLTAGE) //81.92000 
//#define ADC_SUB_TARGET							(float)((ADC_MAX_TARGET-ADC_MIN_TARGET)/(DUTY_MAX-DUTY_MIN)*(ADC_RESOLUTION/ADC_REF_VOLTAGE))//5.60505 
//#define ADCInputV_Sub							(float)	((ADC_MAX_BRIGHT-ADC_MIN_BRIGHT)/(DUTY_MAX-DUTY_MIN)) //0.02737 

#define ADC_SAMPLE_COUNT 						(16ul)			// 8
#define ADC_SAMPLE_POWER 						(4ul)			//(5)	 	// 3	,// 2 ^ ?

#define ADC_DIGITAL_SCALE(void) 					(0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 
#define ADC_CALC_DATA_TO_VOLTAGE(DATA,VREF) 	((DATA) * (VREF) / ADC_DIGITAL_SCALE())

#define CUSTOM_INPUT_VOLT_MAX(VREF)			(VREF)			//(3300ul)
#define CUSTOM_INPUT_VOLT_MIN					(1300ul)

#define GET_ADC_BY_INPUT_VOLT_MAX(VREF)		((CUSTOM_INPUT_VOLT_MAX(VREF)*ADC_MAX_TARGET)/VREF)
#define GET_ADC_BY_INPUT_VOLT_MIN(VREF)		((CUSTOM_INPUT_VOLT_MIN*ADC_MAX_TARGET)/VREF)

#define CALC_DATA_ADC_TO_DUTY(ADC,VREF)		(((ADC-GET_ADC_BY_INPUT_VOLT_MIN(VREF))*(DUTY_MAX-DUTY_MIN))/(GET_ADC_BY_INPUT_VOLT_MAX(VREF)-GET_ADC_BY_INPUT_VOLT_MIN(VREF)))
#define CALC_DATA_DUTY_TO_ADC(DUTY,VREF)		((GET_ADC_BY_INPUT_VOLT_MIN(VREF)+DUTY*(GET_ADC_BY_INPUT_VOLT_MAX(VREF)-GET_ADC_BY_INPUT_VOLT_MIN(VREF)))/(DUTY_MAX-DUTY_MIN))

//#define ENABLE_LED_DIMMING_WITH_PWM
#define ENABLE_CONVERT_ADC_TO_DUTY_DEMO

uint8_t 	u8TH0_Tmp = 0;
uint8_t 	u8TL0_Tmp = 0;

uint8_t 	DUTY_LED = 0;
uint8_t 	FLAG_LED = 1;

double  Bandgap_Voltage,AVdd,Bandgap_Value;      //please always use "double" mode for this
unsigned char xdata ADCdataVBGH, ADCdataVBGL;

uint16_t movingAverage_Target = 0;
uint32_t movingAverageSum_Target = 0;
uint8_t ADCDataReady = 0;

ADC_DataState_TypeDef ADCDataState = ADC_DataState_DEFAULT;


void SendString(uint8_t* Data)
{
	#if 1
	uint16_t i = 0;

	while (Data[i] != '\0')
	{
		UART_Send_Data(UART0,Data[i++]);		
	}

	#endif

	#if 0
	uint16_t i = 0;
	
	for(i = 0;i< (strlen(Data)) ;i++ )
	{
		UART_Send_Data(UART0,Data[i]);
	}
	#endif

	#if 0
    while(*Data)  
    {  
        UART_Send_Data(UART0, (unsigned char) *Data++);  
    } 
	#endif
}


void GPIO_Toggle(void)
{
    static uint8_t flag = 1;

	if (flag)
	{
		P05 = 1;
		flag = 0;
	}
	else
	{
		P05 = 0;
		flag = 1;
	}
}

void GPIO_Init(void)
{
    P05_PUSHPULL_MODE;
}

void PWM0_CH1_SetDuty(uint16_t d)
{
	uint16_t res = 0 ;
	res = d*(MAKEWORD(PWMPH,PWMPL)+1)/100;

    PWM1H = HIBYTE(res);
    PWM1L = LOBYTE(res);

    set_PWMCON0_LOAD;
    set_PWMCON0_PWMRUN;	
}

void PWM0_CH0_SetDuty(uint16_t d)
{
	uint16_t res = 0 ;
	res = d*(MAKEWORD(PWMPH,PWMPL)+1)/100;

    PWM0H = HIBYTE(res);
    PWM0L = LOBYTE(res);

    set_PWMCON0_LOAD;
    set_PWMCON0_PWMRUN;	
}

void PWM0_CHx_Init(uint16_t uFrequency)
{
	P11_PUSHPULL_MODE;	//Add this to enhance MOS output capability
    PWM1_P11_OUTPUT_ENABLE;	

	P12_PUSHPULL_MODE;	//Add this to enhance MOS output capability
    PWM0_P12_OUTPUT_ENABLE;
  
    PWM_IMDEPENDENT_MODE;
    PWM_CLOCK_DIV_16;

/*
	PWM frequency   = Fpwm/((PWMPH,PWMPL)+1) = (24MHz/2)/(PWMPH,PWMPL)+1) = 20KHz
*/	
    PWMPH = HIBYTE((SYS_CLOCK>>4)/uFrequency-1);
    PWMPL = LOBYTE((SYS_CLOCK>>4)/uFrequency-1);

	printf("\r\nPWM:0x%x  ,0x%x\r\n\r\n" , PWMPH,PWMPL);
	
	PWM0_CH0_SetDuty(LED_REVERSE(0));	

	PWM0_CH1_SetDuty(0);
}

void PWM0_LED_DIMMING(void)
{
	PWM0_CH0_SetDuty(LED_REVERSE(DUTY_LED));
//	printf("DUTY:%d\r\n" ,DUTY_LED );
	if (FLAG_LED)
	{
		if ( ++DUTY_LED == 100)
		{
			FLAG_LED = 0;
			DUTY_LED = 100;
		}
	}
	else
	{
		if ( --DUTY_LED == 0)
		{
			FLAG_LED = 1;
			DUTY_LED = 0;
		}			
	}
}

void ADC_ReadAVdd(void)
{
    UINT8 BandgapHigh,BandgapLow,BandgapMark;
    double bgvalue;

/*Read bandgap value */	
    set_CHPCON_IAPEN;
    IAPCN = READ_UID;
    IAPAL = 0x0d;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    BandgapLow = IAPFD;
    BandgapMark = BandgapLow&0xF0;
    BandgapLow = BandgapLow&0x0F;
    IAPAL = 0x0C;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    BandgapHigh = IAPFD;
    Bandgap_Value = (BandgapHigh<<4)+BandgapLow;
    Bandgap_Voltage= Bandgap_Value*3/4;
    clr_CHPCON_IAPEN;

/* ADC Low speed initial*/  
    ENABLE_ADC_BANDGAP;
    ADCCON1|=0x30;            /* clock divider */
    ADCCON2|=0x0E;            /* AQT time */
    AUXR1|=SET_BIT4;          /* ADC clock low speed */
	
/*start bandgap ADC */
    clr_ADCCON0_ADCF;
    set_ADCCON0_ADCS;                                
    while(ADCF == 0);
    ADCdataVBGH = ADCRH;
    ADCdataVBGL = ADCRL;
	
/* to convert VDD value */
    bgvalue = (ADCRH<<4) + ADCRL;
    AVdd = (0x1000/bgvalue)*Bandgap_Voltage;

    printf ("\r\n BG Voltage = %e\r\n", Bandgap_Voltage); 
    printf ("\r\n VDD voltage = %e\r\n", AVdd); 	
}

uint8_t Is_ADC_DataReady(void)
{
	return ADCDataReady;
}

void ADC_DataReady(uint8_t on)
{
	ADCDataReady = on;
}

uint16_t ADC_ModifiedMovingAverage (uint16_t d)
{
	static uint16_t cnt = 0;

	if (Is_ADC_DataReady())
	{
		ADC_DataReady(0);
		
		switch(ADCDataState)
		{
			case ADC_DataState_AVERAGE:
				movingAverageSum_Target += d;
				if (cnt++ >= (ADC_SAMPLE_COUNT-1))
				{
					cnt = 0;
					movingAverage_Target = movingAverageSum_Target >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;;
					ADCDataState = ADC_DataState_MMA;
				}			
				break;
				
			case ADC_DataState_MMA:
				movingAverageSum_Target -=  movingAverage_Target;
				movingAverageSum_Target += d;
				movingAverage_Target = movingAverageSum_Target >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;
	
//				printf("Average : %d\r\n" , movingAverage);
				break;				
		}
	}	

	return movingAverage_Target;
}


void ADC_MMA_Initial(void)
{
	ADCDataState = ADC_DataState_AVERAGE;
	movingAverageSum_Target = 0;
	movingAverage_Target = 0;
	ADC_DataReady(0);
}

uint16_t ADC_ConvertChannel(void)
{
	volatile uint16_t adc_value = 0;
	volatile uint16_t duty_value = 0;
	volatile uint16_t target_value = 0;

	adc_value = ADC_ModifiedMovingAverage((((ADCRH<<4) + ADCRL)>>1)<<1);

	adc_value = (adc_value <= ADC_CONVERT_TARGET) ? (ADC_CONVERT_TARGET) : (adc_value); 
	adc_value = (adc_value >= ADC_RESOLUTION) ? (ADC_RESOLUTION) : (adc_value); 

	#if defined (ENABLE_CONVERT_ADC_TO_DUTY_DEMO)
	target_value = adc_value;

	target_value = (target_value <= GET_ADC_BY_INPUT_VOLT_MIN(AVdd)) ? (GET_ADC_BY_INPUT_VOLT_MIN(AVdd)) : (target_value); 
	target_value = (target_value >= GET_ADC_BY_INPUT_VOLT_MAX(AVdd)) ? (GET_ADC_BY_INPUT_VOLT_MAX(AVdd)) : (target_value); 

	duty_value = CALC_DATA_ADC_TO_DUTY(target_value,AVdd);
	PWM0_CH1_SetDuty(duty_value);
	//for quick demo
	PWM0_CH0_SetDuty(LED_REVERSE(duty_value));

	printf("DUTY:%2d,ADC : %4d ,ADC within range:%4d (VREF : %e v)\r\n",duty_value , adc_value, target_value ,ADC_CALC_DATA_TO_VOLTAGE(adc_value,AVdd) );

	#else 
	
	printf("ADC: 0x%4X (%e v)\r\n",adc_value , ADC_CALC_DATA_TO_VOLTAGE(adc_value,AVdd));	
//	printf("ADC: 0x%4X (%e ,%e)\r\n",adc_value , Bandgap_Voltage,AVdd);

	#endif	

	set_ADCCON0_ADCS; //after convert , trigger again
	
	return adc_value;
}

void ADC_ISR(void) interrupt 11          // Vector @  0x5B
{
	ADC_DataReady(1);
    clr_ADCCON0_ADCF; //clear ADC interrupt flag
}

void ADC_InitChannel(uint8_t CH)
{
	ADC_ReadAVdd();

	switch(CH)
	{
		case ADC_CH0: 
		    ENABLE_ADC_AIN0;
			break;

		case ADC_CH1: 
		    ENABLE_ADC_AIN1;
			break;

		case ADC_CH2: 
		    ENABLE_ADC_AIN2;
			break;

		case ADC_CH3: 
		    ENABLE_ADC_AIN3;
			break;

		case ADC_CH4: 
		    ENABLE_ADC_AIN4;
			break;

		case ADC_CH5: 
		    ENABLE_ADC_AIN5;
			break;

		case ADC_CH6: 
		    ENABLE_ADC_AIN6;
			break;

		case ADC_CH7: 
		    ENABLE_ADC_AIN7;
			break;		
		
	}

  /* ADC Low speed initial*/  
    ADCCON1|=0X30;            /* clock divider */
    ADCCON2|=0X0E;            /* AQT time */

	#if 0
    AUXR1|=SET_BIT4;          /* ADC clock low speed */
	#else
    AUXR1 &= ~SET_BIT4;			//high speed , 500k sps
	#endif

	clr_ADCCON0_ADCF;
	set_ADCCON0_ADCS;                  // ADC start trig signal

	#if 0	//polling
	while(ADCF == 0);
	#else	// Enable ADC interrupt (if use interrupt)
    set_IE_EADC;                        
    ENABLE_GLOBAL_INTERRUPT;
	#endif

	ADC_MMA_Initial();

//	return ((ADCRH<<4) + ADCRL);

}

void Timer0_IRQHandler(void)
{
//	static uint16_t LOG_TIMER = 0;
	static uint16_t CNT_TIMER = 0;
	static uint16_t CNT_ADC = 0;
	static uint16_t CNT_GPIO = 0;
	static uint16_t CNT_LED = 0;

	if (CNT_LED++ >= 18)
	{		
		CNT_LED = 0;
		#if defined (ENABLE_LED_DIMMING_WITH_PWM)
		PWM0_LED_DIMMING();
		#endif
	}

	if (CNT_GPIO++ >= GPIO_TOGGLE_MS)
	{		
		CNT_GPIO = 0;
		GPIO_Toggle();
	}	

	if (CNT_ADC++ >= ADC_SAMPLETIME_MS)
	{		
		CNT_ADC = 0;
		ADC_ConvertChannel();
	}

	if (CNT_TIMER++ >= TIMER_LOG_MS)
	{		
		CNT_TIMER = 0;
//    	printf("LOG:%d\r\n",LOG_TIMER++);
//		SendString("LOG_TIMER\r\n");
	}

}

void Timer0_ISR(void) interrupt 1        // Vector @  0x0B
{
    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;
    clr_TCON_TF0;
	
	Timer0_IRQHandler();
}

void BasicTimer_TIMER0_Init(void)
{
	uint16_t res = 0;

	ENABLE_TIMER0_MODE1;
	
	u8TH0_Tmp = HIBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000);
	u8TL0_Tmp = LOBYTE(TIMER_DIV12_VALUE_1ms_FOSC_240000); 

    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;

    ENABLE_TIMER0_INTERRUPT;                       //enable Timer0 interrupt
    ENABLE_GLOBAL_INTERRUPT;                       //enable interrupts
  
    set_TCON_TR0;                                  //Timer0 run
}

void UART0_Init(void)
{
    UART_Open(SYS_CLOCK,UART0_Timer3,115200);
    ENABLE_UART0_PRINTF;   
}

void SYS_Init(void)
{
    MODIFY_HIRC_24();

    ALL_GPIO_QUASI_MODE;
    ENABLE_GLOBAL_INTERRUPT;                // global enable bit	
}

void main (void) 
{
    SYS_Init();

    UART0_Init();

	PWM0_CHx_Init(PWM_FREQ);	//P1.2 , PWM0_CH0  , LED1
								//P1.1 , PWM0_CH1

	ADC_InitChannel(ADC_CH5);	//P0.4 , ADC_CH5

	GPIO_Init();					//P05 , GPIO
			
	BasicTimer_TIMER0_Init();
	

    while(1)
    {

    }


}



