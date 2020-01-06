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

#include "MS51_16K.h"

enum{
	TARGET_CH0 = 0 ,
	TARGET_CH1 ,
	TARGET_CH2 ,
	TARGET_CH3 ,	
	
	TARGET_CH4 ,
	TARGET_CH5 ,
	TARGET_CH6 ,
	TARGET_CH7 ,

	TARGET_CH_DEFAULT	
}Channel_TypeDef;

typedef enum
{
	ADC_DataState_AVERAGE = 0 ,
	ADC_DataState_MMA , 
	ADC_DataState_DROP , 
	
	ADC_DataState_DEFAULT 	
}ADC_DataState_TypeDef;


//#define ENABLE_16MHz
#define ENABLE_24MHz

#if defined (ENABLE_16MHz)
#define SYS_CLOCK 								(16000000ul)
#elif defined (ENABLE_24MHz)
#define SYS_CLOCK 								(24000000ul)
#endif

#define PWM_FREQ 								(15000ul)

#define LED_REVERSE(x)							(100-x)			// because lED in EVM schematic , need to reverse level

#define TIMER_LOG_MS							(1000ul)
//#define ADC_SAMPLETIME_MS						(20ul)
#define GPIO_TOGGLE_MS							(500ul)

#define ADC_RESOLUTION							(4096ul)
//#define ADC_REF_VOLTAGE							(3300ul)	//(float)(3.3f)

#define ADC_MAX_TARGET							(4095ul)	//(float)(2.612f)
#define ADC_MIN_TARGET							(0ul)	//(float)(0.423f)

#define DUTY_MAX								(100ul)
#define DUTY_MIN								(1ul)
//#define ADC_CONVERT_TARGET						(float)(ADC_MIN_TARGET*ADC_RESOLUTION/ADC_REF_VOLTAGE) //81.92000 

#define ADC_SAMPLE_COUNT 						(8ul)			// 8
#define ADC_SAMPLE_POWER 						(3ul)			//(5)	 	// 3	,// 2 ^ ?
#define ADC_SAMPLE_DROP 						(4ul)

#define CUSTOM_INPUT_VOLT_MAX(VREF)			(VREF)			//(3300ul)
#define CUSTOM_INPUT_VOLT_MIN					(0)	//(600ul)

#define ADC_DIGITAL_SCALE(void) 					(0xFFFU >> ((0) >> (3U - 1U)))		//0: 12 BIT 

//#define ENABLE_LED_DIMMING_WITH_PWM
#define ENABLE_CONVERT_ADC_TO_DUTY_DEMO

//#define ENABLE_ADC_MMA
#define ENABLE_ADC_DROP_AVG

uint8_t 	u8TH0_Tmp = 0;
uint8_t 	u8TL0_Tmp = 0;

uint8_t 	DUTY_LED = 0;


//UART 0
bit BIT_TMP;
bit BIT_UART;
bit uart0_receive_flag=0;
unsigned char uart0_receive_data;

double  Bandgap_Voltage,AVdd,Bandgap_Value;      //please always use "double" mode for this
unsigned char xdata ADCdataVBGH, ADCdataVBGL;

uint16_t adc_target = 0;
unsigned long int adc_sum_target = 0;
uint16_t adc_data = 0;
uint16_t adc_convert_target = 0;
uint16_t adc_ref_voltage = 0;

ADC_DataState_TypeDef ADCDataState = ADC_DataState_DEFAULT;

typedef enum{
	flag_LED = 0 ,
	
	flag_DEFAULT	
}Flag_Index;

uint8_t BitFlag = 0;
#define BitFlag_ON(flag)							(BitFlag|=flag)
#define BitFlag_OFF(flag)							(BitFlag&=~flag)
#define BitFlag_READ(flag)							((BitFlag&flag)?1:0)
#define ReadBit(bit)								(uint8_t)(1<<bit)

uint8_t is_flag_set(Flag_Index idx)
{
	return BitFlag_READ(ReadBit(idx));
}

void set_flag(Flag_Index idx , uint8_t en)
{
	if (en)
	{
		BitFlag_ON(ReadBit(idx));
	}
	else
	{
		BitFlag_OFF(ReadBit(idx));
	}
}

void send_UARTString(uint8_t* Data)
{
	#if 1
	uint16_t i = 0;

	while (Data[i] != '\0')
	{
		#if 1
		SBUF = Data[i++];
		#else
		UART_Send_Data(UART0,Data[i++]);		
		#endif
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

void send_UARTASCII(uint16_t Temp)
{
    uint8_t print_buf[16];
    uint16_t i = 15, j;

    *(print_buf + i) = '\0';
    j = (uint16_t)Temp >> 31;
    if(j)
        (uint16_t) Temp = ~(uint16_t)Temp + 1;
    do
    {
        i--;
        *(print_buf + i) = '0' + (uint16_t)Temp % 10;
        (uint16_t)Temp = (uint16_t)Temp / 10;
    }
    while((uint16_t)Temp != 0);
    if(j)
    {
        i--;
        *(print_buf + i) = '-';
    }
    send_UARTString(print_buf + i);
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

//	printf("\r\nPWM:0x%x  ,0x%x\r\n\r\n" , PWMPH,PWMPL);
//	send_UARTString("\r\nPWM:");	
//	send_UARTASCII(PWMPH);
//	send_UARTString(",");	
//	send_UARTASCII(PWMPL);
//	send_UARTString("\r\n\r\n");	
	
	PWM0_CH0_SetDuty(LED_REVERSE(0));	

	PWM0_CH1_SetDuty(0);
}

void PWM0_LED_DIMMING(void)
{
	PWM0_CH0_SetDuty(LED_REVERSE(DUTY_LED));
	
//	printf("DUTY:%d\r\n" ,DUTY_LED );
//	send_UARTString("DUTY:");	
//	send_UARTASCII(DUTY_LED);
//	send_UARTString("\r\n");

	if (is_flag_set(flag_LED))
	{
		if ( ++DUTY_LED == 100)
		{
			set_flag(flag_LED,Disable);
			DUTY_LED = 100;
		}
	}
	else
	{
		if ( --DUTY_LED == 0)
		{
			set_flag(flag_LED,Enable);
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

//    printf ("\r\n BG Voltage = %e\r\n", Bandgap_Voltage); 
//    printf ("\r\n VDD voltage = %e\r\n", AVdd); 	
}

uint16_t ADC_DropAndAverage (uint8_t drop , uint8_t avg)
{
	uint8_t n = 0;

	switch(ADCDataState)
	{
		case ADC_DataState_DROP:
			for ( n = 0 ; n < drop ; n++)
			{
				while(ADCF);
			 	adc_data = 0;					
				set_ADCCON0_ADCS; //after convert , trigger again	
			}				
			ADCDataState = ADC_DataState_AVERAGE;

			break;
			
		case ADC_DataState_AVERAGE:
			for ( n = 0 ; n < avg ; n++)
			{
				while(ADCF);
				adc_sum_target += adc_data;					
				set_ADCCON0_ADCS; //after convert , trigger again
			}
			adc_target = adc_sum_target >> ADC_SAMPLE_POWER ;
			break;				
	}
	
	adc_sum_target = 0;

	return adc_target;
}

uint16_t ADC_ModifiedMovingAverage (void)
{
	static uint16_t cnt = 0;
		
	switch(ADCDataState)
	{
		case ADC_DataState_AVERAGE:
			while(ADCF);
			adc_sum_target += adc_data;
			if (cnt++ >= (ADC_SAMPLE_COUNT-1))
			{
				cnt = 0;
				adc_target = adc_sum_target >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;;
				ADCDataState = ADC_DataState_MMA;
			}			
			break;
			
		case ADC_DataState_MMA:
			while(ADCF);
			adc_sum_target -=  adc_target;
			adc_sum_target += adc_data;
			adc_target = adc_sum_target >> ADC_SAMPLE_POWER ;	//	/ADC_SAMPLE_COUNT;

			break;				
	}
	

	return adc_target;
}


void ADC_Parameter_Initial(void)
{
	#if defined (ENABLE_ADC_MMA)
	ADCDataState = ADC_DataState_AVERAGE;
	#endif
		
	#if defined (ENABLE_ADC_DROP_AVG)
	ADCDataState = ADC_DataState_DROP;
	#endif
		
	adc_sum_target = 0;
	adc_target = 0;

}

uint16_t ADC_To_Voltage(uint16_t adc_value)
{
	uint16_t volt = 0;

	volt = (AVdd*adc_value)/ADC_DIGITAL_SCALE();
	
//	printf("input:%4d,volt : %4d mv\r\n",adc_value,volt);
	send_UARTString("adc_value:");	
	send_UARTASCII(adc_value);
	send_UARTString(",volt:");
	send_UARTASCII(volt);
	send_UARTString("mv,AVdd:");
	send_UARTASCII(AVdd);	
	send_UARTString("mv\r\n");

	return volt;	
}

uint16_t ADC_To_Duty(uint16_t adc_value)
{
	uint16_t adc_max = 0;
	uint16_t adc_min = 0;
	uint16_t volt_max = CUSTOM_INPUT_VOLT_MAX(AVdd);//CUSTOM_INPUT_VOLT_MAX(0);
	uint16_t volt_min = CUSTOM_INPUT_VOLT_MIN;
	uint16_t duty = 0;
	uint16_t adc_target = 0;	
	uint16_t interval = DUTY_MAX - DUTY_MIN + 1;	
	adc_ref_voltage = AVdd;
	
	adc_max = (ADC_RESOLUTION * volt_max)/adc_ref_voltage ;
	adc_min = (ADC_RESOLUTION * volt_min)/adc_ref_voltage ;	
	
	adc_target = (adc_value <= adc_min) ? (adc_min) : (adc_value) ;
	adc_target = (adc_target >= adc_max) ? (adc_max) : (adc_target) ;

	duty = (float)(adc_target - adc_min)*interval/(adc_max - adc_min) + 1;
	duty = (duty >= DUTY_MAX) ? (DUTY_MAX) : (duty) ;
	
//	printf("adc_value:%4d,adc_min:%4d,adc_max:%4d,adc_target : %4d , duty : %3d \r\n",adc_value,adc_min,adc_max , adc_target , duty);
	send_UARTString("adc_value:");	
	send_UARTASCII(adc_value);
	send_UARTString(",adc_min:");
	send_UARTASCII(adc_min);
	send_UARTString(",adc_max:");
	send_UARTASCII(adc_max);
	send_UARTString(",adc_target:");
	send_UARTASCII(adc_target);
	send_UARTString(",duty:");
	send_UARTASCII(duty);
	send_UARTString("\r\n");

	return duty;	
}


uint16_t ADC_ConvertChannel(void)
{
	volatile uint16_t adc_value = 0;
	volatile uint16_t duty_value = 0;
	adc_ref_voltage = AVdd;
	
	adc_convert_target = (ADC_MIN_TARGET*ADC_RESOLUTION/adc_ref_voltage);

	#if defined (ENABLE_ADC_DROP_AVG)
	adc_value = ADC_DropAndAverage(ADC_SAMPLE_DROP,ADC_SAMPLE_COUNT);
//	send_UARTString("adc_value (DropAndAverage) :");	
//	send_UARTASCII(adc_value);
//	send_UARTString("\r\n");

	adc_value = (adc_value <= adc_convert_target) ? (adc_convert_target) : (adc_value); 
	adc_value = (adc_value >= ADC_RESOLUTION) ? (ADC_RESOLUTION) : (adc_value); 

	#if defined (ENABLE_CONVERT_ADC_TO_DUTY_DEMO)
	duty_value = ADC_To_Duty(adc_value);	
	PWM0_CH1_SetDuty(duty_value);
	//for quick demo
	PWM0_CH0_SetDuty(LED_REVERSE(duty_value));
	#else
	ADC_To_Voltage(adc_value);
	#endif

	
	#endif
	
	#if defined (ENABLE_ADC_MMA)
	adc_value = ADC_ModifiedMovingAverage();
//	send_UARTString("adc_value (MMA) :");	
//	send_UARTASCII(adc_value);
//	send_UARTString("\r\n");
	
	adc_value = (adc_value <= adc_convert_target) ? (adc_convert_target) : (adc_value); 
	adc_value = (adc_value >= ADC_RESOLUTION) ? (ADC_RESOLUTION) : (adc_value); 

	#if defined (ENABLE_CONVERT_ADC_TO_DUTY_DEMO)
	duty_value = ADC_To_Duty(adc_value);	
	PWM0_CH1_SetDuty(duty_value);
	//for quick demo
	PWM0_CH0_SetDuty(LED_REVERSE(duty_value));
	#else
	ADC_To_Voltage(adc_value);
	#endif

	set_ADCCON0_ADCS; //after convert , trigger again
	#endif
	
	return adc_value;
}

void ADC_ISR(void) interrupt 11          // Vector @  0x5B
{	
    _push_(SFRS);

//	adc_data = ((ADCRH<<4) + ADCRL);	
	adc_data = (((ADCRH<<4) + ADCRL)>>1)<<1;

//	send_UARTString("ADC_ISR :");	
//	send_UARTASCII(adc_data);
//	send_UARTString("\r\n");

    clr_ADCCON0_ADCF; //clear ADC interrupt flag

     _pop_(SFRS);   
}

void ADC_InitChannel(uint8_t CH)
{
	ADC_ReadAVdd();

	switch(CH)
	{
		case TARGET_CH0: 
		    ENABLE_ADC_AIN0;
			break;

		case TARGET_CH1: 
		    ENABLE_ADC_AIN1;
			break;

		case TARGET_CH2: 
		    ENABLE_ADC_AIN2;
			break;

		case TARGET_CH3: 
		    ENABLE_ADC_AIN3;
			break;

		case TARGET_CH4: 
		    ENABLE_ADC_AIN4;
			break;

		case TARGET_CH5: 
		    ENABLE_ADC_AIN5;
			break;

		case TARGET_CH6: 
		    ENABLE_ADC_AIN6;
			break;

		case TARGET_CH7: 
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

	ADC_Parameter_Initial();

//	return ((ADCRH<<4) + ADCRL);

}

void Timer0_IRQHandler(void)
{
//	static uint16_t LOG_TIMER = 0;
//	static uint16_t CNT_TIMER = 0;
//	static uint16_t CNT_ADC = 0;
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

//	if (CNT_ADC++ >= ADC_SAMPLETIME_MS)
//	{		
//		CNT_ADC = 0;
//		ADC_ConvertChannel();
//	}

//	if (CNT_TIMER++ >= TIMER_LOG_MS)
//	{		
//		CNT_TIMER = 0;
//    	printf("LOG:%d\r\n",LOG_TIMER++);
//		send_UARTString("LOG_TIMER\r\n");
//	}

}

void Timer0_ISR(void) interrupt 1        // Vector @  0x0B
{
    TH0 = u8TH0_Tmp;
    TL0 = u8TL0_Tmp;
    clr_TCON_TF0;
	
	Timer0_IRQHandler();
}

void TIMER0_Init(void)
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


void Serial_ISR (void) interrupt 4 
{
    if (RI)
    {   
      uart0_receive_flag = 1;
      uart0_receive_data = SBUF;
      clr_SCON_RI;                                         // Clear RI (Receive Interrupt).
    }
    if  (TI)
    {
      if(!BIT_UART)
      {
          TI = 0;
      }
    }
}

void UART0_Init(void)
{
	#if 1
	unsigned long u32Baudrate = 115200;
	P06_QUASI_MODE;    //Setting UART pin as Quasi mode for transmit
	SCON = 0x50;          //UART0 Mode1,REN=1,TI=1
	set_PCON_SMOD;        //UART0 Double Rate Enable
	T3CON &= 0xF8;        //T3PS2=0,T3PS1=0,T3PS0=0(Prescale=1)
	set_T3CON_BRCK;        //UART0 baud rate clock source = Timer3

	#if defined (ENABLE_16MHz)
	RH3    = HIBYTE(65536 - (1000000/u32Baudrate)-1);  
	RL3    = LOBYTE(65536 - (1000000/u32Baudrate)-1);  
	#elif defined (ENABLE_24MHz)
	RH3    = HIBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	RL3    = LOBYTE(65536 - (SYS_CLOCK/16/u32Baudrate));  
	#endif
	
	set_T3CON_TR3;         //Trigger Timer3
	set_IE_ES;

	ENABLE_GLOBAL_INTERRUPT;

	set_SCON_TI;
	BIT_UART=1;
	#else	
    UART_Open(SYS_CLOCK,UART0_Timer3,115200);
    ENABLE_UART0_PRINTF; 
	#endif
}


#if defined (ENABLE_16MHz)
void MODIFY_HIRC_16(void)
{
    unsigned char data hircmap0,hircmap1;
    set_CHPCON_IAPEN;
    IAPAL = 0x30;
    IAPAH = 0x00;
    IAPCN = READ_UID;
    set_IAPTRG_IAPGO;
    hircmap0 = IAPFD;
    IAPAL = 0x31;
    IAPAH = 0x00;
    set_IAPTRG_IAPGO;
    hircmap1 = IAPFD;
    clr_CHPCON_IAPEN;
    TA=0XAA;
    TA=0X55;
    RCTRIM0 = hircmap0;
    TA=0XAA;
    TA=0X55;
    RCTRIM1 = hircmap1;
}

#elif defined (ENABLE_24MHz)
void MODIFY_HIRC_24(void)
{
    unsigned char data hircmap0,hircmap1;
/* Check if power on reset, modify HIRC */
    if (PCON&SET_BIT4)
    {
        set_CHPCON_IAPEN;
        IAPAL = 0x38;
        IAPAH = 0x00;
        IAPCN = READ_UID;
        set_IAPTRG_IAPGO;
        hircmap0 = IAPFD;
        IAPAL = 0x39;
        IAPAH = 0x00;
        set_IAPTRG_IAPGO;
        hircmap1 = IAPFD;
        clr_CHPCON_IAPEN;
        TA=0XAA;
        TA=0X55;
        RCTRIM0 = hircmap0;
        TA=0XAA;
        TA=0X55;
        RCTRIM1 = hircmap1;
        clr_CHPCON_IAPEN;
    }
}

#endif

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

	//P1.2 , PWM0_CH0  , LED1
	//P1.1 , PWM0_CH1
	PWM0_CHx_Init(PWM_FREQ);
								
	//P0.4 , ADC_CH5
	ADC_InitChannel(TARGET_CH5);

	//P05 , GPIO
	GPIO_Init();					
			
	TIMER0_Init();	

    while(1)
    {
		ADC_ConvertChannel();
    }


}



