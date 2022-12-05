/**
 *************************************************************************************
 * @file    main.c
 * @author  Alessandro Scaloni [838287], Daniele Guerrini  [826052],
 * 			Gian Paolo Currà   [789852], Salvatore Bamundo [830264],
 * 			GROUP 10
 * @version V1.3
 * @date    09/06/2020
 * @brief   Compute Roll and Pitch using a filtered Acc
 *************************************************************************************
 */

/* --------------------------------------------------------------------------------------------------
 * JUMPER CABLES CONNECTION:
 * RXD of USB to UART Bridge --> PA2 (Tx) of Board; TXD of USB to UART Bridge --> PA3 (Rx) of Board
 *
 * USART Configuration: 115200-8-N-1
 * -------------------------------------------------------------------------------------------------- */


/*********** Includes  ****************/

#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stm32f4_discovery_lis3dsh.h"

#include <math.h>
#include <mystery_lib.h>

#include <stdio.h>

#include "arm_math.h"
#include "math_helper.h"


/*********** Defines   ****************/

#define NUM_TAPS     10 //num of filter coefficients
#define BLOCK_SIZE   1  //num of input array elements computed per call of fir function

typedef enum {DATA_STREAMING,RESULT_STREAMING}streamingType;


/*********** Declarations *************/
/*------ Function prototypes ---------*/
void USART_Config(void);
void Acc_Config(void);
void Acc_Int_Config(void);

/*------ Global variables  -----------*/
u8 streamActive = 0;
u8 dataReady = 0;
u8 printData = 0;
u8 stream_period_ms = 200;
streamingType ST = DATA_STREAMING;
u8 dataReceived = 0;
u8 chRX = 0;


/* -------------------------------------------------------------------
 * Declare State buffer of size (numTaps + blockSize - 1)
 * ------------------------------------------------------------------- */

static float32_t firStateF32X[BLOCK_SIZE + NUM_TAPS - 1];
static float32_t firStateF32Y[BLOCK_SIZE + NUM_TAPS - 1];
static float32_t firStateF32Z[BLOCK_SIZE + NUM_TAPS - 1];

/* ----------------------------------------------------------------------
 ** FIR Coefficients buffer generated using fir1() MATLAB function.
 ** fir1(9, 5/50)
 ** ------------------------------------------------------------------- */

const float32_t firCoeffs32[NUM_TAPS] = {
		+0.0119822970735782f,    +0.0325936971882185f,   +0.0888097243623084f,    +0.159033608550221f,
		+0.207580672825673f,  	 +0.207580672825673f,    +0.159033608550221f,    +0.0888097243623084f,
		+0.0325936971882185f,    +0.0119822970735782f
};



/***********   Main  ******************/
int main(void)
{
	/*------ Local variables  -----------*/
	//Pitch and Roll buffers declarations
	float32_t  rollBuffer[5];
	float32_t pitchBuffer[5];

	int i; //Iterations counter
	u8 bufferFull = 0; //Pitch and Roll buffer full flag
	int16_t   accData16[3] = {0,0,0};
	float32_t accData32[3] = {0,0,0};
	float32_t bufferYsquared, bufferZsquared; //used to momentarily store the squared value of Y and Z acceleration data

	//FIR filter instances
	arm_fir_instance_f32 X,Y,Z;

	//Five window rolls and pitches mean
	float32_t mRoll, mPitch;
	/*------ End of local variables -----*/

	//Roll and Pitch buffers initialization
	for(i=0; i<5; i++)
	{
		rollBuffer[i] = 0;
		pitchBuffer[i] = 0;
	}
	i = 0;

	// LED initialization
	STM_EVAL_LEDInit(LED3);		// Orange
	STM_EVAL_LEDInit(LED4);		// Green
	STM_EVAL_LEDInit(LED5);		// Red
	STM_EVAL_LEDInit(LED6);		// Blue

	/* USART configuration */
	USART_Config();

	/* Initialize LIS3DSH MEMS Accelerometer and dataReady interrupt */
	Acc_Config();
	Acc_Int_Config();

	/* SysTick configuration */
	if (SysTick_Config(SystemCoreClock / 1000)) {
		/* Capture error */
		STM_EVAL_LEDOn(LED5); //Red LED On if there is an error
		while(1);
	}
	// at startup green LED ON
	STM_EVAL_LEDOn(LED4);

	printf("------------------------------------------------------------------------\r\n");
	printf(" Hello, type s to start/stop streaming\r\n Type d to toggle between data streaming and result streaming\r\n");
	printf("------------------------------------------------------------------------\r\n");

	while (1)
	{
		if (dataReceived)//!= 0
		{
			if (chRX == 's')//Stream toggle
			{
				streamActive = 1 - streamActive;

				if(streamActive == 0)
				{
					//All the LEDS OFF if not streaming
					STM_EVAL_LEDOff(LED6);
					STM_EVAL_LEDOff(LED5);
					STM_EVAL_LEDOff(LED4);
					STM_EVAL_LEDOff(LED3);
					bufferFull = 0; //if chRx is 's' then reset the buffer and the counter
					i = 0;
				}
				else
				{
					//Reinitialize the filters instances every time the stream is toggled on, this flushes the state buffers
					arm_fir_init_f32(&X, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32X[0], BLOCK_SIZE);
					arm_fir_init_f32(&Y, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32Y[0], BLOCK_SIZE);
					arm_fir_init_f32(&Z, NUM_TAPS, (float32_t *)&firCoeffs32[0], &firStateF32Z[0], BLOCK_SIZE);
				}
				printf("------------------------------------------------------------------------\r\n");
				printf("\tStream Toggle\r\n");
				printf("------------------------------------------------------------------------\r\n");
			}
			else if(chRX == 'd')//Mode toggle
			{
				ST = 1 - ST;
				if(ST == 0)
				{//enter data streaming (X, Y, Z)
					printf("------------------------------------------------------------------------\r\n");
					printf("\tStreaming type: Data streaming\r\n");
					printf("------------------------------------------------------------------------\r\n");
				}
				else
				{//enter result streaming (pitch, roll)
					printf("------------------------------------------------------------------------\r\n");
					printf("\tStreaming type: Result streaming\r\n");
					printf("------------------------------------------------------------------------\r\n");
				}
			}
			else if(chRX == 'g')// ??
			{
				if(!streamActive)
					game(&dataReceived, &chRX);
				else
				{
					printf("------------------------------------------------------------------------\r\n");
					printf("\tGame mode not available while streaming\r\n");
					printf("------------------------------------------------------------------------\r\n");
				}

			}
			else
			{
				printf("------------------------------------------------------------------------\r\n");
				printf("\tWrong command\r\n");
				printf("------------------------------------------------------------------------\r\n");
			}

			dataReceived = 0;//Wait for a new dataReceived
		}

		//Data reception and processing
		if (streamActive == 1)
		{
			if (dataReady == 1)
			{
				//Read data from the accelerometer
				LIS3DSH_ReadACC(accData16);

				//Passing from an int16_t array to a float32_t array
				accData32[0] = accData16[0];
				accData32[1] = accData16[1];
				accData32[2] = accData16[2];

				//Filtering the data with the arm FIR CMSIS function
				arm_fir_f32(&X, accData32    , accData32    , BLOCK_SIZE);
				arm_fir_f32(&Y, accData32 + 1, accData32 + 1, BLOCK_SIZE);
				arm_fir_f32(&Z, accData32 + 2, accData32 + 2, BLOCK_SIZE);

				arm_mult_f32(accData32 + 1, accData32 + 1, &bufferYsquared, 1);//using optimized functions instead of raw
				arm_mult_f32(accData32 + 2, accData32 + 2, &bufferZsquared, 1);//data*data multiplication

				//compute Roll and Pitch with atan2 (-90 to 90 deg range)and transform them from rad to deg
				rollBuffer[i] = atan2f(accData32[1], accData32[2])*180/M_PI;
				pitchBuffer[i] = atan2f(-accData32[0],sqrtf(bufferYsquared + bufferZsquared))*180/M_PI;

				i++;//increment the iteration counter
				if(i>4) //Pitch an Roll buffers full condition
				{
					i = 0; //FIFO buffer filling logic when it is full
					bufferFull = 1; //it remains 1 until chRx s is pressed again
				}

				if(bufferFull)//!=0
				{
					//Compute mean value
					arm_mean_f32(pitchBuffer, 5, &mPitch);
					arm_mean_f32(rollBuffer , 5, &mRoll);
				}

				dataReady = 0; //wait for a new AccData
			}

			if(printData == 1)
			{

				if(ST == 0) //Data streaming
				{
					stream_period_ms = 200; //systick streams every 200ms --> 5Hz
					printf("X: %4f     \t\tY: %4f\t\tZ: %4f\r\n", accData32[0], accData32[1], accData32[2]);

				}
				else //Result streaming
				{
					stream_period_ms = 50; //systick streams every 50ms --> 20Hz
					printf("Roll mean: %-3.1f     \t   Pitch mean: %-3.1f\r\n",mRoll,mPitch);
				}
				printData = 0;

				//following code is used to manage LEDs
				switchLEDs(&mRoll, &mPitch);

			}	//END of if(printData)
		}	//END of if(streamActive)
	}	//END while(1)
}	//END main


/*********** Functions   ****************/

/**
 * @brief  This function enables the USART interface
 * @param  None
 * @retval None
 */
void USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	/* Enable UART clock */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure the USART2 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* Enable the RX Interrupt */
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);

	/* USARTx configured as follows: 115200-8-N-1
		- BaudRate = 115200 baud
		- Word Length = 8 Bits
		- One Stop Bit
		- No parity
		- Hardware flow control disabled (RTS and CTS signals)
		- Receive and transmit enabled
	 */
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART initialization */
	USART_Init(USART2, &USART_InitStructure);

	/* Enable USART */
	USART_Cmd(USART2, ENABLE);

}

/**
 * @brief  This function initializes the SPI and the LIS3DSH Accelerometer.
 * @param  None
 * @retval None
 */
void Acc_Config(void)
{
	LIS3DSH_InitTypeDef AccInitStruct;

	AccInitStruct.Output_DataRate = LIS3DSH_DATARATE_100; //Samples of the accelerometer at 100Hz
	AccInitStruct.Axes_Enable = LIS3DSH_XYZ_ENABLE;
	AccInitStruct.SPI_Wire = LIS3DSH_SERIALINTERFACE_4WIRE;
	AccInitStruct.Self_Test = LIS3DSH_SELFTEST_NORMAL;
	AccInitStruct.Full_Scale = LIS3DSH_FULLSCALE_2;
	AccInitStruct.Filter_BW = LIS3DSH_FILTER_BW_800;

	LIS3DSH_Init(&AccInitStruct);

}

/**
 * @brief  This function enables the accelerometer interrupt
 * @param  None
 * @retval None
 */
void Acc_Int_Config(void)
{
	LIS3DSH_InterruptConfigTypeDef AccInterruptStruct;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;


	AccInterruptStruct.Interrupt_Selection_Enable = LIS3DSH_INTERRUPT_1_ENABLE;
	AccInterruptStruct.Interrupt_Request = LIS3DSH_INTERRUPT_REQUEST_PULSED;
	AccInterruptStruct.Interrupt_Signal = LIS3DSH_INTERRUPT_SIGNAL_HIGH;

	LIS3DSH_InterruptConfig(&AccInterruptStruct);

	/* Enable clocks */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/*Configure GPIO pin : PE0 pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN ;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* Configure EXTI Line0 */
	EXTI_InitStructure.EXTI_Line = EXTI_Line0;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Connect EXTI Line0 to PE0 pin */
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource0);

	/* Enable and set EXTI Line0 Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


/*********** IRQ Handlers   ****************/

/**
 * @brief  SysTick interrupt handler, called every ms
 * @param  None
 * @retval None
 */
void SysTick_Handler(void)
{
	static int counterStream_ms = 0;
	static int counterLed_ms = 0;

	if (streamActive == 1)
	{
		counterStream_ms++;
		counterLed_ms++;

		if (counterStream_ms >= stream_period_ms)
		{
			printData = 1; //flag to enable the print
			counterStream_ms = 0;
		}
	}
}

/**
 * @brief  USART IRQ handler, RX Not Empty interrupt, it is called when a data is received
 * @param  None
 * @retval None
 */
void USART2_IRQHandler(void)
{
	/* RX interrupt */
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
	{
		chRX = USART_ReceiveData(USART2);
		dataReceived = 1; //set the flag for the main

	}
}

/**
 * @brief  EXTI IRQ handler, it is called when AccDatas (one per axis) are ready
 * @param  None
 * @retval None
 */
void EXTI0_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line0) != RESET)
	{
		if (streamActive)
			dataReady = 1; //set the flag for the main if the stream is active

		/* Clear the EXTI line 0 pending bit */
		EXTI_ClearITPendingBit(EXTI_Line0);
	}
}


/*********** printf define   ****************/

/**
 * @brief  Retargets the C library printf function to the USART.
 * @param  None
 * @retval None
 */
int __io_putchar(int ch)
{
	/* write a character to the USART */
	USART_SendData(USART2, (uint8_t)ch);

	/* Loop until the end of transmission */
	while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
	{}

	return ch;
}

