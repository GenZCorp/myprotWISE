/*LoRa definitions begin*/
/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdlib.h>
#include "hw.h"
#include "radio.h"
#include "timeServer.h"
#include "delay.h"
#include "low_power.h"
#include "vcom.h"


#if defined( USE_BAND_868 )

#define RF_FREQUENCY                                868000000 // Hz

#elif defined( USE_BAND_915 )

#define RF_FREQUENCY                                915000000 // Hz

#elif defined( USE_BAND_433 )

#define RF_FREQUENCY                                433000000 // Hz

#else
    #error "Please define a frequency band in the compiler options."
#endif

#define IS_ENDNODE 																	1				// switch on endnode@Shees

#define NETWORK_KEY																	0x23  ////adjust these 2 @tobecontinued
#define RADIO_ID																		0x21 ////adjust these 2 @tobecontinued
#define SENSOR_FLAGS																0x00 //test1 if operates as planned should give 0x31 at receiver
#define NUMOFSENSORSATT															8

#define MOISTURE_ATTACHED														1
#define PH_ATTACHED																	0
#define NITROGEN_ATTACHED														0
#define POTASSIUM_ATTACHED													0
#define PHOSPHORUS_ATTACHED													0
#define DISSOLVED_O2_ATTACHED												0
#define GSM_ATTACHED																0

#define BATT1_ADDR																	0xE0 //replace this with addr for sensor 1
#define BATT2_ADDR																	0xE0 //replace with addr to set for bat2

#define MOISTURE_FLAG																0x01
#define PH_FLAG																			0x02
#define NITROGEN_FLAG																0x04
#define PHOSPHORUS_FLAG															0x08
#define POTASSIUM_FLAG															0x10
#define DISSOLVED_O2_FLAG														0x20
#define BATTERY1_FLAG																0x40
#define BATTERY2_FLAG																0x80


#define GPS_RX																			GPIO_PIN_3
#define GPS_TX																			GPIO_PIN_2

#define GPS_PORT																		GPIOA

#define MOISTURE_ENABLE_PIN													GPIO_PIN_10
#define PH_ENABLE_PIN																GPIO_PIN_12
#define NITROGEN_ENABLE_PIN													GPIO_PIN_13
#define PHOSPHORUS_ENABLE_PIN												GPIO_PIN_14
#define POTASSIUM_ENABLE_PIN												GPIO_PIN_15
#define DISSOLVED_O2_ENABLE_PIN											GPIO_PIN_14

#define MOISTURE_ENABLE_PORT												GPIOC
#define PH_ENABLE_PORT															GPIOC
#define NITROGEN_ENABLE_PORT												GPIOA
#define PHOSPHORUS_ENABLE_PORT											GPIOA
#define POTASSIUM_ENABLE_PORT												GPIOA
#define DISSOLVED_O2_ENABLE_PORT										GPIOC

#define MOISTURE_PIN																GPIO_PIN_5
#define PH_PIN																			GPIO_PIN_1
#define NITROGEN_PIN																GPIO_PIN_15
#define PHOSPHORUS_PIN															GPIO_PIN_14
#define POTASSIUM_PIN																GPIO_PIN_13
#define DISSOLVED_O2_PIN														GPIO_PIN_4

#define BATTERY1_SCL_PIN															GPIO_PIN_8 //I2C 1 SCL
#define BATTERY1_SDA_PIN															GPIO_PIN_9 //I2C 1 SDA

#define BATTERY2_SCL_PIN														GPIO_PIN_10 //I2C 1 SCL
#define BATTERY2_SDA_PIN														GPIO_PIN_11 //I2C 1 SDA

#define MOISTURE_PORT																GPIOC
#define PH_PORT																			GPIOB
#define NITROGEN_PORT																GPIOB
#define PHOSPHORUS_PORT															GPIOB
#define POTASSIUM_PORT															GPIOB
#define DISSOLVED_O2_PORT														GPIOC

#define BATTERY_PORT																GPIOB   //uses I2C

#define M_DIVTHRESH																	1 //normalized thresholds mapped to certain floating point scales
#define PH_DIVTHRESH																1 //mapped total range from 0 to 256
#define N_DIVTHRESH																	2
#define P_DIVTHRESH																	2
#define PO_DIVTHRESH																2
#define D_O2_DIVTHRESH															1
#define BAT1_DIVTHRESH															3 //volatge min in volts
#define BAT2_DIVTHRESH															3 //voltage min in volts

#define TIMEOUTWAIT																	1000			  //1sec					@Shees
#define CADWAIT																			500 				//0.5sec				@Shees
#define CADAVAILABLEWAIT														0 					//1sec				@Shees
#define LOWPOWERWAIT																2000 				//2sec				@Shees

#define TX_OUTPUT_POWER                             17        	// dBm


#define RX_TIMEOUT_VALUE                            1000 //changed from 1sec
#define BUFFER_SIZE                                 19 // Define the payload size here in bytes

uint8_t flags;
int testvar=0; //@test

uint16_t BufferSize = BUFFER_SIZE;
uint8_t Buffer[BUFFER_SIZE];

int8_t RssiValue = 0;
int8_t SnrValue = 0;

#if defined( USE_MODEM_LORA )

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         5         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false

#elif defined( USE_MODEM_FSK )

#define FSK_FDEV                                    25e3      // Hz
#define FSK_DATARATE                                50e3      // bps
#define FSK_BANDWIDTH                               50e3      // Hz
#define FSK_AFC_BANDWIDTH                           83.333e3  // Hz
#define FSK_PREAMBLE_LENGTH                         5         // Same for Tx and Rx
#define FSK_FIX_LENGTH_PAYLOAD_ON                   false

#else
    #error "Please define a modem in the compiler options."
#endif

typedef enum
{
		DAQ_GAP,//@shees
		DAQ,//@shees
		RANDOM_WAIT, //@shess
    LOWPOWER,
    RX,
    RX_TIMEOUT,
    RX_ERROR,
    TX,
    TX_TIMEOUT,
		CAD_DETECT, //@Shees
		CAD_DETECTED, //Shees
		WAITFORCADCLEAR,//@Shees
		WAITFORCADAVAILABLE,//@Shees
}States_t;

States_t State = LOWPOWER;


/* Private function prototypes -----------------------------------------------*/
/*!
 * Radio events function pointer
 */
static RadioEvents_t RadioEvents;

/*!
 * \brief Function to be executed on Radio Tx Done event
 */
void OnTxDone( void );

/*!
 * \brief Function to be executed on Radio Rx Done event
 */
void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );

/*!
 * \brief Function executed on Radio Tx Timeout event
 */
void OnTxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Timeout event
 */
void OnRxTimeout( void );

/*!
 * \brief Function executed on Radio Rx Error event
 */
void OnRxError( void );

/*!
 * \brief Function executed on CAD Detected event @Shees
 */
void OnCADDetect( bool channelActivityDetected ); 

/*!
 * \brief Function to put desired string data in buffer @Shees
 */
uint8_t* putinbuffer(uint8_t Buffer[],uint16_t Buffersize, uint8_t* data);

/*LoRa definitions end*/
/**
 * Main application entry point.
 */
 
int main( void )
{

    HAL_Init( );
	ADC_HandleTypeDef hadc;
		I2C_HandleTypeDef hi2c1;
	I2C_HandleTypeDef hi2c2;
	UART_HandleTypeDef huart2;
	if(IS_ENDNODE)
	{
					////////initialize node GPIOS
			//@Initialize GPIO shees begin
	//initialize analog inputs and didgital load switch control outputs
	//Analog BEGIN:
	
	
	
	ADC_ChannelConfTypeDef sConfig;
	
	hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_20;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC GPIO Configuration    
    PC4     ------> ADC_IN14
    PC5     ------> ADC_IN15
    PB1     ------> ADC_IN9
    PB13     ------> ADC_IN19
    PB14     ------> ADC_IN20
    PB15     ------> ADC_IN21	
    */
	
		GPIO_InitTypeDef GPIO_InitStruct;
	
    GPIO_InitStruct.Pin = MOISTURE_PIN|DISSOLVED_O2_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(MOISTURE_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PH_PIN|NITROGEN_PIN|PHOSPHORUS_PIN|POTASSIUM_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(PH_PORT, &GPIO_InitStruct);
	//Analog END:
	
	//Digital BEGIN: ////check which digital pins need to be used @continuehereshees
	
	GPIO_InitStruct.Pin = MOISTURE_ENABLE_PIN|PH_ENABLE_PIN|DISSOLVED_O2_ENABLE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(MOISTURE_ENABLE_PORT, &GPIO_InitStruct);
  	GPIO_InitStruct.Pin = NITROGEN_ENABLE_PIN|
												PHOSPHORUS_ENABLE_PIN|
												POTASSIUM_ENABLE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(NITROGEN_ENABLE_PORT, &GPIO_InitStruct);
	
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();	

	
	HAL_GPIO_WritePin(MOISTURE_ENABLE_PORT, MOISTURE_ENABLE_PIN|PH_ENABLE_PIN|DISSOLVED_O2_ENABLE_PIN, 0);
	HAL_GPIO_WritePin(NITROGEN_ENABLE_PORT, NITROGEN_ENABLE_PIN|PHOSPHORUS_ENABLE_PIN|POTASSIUM_ENABLE_PIN, 0);
	
	//Digital END
	
	
	//I2C Initialization

	hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
	hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
	
	
	  GPIO_InitStruct.Pin = BATTERY1_SCL_PIN|BATTERY1_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(BATTERY_PORT, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();
	
	
		GPIO_InitStruct.Pin = BATTERY2_SCL_PIN|BATTERY2_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(BATTERY_PORT, &GPIO_InitStruct);

    /* Peripheral clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();
		
	//@Initialize GPIO shees end
	//@Initialize UART shees start
	//commentedout for error testing sake
	/*
	huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
 __HAL_RCC_USART2_CLK_ENABLE();
  
    //USART2 GPIO Configuration    
    //PA2     ------> USART2_TX
    //PA3     ------> USART2_RX 
    
    GPIO_InitStruct.Pin = GPS_TX|GPS_RX;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPS_PORT, &GPIO_InitStruct);
	//@Initialize UART shees end
	*/
////////////////////////////////////end of initialize node gpios
	}
		
    SystemClock_Config( );
  
	/*LoRa initialize begin*/
    DBG_Init( );
		

	//

    HW_Init( );  
	
    ///////////////////////////////////////////////// Radio initialization
    RadioEvents.TxDone = OnTxDone;
    RadioEvents.RxDone = OnRxDone;
    RadioEvents.TxTimeout = OnTxTimeout;
    RadioEvents.RxTimeout = OnRxTimeout;
    RadioEvents.RxError = OnRxError;
		RadioEvents.CadDone = OnCADDetect; //@Shees

    Radio.Init( &RadioEvents );

    Radio.SetChannel( RF_FREQUENCY );

#if defined( USE_MODEM_LORA )

    Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000000 );
    
    Radio.SetRxConfig( MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                                   LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                                   LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

#elif defined( USE_MODEM_FSK )

    Radio.SetTxConfig( MODEM_FSK, TX_OUTPUT_POWER, FSK_FDEV, 0,
                                  FSK_DATARATE, 0,
                                  FSK_PREAMBLE_LENGTH, FSK_FIX_LENGTH_PAYLOAD_ON,
                                  true, 0, 0, 0, 3000000 );
    
    Radio.SetRxConfig( MODEM_FSK, FSK_BANDWIDTH, FSK_DATARATE,
                                  0, FSK_AFC_BANDWIDTH, FSK_PREAMBLE_LENGTH,
                                  0, FSK_FIX_LENGTH_PAYLOAD_ON, 0, true,
                                  0, 0,false, true );

#else
    #error "Please define a frequency band/modulation technique in the compiler options."
#endif
 
										/*LoRa initialize end*/	
//Radio.Sleep(); //this might cause problems

		if(IS_ENDNODE) //@Shees
		{

			PRINTF("ENDNODE CODE BEGINNING\n");
			uint8_t data[BufferSize - 3];
			State = DAQ_GAP; //@editShees
			uint8_t preva = 0,prevb =0,prevc=0,prevd=0,preve=0,prevf=0,prevg=0,prevh =0;
			
			int InitialRun[NUMOFSENSORSATT];
			
			//State = TX; //@editShees
			for(int i=0;i<NUMOFSENSORSATT;i++)
			{
				InitialRun[i]=1;
			}
			while(1)
			{
				PRINTF("in while1\n");
//				PRINTF("pre cad\n");// @editshees
//				Radio.StartCad(); ///// @editshees added to check cad //comment this out for now and find the point when it calls the fucntions OnCADDetected below or sets its parameters from similarity of when it calls OnTXDone
//			PRINTF("post cad\n");// @editshees
//				HAL_Delay(1000);// @editshees

				switch(State)
			{
					case DAQ_GAP:
					{
						//for(int i =0;i<1;i++) //1minute for now hence i <1//30minutes delay hence i<30 /////////use timer here instead for better power consumption
						//HAL_Delay(1000);
						PRINTF("DAQ_GAP\n");
						State = DAQ;
						break;
					}
					case DAQ:
					{
						PRINTF("DAQ\n");
						uint8_t a,b,c,d,e,f,g,h; //use fixed point instead of float //mapped values btw 0 and 256
						float j,k;
						int t,u,v,w,x,y;
//						t=10;
//						u=10;
//						v=10;
//						w=10;
//						x=10;
//						y=10;
						
							//use HW function for battery level measurement, initrialize analogue pins and read data put data in floats
						if(MOISTURE_ATTACHED)
						{
						HAL_GPIO_WritePin(MOISTURE_ENABLE_PORT,MOISTURE_ENABLE_PIN,1);
						t = HAL_GPIO_ReadPin(MOISTURE_PORT, MOISTURE_PIN);
						HAL_GPIO_WritePin(MOISTURE_ENABLE_PORT,MOISTURE_ENABLE_PIN,0);
						}
						else {t=0;}
							
						if(PH_ATTACHED)
						{							
						HAL_GPIO_WritePin(PH_ENABLE_PORT,PH_ENABLE_PIN,1);
						u = HAL_GPIO_ReadPin(PH_PORT, PH_PIN);
						HAL_GPIO_WritePin(PH_ENABLE_PORT,PH_ENABLE_PIN,0);
						}
						else 
						{u=0;}
						
						if(NITROGEN_ATTACHED)
						{
						HAL_GPIO_WritePin(NITROGEN_ENABLE_PORT,NITROGEN_ENABLE_PIN,1);
						v = HAL_GPIO_ReadPin(NITROGEN_PORT,	NITROGEN_PIN);
						HAL_GPIO_WritePin(NITROGEN_ENABLE_PORT,NITROGEN_ENABLE_PIN,0);
						}
						else {v=0;}
						
						if(PHOSPHORUS_ATTACHED)
						{
						HAL_GPIO_WritePin(PHOSPHORUS_ENABLE_PORT,PHOSPHORUS_ENABLE_PIN,1);
						w = HAL_GPIO_ReadPin(PHOSPHORUS_PORT, PHOSPHORUS_PIN);
						HAL_GPIO_WritePin(PHOSPHORUS_ENABLE_PORT,PHOSPHORUS_ENABLE_PIN,0);
						}
						else {w=0;}
						
						if(POTASSIUM_ATTACHED)
						{
						HAL_GPIO_WritePin(POTASSIUM_ENABLE_PORT,POTASSIUM_ENABLE_PIN,1);
						x = HAL_GPIO_ReadPin(POTASSIUM_PORT, POTASSIUM_PIN);
						HAL_GPIO_WritePin(POTASSIUM_ENABLE_PORT,POTASSIUM_ENABLE_PIN,0);
						}
						else {x=0;}
						
						if(DISSOLVED_O2_ATTACHED)
						{
						HAL_GPIO_WritePin(DISSOLVED_O2_ENABLE_PORT,DISSOLVED_O2_ENABLE_PIN,1);
						y = HAL_GPIO_ReadPin(DISSOLVED_O2_PORT, DISSOLVED_O2_PIN);
						HAL_GPIO_WritePin(DISSOLVED_O2_ENABLE_PORT,DISSOLVED_O2_ENABLE_PIN,0);
						}
						else {y=0;}
						if(GSM_ATTACHED)
						{
							uint8_t* gsmcommand;
							*gsmcommand = 0x01;
							//HAL_UART_Transmit(&huart2,gsmcommand,4,100); //replace with sending gsm command for receiving lat and long
							
						}
						else {k=99.99;j=99.99;}
							//I2C to get battery value begins here
									//read battery values from i2c in variables g and h
						{
							uint8_t *dataholder;
							*dataholder = 10;
							uint8_t voltagehigherbits = 0x08,voltagelowerbits = 0x09;
							////////////////////////check how to read this address.
						//	HAL_I2C_Master_Receive(&hi2c1,BATT1_ADDR,dataholder,1,100);
							g=38;
							//HAL_I2C_Master_Receive(&hi2c2,BATT2_ADDR,dataholder,1,100);
							h=38;
						}
						
							//I2C to get battery value ends here
							//calculate equivalent float value/ fixed point value
						
						a=t*(256/4096);
						b=u*(256/4096);
						c=v*(256/4096);
						d=w*(256/4096);
						e=x*(256/4096);
						f=y*(256/4096);
						flags = SENSOR_FLAGS;
						//claculate which data diverged beyond threshold for only the sensor that are attached
						//calculate the flags to set based on the values that changed ,set not attached flags to zero
						if(MOISTURE_ATTACHED)
						{
							int divergence;
							//claculate threshold divergence in variable
							divergence = abs(preva - a);
							if(InitialRun[0])
							{
								divergence = 100;
								InitialRun[0] = 0;
							}
							if(divergence > M_DIVTHRESH)
							{
								flags = flags|MOISTURE_FLAG;
								preva = a;
							}
						}
						if(PH_ATTACHED)
						{
							int divergence;
							//claculate threshold divergence in variable
							divergence = abs(prevb - b);
							if(InitialRun[1])
							{
								divergence = 100;
								InitialRun[1] = 0;
							}
							if(divergence > PH_DIVTHRESH)
							{
								flags = flags|PH_FLAG;
								prevb = b;
							}
						}
						if(NITROGEN_ATTACHED)
						{
							int divergence;
							//claculate threshold divergence in variable
							divergence = abs(prevc - c);
							if(InitialRun[2])
							{
								divergence = 100;
								InitialRun[2] = 0;
							}
							if(divergence > N_DIVTHRESH)
							{
								flags = flags|NITROGEN_FLAG;
								prevc = c;
							}
						}
						if(PHOSPHORUS_ATTACHED)
						{
							int divergence;
							//claculate threshold divergence in variable
							divergence = abs(prevd - d);
							if(InitialRun[3])
							{
								divergence = 100;
								InitialRun[3] = 0;
							}
							if(divergence > P_DIVTHRESH)
							{
								flags = flags|PHOSPHORUS_FLAG;
								prevd = d;
							}
						}
						if(POTASSIUM_ATTACHED)
						{
							int divergence;
							//claculate threshold divergence in variable
							divergence = abs(preve - e);
							if(InitialRun[4])
							{
								divergence = 100;
								InitialRun[4] = 0;
							}
							if(divergence > PO_DIVTHRESH)
							{
								flags = flags|POTASSIUM_FLAG;
								preve = e;
							}
						}
						if(DISSOLVED_O2_ATTACHED)
						{
							int divergence;
							//claculate threshold divergence in variable
							divergence = abs(prevf - f);
							if(InitialRun[5])
							{
								divergence = 100;
								InitialRun[5] = 0;
							}
							if(divergence > D_O2_DIVTHRESH)
							{
								flags = flags|DISSOLVED_O2_FLAG;
								prevf = f;
							}
						}
						{
							int divergence;
							//claculate threshold divergence in variable
							divergence = abs(prevg - g);
							if(InitialRun[6])
							{
								divergence = 100;
								InitialRun[6] = 0;
							}
							if(divergence > BAT1_DIVTHRESH)
							{
								flags = flags|BATTERY1_FLAG;
								prevg = g;
							}
						}
						{
							int divergence;
							//claculate threshold divergence in variable
							divergence = abs(prevh - h);
							if(InitialRun[7])
							{
								divergence = 100;
								InitialRun[7] = 0;
							}
							if(divergence > BAT2_DIVTHRESH)
							{
								flags = flags|BATTERY2_FLAG;
								prevh = h;
							}
						}
						{
							int i=0;
							if(GSM_ATTACHED)
							{
								//somehow store all data in data[i]
								
								data[i]=j;
								i++;
								data[i]=k;
								i++;
							}
							if((flags&MOISTURE_FLAG)==MOISTURE_FLAG)
							{
								data[i]=a;
								i++;
							}
							if((flags&PH_FLAG)==PH_FLAG)
							{
								data[i]=b;
								i++;
							}
							if((flags&NITROGEN_FLAG)==NITROGEN_FLAG)
							{
								data[i]=c;
								i++;
							}
							if((flags&PHOSPHORUS_FLAG)==PHOSPHORUS_FLAG)
							{
								data[i]=d;
								i++;
							}
							if((flags&POTASSIUM_FLAG)==POTASSIUM_FLAG)
							{
								data[i]=e;
								i++;
							}
							if((flags&DISSOLVED_O2_FLAG)==DISSOLVED_O2_FLAG)
							{
								data[i]=f;
								i++;
							}
							if((flags&BATTERY1_FLAG)==BATTERY1_FLAG)
							{
								data[i]=g;
								i++;
							}
							if((flags&BATTERY2_FLAG)==BATTERY2_FLAG)
							{
								data[i]=h;
								i++;
							}
						}
					//typecast the values to string
					//data = strcat(); //catenate floats after typecast into string
					//for test purposes///
  				//data = "ok";
					State = RANDOM_WAIT;
					break;
				}
				case RANDOM_WAIT:
				{
					PRINTF("RANDOM_WAIT\n");
					int x,N=6;//what is N ???
					x = (int)((double)rand() / ((double)RAND_MAX + 1) * N);	
					//PRINTF("Random: %d\n",x);
					HAL_Delay(x+(NETWORK_KEY*RADIO_ID));
					State=CAD_DETECT;
					break;
				}
				case TX:
				{
					PRINTF("TX\n");
					*Buffer = *putinbuffer(Buffer,BufferSize,data);
					Radio.Send( Buffer, BufferSize );
					//PRINTF("Payload Sent\n"); 
					State=CAD_DETECT; //@editShees @forembedded
					break;
				}
				case TX_TIMEOUT:
				{
					//PRINTF("Send Failed TX_TIMEOUT\n");
					PRINTF("TIMEOUT\n");
					HAL_Delay(TIMEOUTWAIT);
					//State = CAD_DETECT; @editShees
					State = CAD_DETECT; //@editShees
					break;
				}
				case CAD_DETECT:
				{
					//PRINTF("CAD DETECT RUN\n");
					PRINTF("CAD\n");
					State=TX;
					Radio.StartCad();
					HAL_Delay(3000);
					//PRINTF("CAD DETECT ENDED\n");
					break;
				}
				case CAD_DETECTED:
				{
					//PRINTF("CAD DETECTED\n");
					PRINTF("CAD_DET\n");
					State=WAITFORCADCLEAR;
					break;
				}
				case WAITFORCADCLEAR:
				{
					//PRINTF("WAITING FOR CAD CLEAR\n");
					PRINTF("CAD_WAIT\n");
					HAL_Delay(CADWAIT);
					State = RANDOM_WAIT;
					break;
				}
				case LOWPOWER:
				{
					//PRINTF("L");
					PRINTF("LOWPOWER\n");
					Radio.Sleep();
					HAL_Delay(LOWPOWERWAIT);
					State=TX;
					break;
				}
				default:
					State=LOWPOWER;
			}

			////////////////////////
			   DISABLE_IRQ( );
    /* if an interupt has occured after __disable_irq, it is kept pending 
     * and cortex will not enter low power anyway  */
    if (State == LOWPOWER)
    {
#ifndef LOW_POWER_DISABLE
      LowPower_Handler( );
#endif
    }
    ENABLE_IRQ( );
		}
		}	
			
		else if(!IS_ENDNODE)
		{
				PRINTF("BEGIN GATEWAY PROTOCOL");
			//State = CAD_DETECT; //@editShees
				State = RX; //@for embedded shees
			while(1)
			{
			switch(State)
			{
				case RX:
					{
				//..............Fill buffer all the way to buffersize before sending here
						//Radio.Rx( RX_TIMEOUT_VALUE ); @forembedded shees
						PRINTF ("Receiving ");
						Radio.Rx( 5000 );
						HAL_Delay(5000);
						if(BufferSize > 0)
						{
//@forembedded shees							PRINTF("in RX\n");
//							for(int i=0;i < BufferSize;i++)
//							PRINTF((char*)Buffer[i]);
//@forembedded shees							PRINTF("\n");
							PRINTF("RX\n");
							//@forembedded shees PRINTF("\n");PRINTF("\n");
						}
						//State=CAD_DETECT; //@editShees
						//State = LOWPOWER; //@forembedded shees
						
					break;
					}
				case RX_TIMEOUT:
				{
					PRINTF("Receive Failed RX_TIMEOUT\n");
					HAL_Delay(TIMEOUTWAIT);
					//State = CAD_DETECT; //@editShees
					State = RX; //@forembedded shees
					break;
				}
				case RX_ERROR:
				{
					PRINTF("Receive Failed RX_ERROR\n");
					HAL_Delay(TIMEOUTWAIT);
					//State = CAD_DETECT; //@editShees
					State = RX;//@forembedded shees
					break;
				}
				case CAD_DETECT:
				{
					PRINTF("CAD DETECT RUN\n");
					State=WAITFORCADAVAILABLE;
					Radio.StartCad();
					HAL_Delay(2000);
					PRINTF("CAD START ENDED\n");
					break;
				}
				case CAD_DETECTED:
				{
					PRINTF("CAD DETECTED RUN\n");
					State=RX;
					break;
				}
				case WAITFORCADAVAILABLE:
				{
					PRINTF("Waiting CAD AVAILABLE\n");
					HAL_Delay(CADAVAILABLEWAIT);
					State = CAD_DETECT;
					break;
				}
				
				case LOWPOWER:
				{
					PRINTF("LOWPOWER\n");
					Radio.Sleep();
					//HAL_Delay(LOWPOWERWAIT); //@forembedded shees continuous receive
					State = RX; //@editShees
					break;
				}
				default:
					State=LOWPOWER;
			}
			////////////////////////
			   DISABLE_IRQ( );
    /* if an interupt has occured after __disable_irq, it is kept pending 
     * and cortex will not enter low power anyway  */
    if (State == LOWPOWER)
    {
#ifndef LOW_POWER_DISABLE
      LowPower_Handler( );
#endif
    }
    ENABLE_IRQ( );
       
		}
		}
		else
			PRINTF("PLEASE DEFINE A MODE OF OPERATION, ENDNODE or GATEWAY(NOT_ENDNODE)");

}


/*LoRa function define begin*/
void OnTxDone( void )
{
    Radio.Sleep( );
    State = TX;
 //   PRINTF("OnTxDone\n");
}

void OnRxDone( uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr )
{
    Radio.Sleep( );
    BufferSize = size;
    memcpy( Buffer, payload, BufferSize );
    RssiValue = rssi;
    SnrValue = snr;
    State = RX;
  
    PRINTF("OnRxDone\n");
    PRINTF("RssiValue=%d dBm, SnrValue=%d\n", rssi, snr);
#if 1  //@shees
    PRINTF("RxData: ");
    for (int i=0; i<BufferSize; i++)
    {
      PRINTF("%d", Buffer[i]); //used %c for character print @shees 
			PRINTF(",");
    }
		//PRINTF("%d", Buffer[0]); //@editShees
    PRINTF("\n");
#endif
}

void OnTxTimeout( void )
{
    Radio.Sleep( );
    State = TX_TIMEOUT;
  
    //PRINTF("OnTxTimeout\n");
}

void OnRxTimeout( void )
{
    Radio.Sleep( );
    State = RX_TIMEOUT;
    //PRINTF("OnRxTimeout\n");
}

void OnRxError( void )
{
    Radio.Sleep( );
    State = RX_ERROR;
    //PRINTF("OnRxError\n");
}

void OnCADDetect(bool channelActivityDetected) //@Shees
{
	//PRINTF("OnCADDetected Function called\n");
	if(channelActivityDetected)
	{
		State = CAD_DETECTED;
   // PRINTF("OnCADDetected\n");
	}
	else ;//PRINTF("No CAD DETECTED\n");
}
///////////////////Insert number to character convertor here using parameter type self detect function////////////

uint8_t* putinbuffer(uint8_t buffer[],uint16_t Buffersize, uint8_t* data) //optimize by using pointers
{
	uint16_t i;
	buffer[0]=NETWORK_KEY;
	buffer[1]=RADIO_ID;
	buffer[2]=flags;
	i=3;
	for(;i<((sizeof(data)/sizeof(uint8_t))-1);i++)
	{
		buffer[i]=data[i-3];
		if(i>(Buffersize-1))
			break;
	}
	while (i<Buffersize)
	{
		buffer[i]=testvar;
		i++;
	}
	testvar++;
	return buffer;
}

/*LoRa function define end*/