/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32l4xx_hal.h"
#include "stdbool.h"
#include"assert.h"
#include "string.h"
#include "VL53L3CX_ULP_api.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define RFM9x_VER 0x12 // chip version can be read from ver register


#ifndef RFM95_SPI_TIMEOUT
#define RFM95_SPI_TIMEOUT 10
#endif

#ifndef RFM95_WAKEUP_TIMEOUT
#define RFM95_WAKEUP_TIMEOUT 10
#endif

#ifndef RFM95_SEND_TIMEOUT
#define RFM95_SEND_TIMEOUT 150
#endif

#ifndef RFM95_RECEIVE_TIMEOUT
#define RFM95_RECEIVE_TIMEOUT 1000
#endif

#ifndef RX_CONTINUOUS_TIMEOUT
#define RX_CONTINUOUS_TIMEOUT 2500
#endif


#define no_check 10
#define check 0
#define object_detected 1

#define SetUp_mode 10
#define Monitoring_mode 20

#define sending 1
#define service_channel 0
/**
 * LoRa Registers addresses read page 90 and page 108 on SX1276 data sheet
 Note: Reg function and name is different for each modulation scheme!
 */
typedef enum
{
	RFM95_REGISTER_FIFO_ACCESS = 0x00,
	RFM95_REGISTER_OP_MODE = 0x01,
	RFM95_REGISTER_FR_MSB = 0x06,
	RFM95_REGISTER_FR_MID = 0x07,
	RFM95_REGISTER_FR_LSB = 0x08,
	RFM95_REGISTER_PA_CONFIG = 0x09,
	RFM95_REGISTER_LNA = 0x0C,
	RFM95_REGISTER_FIFO_ADDR_PTR = 0x0D,
	RFM95_REGISTER_FIFO_TX_BASE_ADDR = 0x0E,
	RFM95_REGISTER_FIFO_RX_BASE_ADDR = 0x0F,
	RFM95_REGISTER_IRQ_FLAGS = 0x12,
	RFM95_REGISTER_FIFO_RX_BYTES_NB = 0x13,
	RFM95_REGISTER_PACKET_SNR = 0x19,
	RFM95_REGISTER_MODEM_CONFIG_1 = 0x1D,//configures the lore modem freq, SF, coding rate...
	RFM95_REGISTER_MODEM_CONFIG_2 = 0x1E,//same here
	RFM95_REGISTER_SYMB_TIMEOUT_LSB = 0x1F,
	RFM95_REGISTER_PREAMBLE_MSB = 0x20,
	RFM95_REGISTER_PREAMBLE_LSB = 0x21,
	RFM95_REGISTER_PAYLOAD_LENGTH = 0x22,
	RFM95_REGISTER_MAX_PAYLOAD_LENGTH = 0x23,
	RFM95_REGISTER_REG_OSC = 0x24,
	RFM95_REGISTER_MODEM_CONFIG_3 = 0x26,
	RFM95_REGISTER_INVERT_IQ_1 = 0x33,
	RFM95_REGISTER_SYNC_WORD = 0x39,
	RFM95_REGISTER_INVERT_IQ_2 = 0x3B,
	RFM95_REGISTER_DIO_MAPPING_1 = 0x40,
	RFM95_REGISTER_VERSION = 0x42,
	RFM95_REGISTER_PA_DAC = 0x4D,
	RegHopPeriod = 0x24,
	REG_DETECT_THRESHOLD = 0x37

} rfm95_register_t;

#define RFM95_REGISTER_OP_MODE_SLEEP                            0x00
#define RFM95_REGISTER_OP_MODE_LORA_SLEEP                       0x80
#define RFM95_REGISTER_OP_MODE_LORA_STANDBY                     0x81
#define RFM95_REGISTER_OP_MODE_LORA_TX                          0x83
#define RFM95_REGISTER_OP_MODE_LORA_RX_SINGLE                   0x86
#define RFM95_REGISTER_OP_MODE_LORA_RX_CONTI               0x85
#define RFM95_REGISTER_OP_MODE_LORA_CAD              			0x87

#define RFM95_REGISTER_PA_DAC_LOW_POWER                         0x84
#define RFM95_REGISTER_PA_DAC_HIGH_POWER                        0x87

#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE             0x40
#define RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE             0x00

#define RFM95_REGISTER_CLK_OUT_OFF           					 0x07

#define RFM95_REGISTER_INVERT_IQ_1_TX                    		    0x26
#define RFM95_REGISTER_INVERT_IQ_2_TX							              0x1d

#define RFM95_REGISTER_INVERT_IQ_1_RX                    		    0x67
#define RFM95_REGISTER_INVERT_IQ_2_RX						0x19
#define RegHopPeriod 		0x24
#define REG_DETECT_THRESHOLD 0x37


#define RFM95_INTERRUPT_COUNT 3//number of interrupt lines
#define No_Pin 0x41000

typedef enum
{
	RFM95_INTERRUPT_DIO0 ,
	RFM95_INTERRUPT_DIO1 ,
	RFM95_INTERRUPT_DIO5

} rfm95_interrupt_t;

typedef enum
{
	RFM95_RECEIVE_MODE_NONE,// no reciever
	RFM95_RECEIVE_MODE_RX1_ONLY,// recieve only with RX1 time target
	RFM95_RECEIVE_MODE_RX12,// receive with RX1 time target, of fails use the longer RX2 time target
	RFM95_RECEIVE_MODE_CONTINUOUS,//continously fills the data buffer but generates a an RX done flag, no timeout interrupt for the module in rx mode
	FM95_RECEIVE_MODE_CAD//detects if a valid Lora preable is detetc

} rfm95_receive_mode_t;


typedef struct
{// the elements of union share the same address space, thus in this case buffer union is just used to pass the contetnts of the whole struct.
	union {//struct used the bit field number": 1" as to compact the size to given number if bits
		struct {
			uint8_t output_power : 4;
			uint8_t max_power : 3;
			uint8_t pa_select : 1;
		};
		uint8_t buffer;
	};
} rfm95_register_pa_config_t;

typedef struct {

	uint32_t frequency;//frequency band give in Hz

} rfm95_channel_config_t;

typedef struct {

	/**
	 * MAGIC
	 */
	uint16_t magic;

	/**
	 * The current RX frame counter value.
	 */
	uint16_t rx_frame_count;

	/**
	 * The current TX frame counter value.
	 */
	uint16_t tx_frame_count;

	/**
	 * The delay to the RX1 window, default is 1
	 */
	uint8_t rx1_delay ;

	/**
	 * The configuration of channels, option of 16 chanel with diffrent frequencies
	 */
	rfm95_channel_config_t channels[16];

	/**
	 * Mask defining which channels are configured.
	 */
	uint16_t channel_mask;

} rfm95_eeprom_config_t;




typedef void (*rfm95_on_after_interrupts_configured)();


typedef void (*rfm95_precision_sleep_until)(uint32_t ticks_target);

typedef uint8_t (*rfm95_random_int)(uint8_t max);
typedef uint8_t (*rfm95_get_battery_level)();


typedef struct {

	/**
	 * The handle to the SPI bus for the device.
	 */
	SPI_HandleTypeDef *spi_handle;

	/**
	 * The port of the NSS pin.
	 */
	GPIO_TypeDef *nss_port;

	/**
	 * The NSS pin.
	 */
	uint16_t nss_pin;

		/**
	 * The DIO0 pin.
	 */
	uint16_t DIO0_pin;

	/**
	 * The DIO1 pin.
	 */
	uint16_t DIO1_pin;

	/**
	 * The DIO5 pin.
	 */
	uint16_t DIO5_pin;

	/**
	 * The port of the RST pin.
	 */
	GPIO_TypeDef *nrst_port;

	/**
	 * The RST pin.
	 */
	uint16_t nrst_pin;

	/**
	 * The device address for the LoraWAN
	 */
	uint8_t source_address[4];

	/**
	* The address of the destination (gateway)
	 */

	uint8_t destination_address[4];

	/**
	 * The network session key for ABP activation with the LoraWAN
	 */
	uint8_t network_session_key[16];

	/**
	 * The application session key for ABP activation with the LoraWAN
	 */
	uint8_t application_session_key[16];

	/**
	 * The frequency of the precision tick in Hz.
	 */
	uint32_t precision_tick_frequency;

	/**
	 * The +/- timing drift per second in nanoseconds.
	 */
	uint32_t precision_tick_drift_us_per_s;

	int8_t recieved_snr;

	/**
	 * The receive mode to operate at.
	 */
	rfm95_receive_mode_t receive_mode;



	/**
	 * Function that provides a precise sleep until a given tick count is reached.
	 */
	rfm95_precision_sleep_until precision_sleep_until;

	/**
	 * Function that provides a random integer.
	 */
	rfm95_random_int random_int;

	/**
	 * Function that returns the device's battery level.
	 */
	rfm95_get_battery_level get_battery_level;


	/**
	 * Callback called after the interrupt functions have been properly configred;
	 */
	rfm95_on_after_interrupts_configured on_after_interrupts_configured;


/**
	 * The config saved into the eeprom!! No eemprom so used to hold the channel frequencies
	 */
	rfm95_eeprom_config_t config;


	/**
	 * Tick values when each interrupt was called.
	 */
	volatile uint32_t interrupt_times[RFM95_INTERRUPT_COUNT];


	bool status;

} rfm95_handle_t;

typedef struct {

			uint16_t dev_address;

			uint16_t sensor_id;

			int status;



			uint8_t mes_status;

			uint16_t distance;

			uint16_t sigma;

			uint16_t signal_kcps;

			uint16_t ambient_kcps;

} VL53L3CX_handle_struct;


volatile uint8_t IntCount = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;

ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void sleep (void){
		HAL_SuspendTick();
		//could make it so it uses a lower power mode
		HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
		SystemClock_Config();
		HAL_ResumeTick();

}



//code works well but should be more precise and pass float variables //could add the option off passing a prescale value
void sleep_precise_s(uint8_t seconds){
	htim2.Instance = TIM2;
  	htim2.Init.Prescaler = 15;

  	htim2.Init.Period = ((seconds * HAL_RCC_GetPCLK1Freq())/(htim2.Init.Prescaler+1))-1;

  	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  	{
    	Error_Handler();
  	}

  	HAL_GetTick();
	HAL_TIM_Base_Start_IT(&htim2);

	sleep();


}

void sleep_precise_ms(uint16_t mili_seconds){
	htim2.Instance = TIM2;
  	htim2.Init.Prescaler = 15;

  	htim2.Init.Period = (((mili_seconds * HAL_RCC_GetPCLK1Freq())/((htim2.Init.Prescaler+1)*1000))-1);

  	if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  	{
    	Error_Handler();
  	}

  	HAL_GetTick();
	HAL_TIM_Base_Start_IT(&htim2);
	sleep();

}


uint16_t rfm95_interrupt_poll (uint16_t GPIO_Pin, uint8_t check_flag, uint8_t clear_all_flag){

	static uint16_t interupt_num[3];

	 if(clear_all_flag == 10){

		 interupt_num[0] = 0;
		 interupt_num[1] = 0;
		 interupt_num[2] = 0;

		 return 1;
	 }

		interupt_num[0] = (check_flag  == check)*interupt_num[0] + (GPIO_Pin == RFM95_DIO0_Pin && check_flag  != check) * HAL_GetTick();
		interupt_num[1] = (check_flag  == check)*interupt_num[1] + (GPIO_Pin == RFM95_DIO1_Pin && check_flag  != check) * HAL_GetTick();
		interupt_num[2] = (check_flag  == check)*interupt_num[2] + (GPIO_Pin == RFM95_DIO5_Pin && check_flag  != check) * HAL_GetTick();


		return (GPIO_Pin == RFM95_DIO0_Pin)*interupt_num[0] + (GPIO_Pin == RFM95_DIO1_Pin)*interupt_num[1]+ (GPIO_Pin == RFM95_DIO5_Pin)*interupt_num[2];

}


static bool read_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t *buffer, size_t length)
{
	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);
	HAL_Delay(1);

	uint8_t transmit_buffer = (uint8_t)reg & 0x7fu;

	if (HAL_SPI_Transmit(handle->spi_handle, &transmit_buffer, 1, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	if (HAL_SPI_Receive(handle->spi_handle, buffer, length, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

	return true;
}

static bool write_register(rfm95_handle_t *handle, rfm95_register_t reg, uint8_t value)
{
	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_RESET);
	HAL_Delay(1);

	uint8_t transmit_buffer[2] = {((uint8_t)reg | 0x80u), value};

	if (HAL_SPI_Transmit(handle->spi_handle, transmit_buffer, 2, RFM95_SPI_TIMEOUT) != HAL_OK) {
		return false;
	}

	HAL_GPIO_WritePin(handle->nss_port, handle->nss_pin, GPIO_PIN_SET);

	return true;
}

//sets a num,ber of diffrent channel bands
//static void config_set_channel(rfm95_handle_t *handle, uint8_t channel_index, uint32_t frequency)
//{
//	assert(channel_index < 16);
//	handle->config.channels[channel_index].frequency = frequency;
//	handle->config.channel_mask |= (1 << channel_index);
//}



static void reset(rfm95_handle_t *handle)//resets the RFM module
{
	HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_RESET);
	HAL_Delay(1); // 0.1ms would theoretically be enough
	HAL_GPIO_WritePin(handle->nrst_port, handle->nrst_pin, GPIO_PIN_SET);
	HAL_Delay(5);
}

static bool configure_frequency(rfm95_handle_t *handle, uint32_t frequency)//caclulate the channel freq and configure module by sending binary to chip
{
	// FQ = (FRF * 32 Mhz) / (2 ^ 19)
	uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

	if (!write_register(handle, RFM95_REGISTER_FR_MSB, (uint8_t)(frf >> 16))) return false;
	if (!write_register(handle, RFM95_REGISTER_FR_MID, (uint8_t)(frf >> 8))) return false;
	if (!write_register(handle, RFM95_REGISTER_FR_LSB, (uint8_t)(frf >> 0))) return false;

	return true;
}

//a bit redundant if only one channel is used
static bool configure_channel(rfm95_handle_t *handle, size_t channel_index)// configures a specific channel freq
{
	return configure_frequency(handle, handle->config.channels[channel_index].frequency);
}

static bool wait_for_irq(rfm95_handle_t *handle, rfm95_interrupt_t interrupt, uint16_t GPIO_Pin ,uint32_t timeout_ms)
{	// need to check if the get tick works correctly
	uint32_t timeout_tick = HAL_GetTick() +  timeout_ms * ((handle->precision_tick_frequency *1000) / 1000);

	while ((handle->interrupt_times[interrupt] = rfm95_interrupt_poll ( GPIO_Pin,check,0 ) ) == 0) {
		if (HAL_GetTick() >= timeout_tick) {
			return false;
		}
	}

	return true;
}


static bool wait_for_rx_irqs_long_timeout(rfm95_handle_t *handle, uint16_t GPIO_Pin ){
	 uint32_t timeout_tick = HAL_GetTick() +  10000 * ((handle->precision_tick_frequency * 1000) / 1000);

	while (((handle->interrupt_times[RFM95_INTERRUPT_DIO0] = rfm95_interrupt_poll ( GPIO_Pin,check,0 )) == 0)) {
		if (HAL_GetTick() >= timeout_tick) {
			return false;
		}
	}
	rfm95_interrupt_poll ( GPIO_Pin,check,10 );
	return handle->interrupt_times[RFM95_INTERRUPT_DIO0] != 0;
}

//static bool wait_for_rx_irqs(rfm95_handle_t *handle, uint16_t GPIO_Pin, uint16_t GPIO_Pin2)//generates interrupt from DIO0 pin when rx is done, or rx tinmeout, check pga 46, depeding on DIO mapping each pin has diffrent pin rise for IRQ flags
//{
//	uint32_t timeout_tick = HAL_GetTick() + RFM95_RECEIVE_TIMEOUT * ((handle->precision_tick_frequency*1000) / 1000);
//
//	while (((handle->interrupt_times[RFM95_INTERRUPT_DIO0] = rfm95_interrupt_poll ( GPIO_Pin,check,0 )) == 0) && ((handle->interrupt_times[RFM95_INTERRUPT_DIO1]= rfm95_interrupt_poll ( GPIO_Pin2,check,0 )) == 0)) {
//		if (HAL_GetTick() >= timeout_tick) {
//			return false;
//		}
//	}
//	rfm95_interrupt_poll ( GPIO_Pin,check,10 );
//	return handle->interrupt_times[RFM95_INTERRUPT_DIO0] != 0;
//}

bool rfm95_set_power(rfm95_handle_t *handle, int8_t power)// sets power ouptu of transmistters
{
	assert((power >= 2 && power <= 17) || power == 20);

	rfm95_register_pa_config_t pa_config = {0};
	uint8_t pa_dac_config = 0;

	if (power >= 2 && power <= 17) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = (power - 2);
		pa_dac_config = RFM95_REGISTER_PA_DAC_LOW_POWER;

	} else if (power == 20) {
		pa_config.max_power = 7;
		pa_config.pa_select = 1;
		pa_config.output_power = 15;
		pa_dac_config = RFM95_REGISTER_PA_DAC_HIGH_POWER;
	}

	if (!write_register(handle, RFM95_REGISTER_PA_CONFIG, pa_config.buffer)) return false;
	if (!write_register(handle, RFM95_REGISTER_PA_DAC, pa_dac_config)) return false;

	return true;
}

bool rfm95_init(rfm95_handle_t *handle)// iunitialise a rfm95 instance
{

  	reset(handle);

	// Check for correct version.
	uint8_t version = 0;
	if (!read_register(handle, RFM95_REGISTER_VERSION, &version, 1)) return false;
	if (version != RFM9x_VER) return false;

	//turns of the clk out on DIO 5 pin
	if (!write_register(handle, RFM95_REGISTER_REG_OSC, RFM95_REGISTER_CLK_OUT_OFF)) return false;

	//enable interrupt line after disabling clk output on that DIO5 line.


	// Module must be placed in sleep mode before switching to lora.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_SLEEP)) return false;

	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	// Default interrupt configuration, must be done to prevent DIO5 clock interrupts at 1Mhz
	if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;


	//no hopping
	if (!write_register(handle, RegHopPeriod, 0x00)) return false;

	// if (!write_register(handle, 0x31, 0xC3)) return false;
	// if (!write_register(handle, REG_DETECT_THRESHOLD, 0x0A)) return false;

	// Set module power to 5dbm.
	if (!rfm95_set_power(handle, 5)) return false;

	// Set LNA to the highest gain with 150% boost.
	if (!write_register(handle, RFM95_REGISTER_LNA, 0x23)) return false;

	// Preamble set to 31 + 4.25 = 35.25 symbols.
	if (!write_register(handle, RFM95_REGISTER_PREAMBLE_MSB, 0x00)) return false;

	if (!write_register(handle, RFM95_REGISTER_PREAMBLE_LSB, 0x1F)) return false;

	//playing with the sync word casues problems, better to leave it to the default 0x34
	// Set TTN sync word 0x34.
  //When the programmed Sync word is detected the user can assume that this incoming packet is for the node and can be processed accordingly
//	if (!write_register(handle, RFM95_REGISTER_SYNC_WORD, 0x12)) return false;



	// Set up TX and RX FIFO base addresses.
	if (!write_register(handle, RFM95_REGISTER_FIFO_TX_BASE_ADDR, 0x80)) return false;

	if (!write_register(handle, RFM95_REGISTER_FIFO_RX_BASE_ADDR, 0x00)) return false;


	// Maximum payload length of the RFM95 is 64.
	if (!write_register(handle, RFM95_REGISTER_MAX_PAYLOAD_LENGTH, 64)) return false;

	// Let module sleep after initialisation.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	return true;
}

//sleeps until the given time target
//static bool receive_at_scheduled_time(rfm95_handle_t *handle, uint32_t scheduled_time)// make the modle have a recieve time window
//{
//	// Sleep until 1ms before the scheduled time. need to call that function//replace with sleep functions
//	handle->precision_sleep_until(scheduled_time -((handle->precision_tick_frequency*1000) / 1000));
//
//	// Clear flags and previous interrupt time, configure mapping for RX done.
//	if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;
//	if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return false;
//	handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;
//	handle->interrupt_times[RFM95_INTERRUPT_DIO1] = 0;
//	handle->interrupt_times[RFM95_INTERRUPT_DIO5] = 0;
//
//	// Move modem to lora standby.
//	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return false;
//
//	// Wait for the modem to be ready.
//if(!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, handle->DIO5_pin,RFM95_WAKEUP_TIMEOUT)) return false;
//
//	// Now sleep until the real scheduled time.//replace with sleep functions
//	handle->precision_sleep_until(scheduled_time);
//
//	// could change it to contionous for testing, note timeout timer in the module is still active need to change if longer time out.
//	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_RX_SINGLE)) return false;
//
//	return true;
//}

////calculates the time target and the number of symbols//check the tick_frequency calculation
//static void calculate_rx_timings(rfm95_handle_t *handle, uint32_t bw, uint8_t sf, uint32_t tx_ticks,
//                                 uint32_t *rx_target, uint32_t *rx_window_symbols)
//{
//	//2<<(sf-1) is equivalent to 1<<sf = 2^sf
//	volatile int32_t symbol_rate_us = (int32_t)(((2 << (sf - 1)) * 1000000) / bw);
//// depends on the drift of the clock source, check the data sheet of the RC clock for STM32L412, ppm times the symbol rate to get the error drift within the period
//	volatile int32_t rx_timing_error_us = (int32_t)(/* handle->precision_tick_drift_us_per_s */ 0.01 * handle->config.rx1_delay);
//	volatile int32_t rx_window_us = 2 * symbol_rate_us + 2 * rx_timing_error_us;
//	volatile int32_t rx_offset_us = 4 * symbol_rate_us - (rx_timing_error_us / 2);
//	volatile int32_t rx_offset_ticks = (int32_t)(((int64_t)rx_offset_us * (int64_t)handle->precision_tick_frequency) / 1000000);
//	//uses TX_ticks time  to calculate the number of RX ticks(time till recieving reply) since sending the uplink
//	*rx_target = tx_ticks + handle->precision_tick_frequency * handle->config.rx1_delay + rx_offset_ticks;
//	*rx_window_symbols = rx_window_us / symbol_rate_us;
//}

//static bool receive_package(rfm95_handle_t *handle, uint32_t tx_ticks, uint8_t *payload_buf, size_t *payload_len, int8_t *snr)
//{
//	*payload_len = 0;
//
//	uint32_t rx1_target, rx1_window_symbols;
//	//calculates the RX target time, prpagation time, etc.
//	calculate_rx_timings(handle, 125000, 7, tx_ticks, &rx1_target, &rx1_window_symbols);
//
//	assert(rx1_window_symbols <= 0x3ff);
//
//	// Configure modem (125kHz, 4/6 error coding rate, SF7, single packet, CRC enable, AGC auto on)
//	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x72)) return false;
//	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0x74 | ((rx1_window_symbols >> 8) & 0x3))) return false;
//	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, 0x04)) return false;
//
//	// Set maximum symbol timeout.
//	if (!write_register(handle, RFM95_REGISTER_SYMB_TIMEOUT_LSB, rx1_window_symbols)) return false;
//
//	// Set IQ registers according to AN1200.24.//NOTICE
//	if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_1, RFM95_REGISTER_INVERT_IQ_1_RX)) return false;
//	if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_2, RFM95_REGISTER_INVERT_IQ_2_RX)) return false;
//
//
//	//sleeps until the calculated RX time target and then wakes up 1 ms before and does the configureation, then waits for a RXIRQ flag
//	receive_at_scheduled_time(handle, rx1_target);
//
//	// If there was nothing received during RX1, try RX2 longer MCU sleep till reception, module timeout is still the same.
//	if (!wait_for_rx_irqs(handle, handle->DIO0_pin, handle->DIO1_pin)) {
//
//		// Return modem to sleep.
//		if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;
//
//		if (handle->receive_mode == RFM95_RECEIVE_MODE_RX12) {
//
//			uint32_t rx2_target, rx2_window_symbols;
//			//calcultae timming with larger spreading factor, increase symbol rate and this time delay.
//			calculate_rx_timings(handle, 125000, 12, tx_ticks, &rx2_target, &rx2_window_symbols);
//
//			// Configure 869.525 MHz
//			//if (!configure_frequency(handle, 869525000)) return false;
//			if (!configure_frequency(handle, 868000000)) return false;
//
//			// Configure modem SF12
//			if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0xc2)) return false;
//			if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0x74 | ((rx2_window_symbols >> 8) & 0x3))) return false;
//			if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, 0x04)) return false;
//
//			// Set maximum symbol timeout.
//			if (!write_register(handle, RFM95_REGISTER_SYMB_TIMEOUT_LSB, rx2_window_symbols)) return false;
//
//			receive_at_scheduled_time(handle, rx2_target);
//
//			if (!wait_for_rx_irqs(handle,handle->DIO0_pin, handle->DIO1_pin)) {
//				// No payload during in RX1 and RX2
//				return true;
//			}
//		}
//
//		return true;
//	}
//
//	//reads CRC error flag
//	uint8_t irq_flags;
//	read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irq_flags, 1);
//
//	// Check if there was a CRC error. if true exit and disregard transmission
//	if (irq_flags & 0x20) {
//		return true;
//	}
//
//	handle->config.rx_frame_count++;
//
//	int8_t packet_snr;
//	if (!read_register(handle, RFM95_REGISTER_PACKET_SNR, (uint8_t *)&packet_snr, 1)) return false;
//	*snr = (int8_t)(packet_snr / 4);
//
//	// Read received payload length.
//	uint8_t payload_len_internal;
//	if (!read_register(handle, RFM95_REGISTER_FIFO_RX_BYTES_NB, &payload_len_internal, 1)) return false;
//
//	// Read received payload itself.
//	if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, 0)) return false;
//	if (!read_register(handle, RFM95_REGISTER_FIFO_ACCESS, payload_buf, payload_len_internal)) return false;
//
//	// Return modem to sleep.
//	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;
//
//	// Successful payload receive, set payload length to tell caller.
//	*payload_len = payload_len_internal;
//	return true;
//}

static bool recieve_packets(rfm95_handle_t *handle, uint8_t *payload_buf, size_t *payload_len, int8_t *snr, uint8_t channel)
{
*payload_len = 0;



		if (!configure_channel(handle, channel)) return false;

		// Configure modem (125kHz, 4/5 error coding rate, SF6, single packet, CRC disabled, Explicit header,AGC auto on,  max timeout
		if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x72)) return false;

		if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0x60)) return false;
		if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, 0x04)) return false;



		// Set IQ registers according to AN1200.24.//NOTICE//for receiving uses inverted IQ, could be 0x66 or 0x67;

//		if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_1, 0x26)) return false;
//			if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_2, 0x1D)) return false;

		//sets the max timeout for the RX single timeout
		if (!write_register(handle, RFM95_REGISTER_SYMB_TIMEOUT_LSB, 0xFF)) return false;

		//sleeps until the calculated RX time target and then wakes up 1 ms before and does the configureation, then waits for a RXIRQ flag
		if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_RXDONE)) return false;
		if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return false;
		handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;
		handle->interrupt_times[RFM95_INTERRUPT_DIO1] = 0;
		handle->interrupt_times[RFM95_INTERRUPT_DIO5] = 0;

		if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return false;



		if(!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, handle->DIO5_pin,RFM95_WAKEUP_TIMEOUT)) return false;
		if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return false;
		handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;
		handle->interrupt_times[RFM95_INTERRUPT_DIO1] = 0;
		handle->interrupt_times[RFM95_INTERRUPT_DIO5] = 0;



	//uint8_t flag_check = 0;
	// could change it to contionous for testing, note timeout timer in the module is still active need to change if longer time out.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_RX_CONTI)) return false;
	//if(!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, handle->DIO5_pin,RFM95_WAKEUP_TIMEOUT)) return false;


	// If there was nothing received during RX1, try RX2 longer MCU sleep till reception, module timeout is still the same.
	if (!wait_for_rx_irqs_long_timeout(handle, handle->DIO0_pin))return false;
	//wait for the rx done, then check for the crc error if yes wait for a new rx done, if no read data




	//reads CRC error flag
	uint8_t irq_flags;
	read_register(handle, RFM95_REGISTER_IRQ_FLAGS, &irq_flags, 1);

	// Check if there was a CRC error. if true exit and disregard transmission
	if (irq_flags & 0x20) {
		return true;
	}

	handle->config.rx_frame_count++;

	int8_t packet_snr;
	if (!read_register(handle, RFM95_REGISTER_PACKET_SNR, (uint8_t *)&packet_snr, 1)) return false;
	*snr = (int8_t)(packet_snr / 4);

	// Read received payload length.
	uint8_t payload_len_internal;
	if (!read_register(handle, RFM95_REGISTER_FIFO_RX_BYTES_NB, &payload_len_internal, 1)) return false;

	// Read received payload itself.
	if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, 0)) return false;
	if (!read_register(handle, RFM95_REGISTER_FIFO_ACCESS, payload_buf, payload_len_internal)) return false;

	// Return modem to sleep.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	// Successful payload receive, set payload length to tell caller.
	*payload_len = payload_len_internal;
	return true;

}

static bool send_package(rfm95_handle_t *handle, uint8_t *payload_buf, size_t payload_len, uint8_t channel
                         )
{
	// Configure channel for transmission.
	if (!configure_channel(handle, channel)) return false;

	// Configure modem (125kHz, 4/5 error coding rate, SF6, single packet, CRC disable)
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_1, 0x72)) return false;
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_2, 0x60)) return false;
	if (!write_register(handle, RFM95_REGISTER_MODEM_CONFIG_3, 0x04)) return false;

//	// Set IQ registers according to AN1200.24.
//	if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_1, RFM95_REGISTER_INVERT_IQ_1_TX)) return false;
//	if (!write_register(handle, RFM95_REGISTER_INVERT_IQ_2, RFM95_REGISTER_INVERT_IQ_2_TX)) return false;

	// Set the payload length.
	if (!write_register(handle, RFM95_REGISTER_PAYLOAD_LENGTH, payload_len)) return false;

	// Enable tx-done interrupt, clear flags and previous interrupt time.
	if (!write_register(handle, RFM95_REGISTER_DIO_MAPPING_1, RFM95_REGISTER_DIO_MAPPING_1_IRQ_FOR_TXDONE)) return false;
	if (!write_register(handle, RFM95_REGISTER_IRQ_FLAGS, 0xff)) return false;
	handle->interrupt_times[RFM95_INTERRUPT_DIO0] = 0;	//cheks how many interrupts have happed or time since interruprt
	handle->interrupt_times[RFM95_INTERRUPT_DIO5] = 0;

	// Move modem to lora standby.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_STANDBY)) return false;

	// Wait for the modem to be ready.
	if(!wait_for_irq(handle, RFM95_INTERRUPT_DIO5, handle->DIO5_pin,RFM95_WAKEUP_TIMEOUT)) return false;

	// Set pointer to start of TX section in FIFO.
	if (!write_register(handle, RFM95_REGISTER_FIFO_ADDR_PTR, 0x80)) return false;

	// Write payload to FIFO.
	for (size_t i = 0; i < payload_len; i++) {
		write_register(handle, RFM95_REGISTER_FIFO_ACCESS, payload_buf[i]);
	}

	// Set modem to tx mode.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_TX)) return false;


	// Wait for the transfer complete interrupt.
	if (!wait_for_irq(handle, RFM95_INTERRUPT_DIO0, handle->DIO0_pin,RFM95_SEND_TIMEOUT)) return false;

	// Set real tx time in ticks// gives the tick time at which TX is done


	// Return modem to sleep.
	if (!write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP)) return false;

	// Increment tx frame counter.
	handle->config.tx_frame_count++;

	return true;
}




////You can modify send data itself but the object pointed to by send_data shall not be modified, so if i want to ge the recievd packet i could return it.
//bool rfm95_send_recive(rfm95_handle_t *handle,  uint8_t send_data[], size_t send_data_length, uint8_t channel)
//{
//	uint8_t phy_payload_buf[64] = {0};
//
//
//	// Build the up-link phy payload.
//	// size_t phy_payload_len = encode_phy_payload(handle, phy_payload_buf, send_data, send_data_length, 1);
//
//	//builds the datagram of the message
//size_t phy_payload_len = 0; //build_datagram(handle, phy_payload_buf, send_data, send_data_length);
//
////	size_t phy_payload_len = 64;
//
//	//defult is rindom channnel but for this case it is the second channel with freq 868 Mhz
//	uint8_t random_channel = 1;// select_random_channel(handle);
//
//
//
//	// Send the requested up-link.
//	if (!send_package(handle, phy_payload_buf, phy_payload_len, random_channel)) {
//		write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP);
//		return false;
//	}
//
//	//Only receive if configured to do so.
//	if(handle->receive_mode == RFM95_RECEIVE_MODE_NONE) {
//		return true;
//	}else{
//	// Clear phy payload buffer to reuse for the down-link message.
//		memset(phy_payload_buf, 0x00, sizeof(phy_payload_buf));
//		phy_payload_len = 0;
//
//
//	 	int8_t snr;
//
//	//Try receiving a down-link.
//		if (!recieve_packets(handle,  phy_payload_buf, &phy_payload_len, &snr, channel)) {
//			write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP);
//
//			return false;
//		}
//
//		 //Any RX payload was received decode the datagram
//		if (phy_payload_len != 0) {
//
//
//
//		 }
//	 }
//
//
//
//	return true;
//}

//takes about half a second from buidling payload, tx configuring till sending.
bool rfm95_send_test(rfm95_handle_t *handle,  uint8_t send_data[10], size_t send_data_length, uint8_t channel){
	uint8_t phy_payload_buf[64] = {0};
	uint8_t phy_payload_len = 0;



	// 64 bytes is maximum size of the payload can chage in register config
	assert(send_data_length + 4 + 9 <= 64);
	int i = 0;
	for(; i < 4; i++){
		phy_payload_buf[i] = handle->source_address[i];
		phy_payload_len += 1;
	}

	for(; i < 8; i++){
		phy_payload_buf[i] = handle->destination_address[i-4];
		phy_payload_len += 1;
	}
	//sets the number of hops intil being disregarded

	for(; i < 8 + send_data_length; i++){
		phy_payload_buf[i] = send_data[i-8];
		phy_payload_len += 1;
	}



	//defult is rindom channnel but for this case it is the second channel with freq 868 Mhz




	// Send the requested up-link.

	if (!send_package(handle, phy_payload_buf, phy_payload_len, channel)) {



		write_register(handle, RFM95_REGISTER_OP_MODE, RFM95_REGISTER_OP_MODE_LORA_SLEEP);
		return false;
	}
	return true;

}

bool rfm95_recive_test(rfm95_handle_t *handle, uint8_t *phy_payload_buf,uint8_t channel){

	size_t phy_payload_len = 0;
	if(handle->receive_mode == RFM95_RECEIVE_MODE_NONE) {
		return true;
	}else{
		int8_t snr = 0;



	//Try receiving a down-link.
		if(!recieve_packets(handle,  phy_payload_buf, &phy_payload_len, &snr, channel)) return false;

		handle->recieved_snr = snr;
		return true;

	}
}


void ToF_fault_reset(VL53L3CX_handle_struct *handle){
	uint8_t status_id = 1;
	uint8_t status_init = 1;

	for(int i = 0;status_id && status_init && i < 10;i++){//tries a few times to initilise


		 	 	 	HAL_GPIO_WritePin(VL53L3CX_XSHUT_GPIO_Port, VL53L3CX_XSHUT_Pin, GPIO_PIN_RESET);
		 	 	 	HAL_Delay(250);
		 	 	 	HAL_GPIO_WritePin(VL53L3CX_XSHUT_GPIO_Port, VL53L3CX_XSHUT_Pin, GPIO_PIN_SET);
		 	 	 	HAL_Delay(250);


		    		status_id = VL53L3CX_ULP_GetSensorId(handle->dev_address, &handle->sensor_id);
		    		status_init = VL53L3CX_ULP_SensorInit(handle->dev_address);




		    	}

	if(status_init ||status_id ){

			 HAL_NVIC_SystemReset();


	}else{

		VL53L3CX_ULP_SetInterruptConfiguration(handle->dev_address, 140, 1);

		// sets the equivalent integration time, higher value larger max distance
		 VL53L3CX_ULP_SetMacroTiming(handle->dev_address, 200);

		/* (Optional) Program a 10Hz ranging frequency */
		 VL53L3CX_ULP_SetInterMeasurementInMs(handle->dev_address, 50);

		/* (Optional) Enable all the SPADS (increase max distance but also power consumption) */
	 	 VL53L3CX_ULP_SetROI(handle->dev_address, 16);
		 VL53L3CX_ULP_ClearInterrupt(handle->dev_address);


	}


}






void RFM_fault_reset(rfm95_handle_t *handle){
	bool status_here = false;

	for(int i = 0;!status_here && i < 10;i++){//tries a few times to initilise
		 	 	 HAL_NVIC_DisableIRQ(EXTI1_IRQn);


		    		status_here = rfm95_init(handle);

		    		HAL_Delay(500);

		    	}

	if(!status_here){
			 HAL_NVIC_SystemReset();

	}
	 HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	 handle->status = status_here;


}



//updates the tick time when the interrupt happens
void HAL_GPIO_EXTI_Callback( uint16_t GPIO_Pin)

{

	if (GPIO_Pin == VL53L3CX_GPIO1_INT_Pin){

		IntCount = object_detected;

	}else{

		rfm95_interrupt_poll ( GPIO_Pin,no_check,0 );
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t data_request_fail= 0;
	uint8_t state = 0;
	uint8_t seconds = 30;//update approximate update period for the radio
	uint8_t sub_sleeps_seconds = 0;

	uint8_t sleeps_done = 0;

	uint8_t Operation_Mode = 0;
	uint8_t Config_SW = 0;
	uint8_t NEW_setup_flag = 0;

	uint8_t recieves_failed = 0;
	uint8_t battery_state = 0;

	uint8_t check_for_device_address = 0;


	uint8_t recieved_payload[64] = {0};
	uint8_t *recieve_pointer = recieved_payload;

	uint8_t data_packet[10] = {0};

	 rfm95_handle_t rfm95_handle = {
            .spi_handle = &hspi1,
    		.nss_port = RFM95_NSS_SPI_GPIO_Port,
    		.nss_pin = RFM95_NSS_SPI_Pin,
    		.nrst_port =RFM95_NRST_GPIO_Port,
    		.nrst_pin =  RFM95_NRST_Pin,
  		  .DIO0_pin = RFM95_DIO0_Pin,
   	  	  	.DIO5_pin = RFM95_DIO5_Pin,
   	  	  	  .DIO1_pin = RFM95_DIO1_Pin,
    		 .source_address = {
                 0x02, 0x01, 0x02, 0x01
             },
  		   .destination_address = {
  				0x03, 0x02, 0x03, 0x02

  		   },
    		.receive_mode = RFM95_RECEIVE_MODE_CONTINUOUS,
  		.status = false,

        };


        rfm95_handle.config.channels[sending].frequency = 869000000;
        rfm95_handle.config.channels[service_channel].frequency = 869000000;// make requency 868 Mhz
        rfm95_handle.precision_tick_frequency = HAL_GetTickFreq();
  		rfm95_handle.config.tx_frame_count = 0;
  		rfm95_handle.config.rx_frame_count = 0;
  		rfm95_handle.config.rx1_delay = 1;

  		VL53L3CX_handle_struct VL53L3CX_handle1 = {
  	  		.dev_address = 0x53,
  	  		.status = VL53L3CX_ULP_ERROR_NONE,
  	  	};


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */


  		VL53L3CX_handle1.status = VL53L3CX_ULP_GetSensorId(VL53L3CX_handle1.dev_address, &VL53L3CX_handle1.sensor_id);

  		if(VL53L3CX_handle1.status != VL53L3CX_ULP_ERROR_NONE){
  			ToF_fault_reset(&VL53L3CX_handle1);

  		}

  		VL53L3CX_handle1.status = VL53L3CX_ULP_SensorInit(VL53L3CX_handle1.dev_address);

  		if(VL53L3CX_handle1.status != VL53L3CX_ULP_ERROR_NONE){
  					ToF_fault_reset(&VL53L3CX_handle1);

  		}else{


  			/* (Optional) Program sensor to raise an interrupt ONLY below 300mm */
  			VL53L3CX_handle1.status = VL53L3CX_ULP_SetInterruptConfiguration(VL53L3CX_handle1.dev_address, 140, 1);

  			// sets the equivalent integration time, higher value larger max distance
  			VL53L3CX_handle1.status = VL53L3CX_ULP_SetMacroTiming(VL53L3CX_handle1.dev_address, 100);

  			/* (Optional) Program a 10Hz ranging frequency */
  			VL53L3CX_handle1.status = VL53L3CX_ULP_SetInterMeasurementInMs(VL53L3CX_handle1.dev_address, 400);

  			/* (Optional) Enable all the SPADS (increase max distance but also power consumption) */
  			VL53L3CX_handle1.status = VL53L3CX_ULP_SetROI(VL53L3CX_handle1.dev_address, 16);
  			VL53L3CX_handle1.status = VL53L3CX_ULP_ClearInterrupt(VL53L3CX_handle1.dev_address);
  	}


  		IntCount  = 0;

    	 // Initialise RFM95 module.




  		rfm95_handle.status = rfm95_init(&rfm95_handle);
  		if(rfm95_handle.status != true){

  			RFM_fault_reset(&rfm95_handle);

  		}


  	  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  	  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  	  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  	  IntCount  = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  if(NEW_setup_flag == 1){
		  check_for_device_address = (rfm95_handle.source_address[0] == recieved_payload[4]) + (rfm95_handle.source_address[1] == recieved_payload[5]) + (rfm95_handle.source_address[2] == recieved_payload[6]) + (rfm95_handle.source_address[3] == recieved_payload[7]);
		  if(check_for_device_address == 4){
			  check_for_device_address = 0;

		  for(int i = 0; i < 4  ;i++)
		  {
			  rfm95_handle.destination_address[i] = recieved_payload[i+8];

		  }
		  for(int i = 0; i < 4  ;i++)
		 {
		 		  rfm95_handle.source_address[i] = recieved_payload[i+12];

		 }

		 seconds = recieved_payload[16];


		  //more functions can be added here
		  NEW_setup_flag = 0;

		  for(int i = 0; i < 41  ;i++)//upon successful configuration flash status LED for 10 seconds
		  {
			  HAL_GPIO_WritePin(GPIOB,status_led_Pin, GPIO_PIN_RESET);
			  HAL_Delay(250);
			  HAL_GPIO_WritePin(GPIOB,status_led_Pin, GPIO_PIN_SET);
			  HAL_Delay(250);

		  }

	  }
	  }

	  Config_SW = HAL_GPIO_ReadPin(config_sw_GPIO_Port,config_sw_Pin);

	  Operation_Mode = (Config_SW)*SetUp_mode + (!Config_SW)*Monitoring_mode;

	 // Operation_Mode = Monitoring_mode;

switch(Operation_Mode)
{
case Monitoring_mode:

			recieves_failed = 0;
			HAL_GPIO_WritePin(GPIOB,status_led_Pin, GPIO_PIN_RESET);
    /* USER CODE END WHILE */
	  	  data_request_fail = 0;
	  	  sleeps_done = 0;



	  	  // split the sleeps in to sections which when wakes up it checks for the chnage in state, if state is change build populate data array and if time is still left go to sleep untill it is sending time


	  	  sub_sleeps_seconds = seconds/5; //value should be multiples of five

	  	  for(uint8_t i = 0; i < 6; i++){
	  		  VL53L3CX_handle1.status = VL53L3CX_ULP_StartRanging(VL53L3CX_handle1.dev_address);

	  		  if(VL53L3CX_handle1.status != VL53L3CX_ULP_ERROR_NONE){
	  			  	 ToF_fault_reset(&VL53L3CX_handle1);

	  		  }


	  		  VL53L3CX_handle1.status = VL53L3CX_ULP_ClearInterrupt(VL53L3CX_handle1.dev_address);
	  		if(VL53L3CX_handle1.status != VL53L3CX_ULP_ERROR_NONE){
	  			  	 ToF_fault_reset(&VL53L3CX_handle1);

	  		}

	  		HAL_Delay(500);
	  		VL53L3CX_ULP_StopRanging(VL53L3CX_handle1.dev_address);



	  		if(IntCount == object_detected){
	  			break; //exit loop when object was detected
	  		}


	  	  sleep_precise_s(sub_sleeps_seconds);


	  	  if(TIM2->CNT< 1000){
	  		  	  sleeps_done++;


	  	  		  HAL_TIM_Base_Stop_IT(&htim2);

	  	  	  }

	  	  }

	  	 HAL_TIM_Base_Stop_IT(&htim2);
	  	 VL53L3CX_handle1.status = VL53L3CX_ULP_StartRanging(VL53L3CX_handle1.dev_address);
	  	 HAL_Delay(500);//could be even less



	  	  VL53L3CX_handle1.status = VL53L3CX_ULP_DumpDebugData(VL53L3CX_handle1.dev_address, &VL53L3CX_handle1.mes_status ,&VL53L3CX_handle1.distance, &VL53L3CX_handle1.sigma, &VL53L3CX_handle1.signal_kcps, &VL53L3CX_handle1.ambient_kcps );

	  	  if(VL53L3CX_handle1.status != VL53L3CX_ULP_ERROR_NONE){

	  		   VL53L3CX_ULP_ClearInterrupt(VL53L3CX_handle1.dev_address);
	  		  data_request_fail = 1;

	  		  //if there was an error in the data request, attempt a few times
	  		  for(int i = 0; VL53L3CX_handle1.status != VL53L3CX_ULP_ERROR_NONE && i<6; i++){

	  			  	  HAL_Delay(100);
	  				  VL53L3CX_handle1.status = VL53L3CX_ULP_DumpDebugData(VL53L3CX_handle1.dev_address, &VL53L3CX_handle1.mes_status ,&VL53L3CX_handle1.distance, &VL53L3CX_handle1.sigma, &VL53L3CX_handle1.signal_kcps, &VL53L3CX_handle1.ambient_kcps );

	  			  }
	  			  data_request_fail = (VL53L3CX_handle1.status != VL53L3CX_ULP_ERROR_NONE);

	  	  	  }
	  	  	  // if detecected object is very close check a few times to seem if possible debris is still there.
	  	  	  if(VL53L3CX_handle1.distance <= 15){

	  	  		for(int i = 0; VL53L3CX_handle1.status == VL53L3CX_ULP_ERROR_NONE && i<10; i++){


	  	  					VL53L3CX_handle1.status = VL53L3CX_ULP_DumpDebugData(VL53L3CX_handle1.dev_address, &VL53L3CX_handle1.mes_status ,&VL53L3CX_handle1.distance, &VL53L3CX_handle1.sigma, &VL53L3CX_handle1.signal_kcps, &VL53L3CX_handle1.ambient_kcps );
	  	  					HAL_Delay(500);
	  	  		}
	  	  	  }


	  	   VL53L3CX_ULP_StopRanging(VL53L3CX_handle1.dev_address);
	  	   //during idle current is about 56 uA
	  	   VL53L3CX_ULP_ClearInterrupt(VL53L3CX_handle1.dev_address);
	  	   IntCount = 0;

	  	 //200 most licky a car is present, 100 means some trash is over the sensor, 50 means not car is present
	  	   state = (VL53L3CX_handle1.distance < 300 && VL53L3CX_handle1.distance > 40)*200 + (VL53L3CX_handle1.distance < 40)*100 + (VL53L3CX_handle1.distance > 300)*50 ;
	  	   state += data_request_fail*20;

	  		HAL_ADC_Start(&hadc1);
	  		HAL_ADC_PollForConversion(&hadc1, 1000);
	  		HAL_ADC_Stop(&hadc1);
	  		battery_state = (HAL_ADC_GetValue(&hadc1)>113)*100;// about 4.4v at 80% discharge (4.4/3)/(3.3/2^8) = 113.777



	  	  data_packet[0] = VL53L3CX_handle1.mes_status*!data_request_fail ;
	  	  data_packet[1] = (uint8_t) ((VL53L3CX_handle1.distance&255)*!data_request_fail);
	  	  data_packet[2] = (uint8_t) (((VL53L3CX_handle1.distance & 65280)>>8)*!data_request_fail);
	  	  data_packet[3] = (uint8_t) ((VL53L3CX_handle1.signal_kcps&255)*!data_request_fail);
	  	  data_packet[4] = (uint8_t) (((VL53L3CX_handle1.signal_kcps & 65280)>>8)*!data_request_fail);
	  	  data_packet[5] = (uint8_t) ((VL53L3CX_handle1.ambient_kcps&255)*!data_request_fail);
	  	  data_packet[6] = (uint8_t) (((VL53L3CX_handle1.ambient_kcps & 65280)>>8)*!data_request_fail);
	  	  data_packet[7] =  255*!data_request_fail;
	  	  data_packet[8] = state ;//200 most licky a car is present, 200 means some trash is over the sensor, above 300 means nothing is present
		  data_packet[9] = battery_state;



	      /* USER CODE END WHILE */


	      /* USER CODE BEGIN 3 */
	  		if(sleeps_done < 5){
	  			sleep_precise_s(sub_sleeps_seconds*(5-sleeps_done));
	  			HAL_TIM_Base_Stop_IT(&htim2);
	  		}



	  

	  	  for (int i = 0; i<2; i++){


	  		  rfm95_handle.status = rfm95_send_test(&rfm95_handle, data_packet, 10, sending);

	  		HAL_Delay(50);
	  	  	  if(rfm95_handle.status != true){
		


	  	  			RFM_fault_reset(&rfm95_handle);


	  	  	  }else{
	  	  		  break;
	  	  	  }
	  	  	  HAL_Delay(1000);

	    	  }

	    /* USER CODE END 3 */

	  	break;

case SetUp_mode:

		HAL_GPIO_WritePin(GPIOB,status_led_Pin, GPIO_PIN_SET);
		HAL_Delay(50);
		rfm95_handle.status = rfm95_recive_test(&rfm95_handle, recieve_pointer, service_channel);

		if(rfm95_handle.status != true){
			recieves_failed++;

		if(recieves_failed >= 20){
			RFM_fault_reset(&rfm95_handle);
			recieves_failed = 0;

		}

		}else{

			if(rfm95_handle.recieved_snr >= 7){
			recieves_failed = 0;
			NEW_setup_flag = 1;
			}
		}


		break;




	  }
    /* USER CODE BEGIN 3 */
  }
}
  /* USER CODE END 3 */


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_10;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3999999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RFM95_NSS_SPI_Pin|RFM95_NRST_Pin|status_led_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(VL53L3CX_XSHUT_GPIO_Port, VL53L3CX_XSHUT_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : RFM95_DIO5_Pin */
  GPIO_InitStruct.Pin = RFM95_DIO5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RFM95_DIO5_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RFM95_NSS_SPI_Pin */
  GPIO_InitStruct.Pin = RFM95_NSS_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(RFM95_NSS_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RFM95_NRST_Pin status_led_Pin */
  GPIO_InitStruct.Pin = RFM95_NRST_Pin|status_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RFM95_DIO0_Pin RFM95_DIO1_Pin RFM95_DIO2_Pin VL53L3CX_GPIO1_INT_Pin */
  GPIO_InitStruct.Pin = RFM95_DIO0_Pin|RFM95_DIO1_Pin|RFM95_DIO2_Pin|VL53L3CX_GPIO1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : config_sw_Pin */
  GPIO_InitStruct.Pin = config_sw_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(config_sw_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : VL53L3CX_XSHUT_Pin */
  GPIO_InitStruct.Pin = VL53L3CX_XSHUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(VL53L3CX_XSHUT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
