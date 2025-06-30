/*
 * api_motor_driver_tmc5160.c
 *
 *  Created on: Jun 28, 2025
 *      Author: vijji
 */
#include <stdio.h>
#include "api_motor_driver.h"
#include "api_motor_driver_tmc5160.h"
#include "stm32f4xx_hal_spi.h"
#include "main.h"

// Numeric Constants

static const uint8_t TMC5160_WRITE_ACCESS_BIT = 0x80;
static const uint8_t BYTE_MASK                = 0xFF;
static const uint8_t SHIFT_BYTE_3             = 24;
static const uint8_t SHIFT_BYTE_2             = 16;
static const uint8_t SHIFT_BYTE_1             = 8;
static const uint8_t SHIFT_BYTE_0             = 0;
static const uint32_t DATA_GSTAT_REGISTER_CLEAR_FLAGS = 0x00000007;
static const uint8_t TMC5160_IHOLD_MASK       = 0x1F;
static const uint8_t TMC5160_IRUN_MASK        = 0x1F;
static const uint8_t TMC5160_IHOLDDELAY_MASK  = 0x0F;
static const uint16_t FULL_STEPS_PER_REV         = 200;   // 360° / 1.8°(Motor data sheet)
static const uint16_t MICROSTEPS_PER_FULL_STEP   = 256;   // As per data sheet (default)
static const uint32_t MICROSTEPS_PER_REVOLUTION  = FULL_STEPS_PER_REV * MICROSTEPS_PER_FULL_STEP; // 51200
static const uint8_t  SECONDS_PER_MINUTE         = 60;


typedef enum e_api_motor_driver_tmc5160_register_address_t
{
	e_api_motor_driver_tmc5160_register_address_GCONF = 0x00,
	e_api_motor_driver_tmc5160_register_address_GSTAT = 0x01,
	e_api_motor_driver_tmc5160_register_address_IHOLD_IRUN = 0x10,
	e_api_motor_driver_tmc5160_register_address_VACTUAL = 0x22
}e_api_motor_driver_tmc5160_register_address_t;

// Function Prototypes
/*
 * @brief Initializes the motor driver.
 * @parameter[input] p_motor_driver The pointer to the motor driver.
 * @return The result of the operation
 *  */
HAL_StatusTypeDef api_motor_driver_tmc5160_initialize(const struct s_api_motor_driver_t * p_motor_driver);

/*
 * @brief Initializes the motor driver.
 * @parameter[input] p_motor_driver The pointer to the motor driver.
 * @parameter[input] rpm and direction The parameters required for the motor movement.
 * @return The result of the operation
 *  */
HAL_StatusTypeDef api_motor_driver_tmc5160_set_velocity(const struct s_api_motor_driver_t * p_motor_driver, int16_t rpm, e_api_motor_driver_motor_direction_t direction);

// Public Functions
// It takes the Register address and the 32bit data to be wriiten and gives the 40 bit data that to be send through SPI
void api_motor_driver_tmc5160_build_SPI_telegram(e_api_motor_driver_tmc5160_register_address_t reg_address, uint32_t data, uint8_t out_buffer[BYTE_SIZE])
{
    if (out_buffer == NULL)
        return;

     out_buffer[0] = reg_address | TMC5160_WRITE_ACCESS_BIT;   //set MSB as 1 for write access
     out_buffer[1] = (data >> SHIFT_BYTE_3) & BYTE_MASK;
     out_buffer[2] = (data >> SHIFT_BYTE_2) & BYTE_MASK;
     out_buffer[3] = (data >> SHIFT_BYTE_1) & BYTE_MASK;
     out_buffer[4] = (data >> SHIFT_BYTE_0) & BYTE_MASK;
}

// It takes the IHOLD, IRUN, IHOLDDELAY values and combines them to 32 bit data as per data sheet of TMC5160
uint32_t api_motor_driver_tmc5160_ihold_irun_value(uint8_t ihold, uint8_t irun, uint8_t iholddelay)
{
    uint32_t value = 0;

    value |= ((uint32_t)(ihold       & TMC5160_IHOLD_MASK)      << SHIFT_BYTE_0);
    value |= ((uint32_t)(irun        & TMC5160_IRUN_MASK)       << SHIFT_BYTE_1);
    value |= ((uint32_t)(iholddelay  & TMC5160_IHOLDDELAY_MASK) << SHIFT_BYTE_2);

    return value;
}

const s_api_motor_driver_t * s_api_motor_driver_tmc5160_try_construct(s_api_motor_driver_tmc5160_t * const p_self,
												                       const SPI_HandleTypeDef *hspi,
												                       const e_api_motor_driver_tmc5160_IHOLD_t I_HOLD,
												                       const e_api_motor_driver_tmc5160_IRUN_t I_RUN,
												                       const e_api_motor_driver_tmc5160_IHOLDDELAY_t I_HOLDDELAY)
{
	p_self->hspi = hspi;
	p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_idle;
	p_self->state_set_velocity = e_api_motor_driver_tmc5160_SPI_state_idle;
	p_self->I_HOLD = I_HOLD;
	p_self->I_RUN = I_RUN;
	p_self->I_HOLDDELAY = I_HOLDDELAY;

	p_self->api_motor_driver.initialize = api_motor_driver_tmc5160_initialize;
	p_self->api_motor_driver.set_velocity = api_motor_driver_tmc5160_set_velocity;
	return &(p_self->api_motor_driver);
}

// Private Functions

HAL_StatusTypeDef api_motor_driver_tmc5160_initialize(const struct s_api_motor_driver_t * p_motor_driver)
{
    if (p_motor_driver == NULL)
    {
        return HAL_ERROR;
    }
	s_api_motor_driver_tmc5160_t * p_self = (s_api_motor_driver_tmc5160_t*)p_motor_driver;
	HAL_StatusTypeDef result_state_machine = HAL_BUSY;
	HAL_StatusTypeDef result_internal = HAL_BUSY;
	switch(p_self -> state_initialize)
	{
		case e_api_motor_driver_tmc5160_SPI_state_idle:
			HAL_GPIO_WritePin(SD_Mode_GPIO_Port, SD_Mode_Pin, GPIO_PIN_RESET); // Pull SD_Mode LOW for selecting Mode 1 of TMC5160A, That is SPI Mode
			p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_acquiring_slave;
		case e_api_motor_driver_tmc5160_SPI_state_acquiring_slave:
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); // Pull CS LOW for making SPI Active
			p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_calculate_data_IHOLD_IRUN;
		case e_api_motor_driver_tmc5160_SPI_state_calculate_data_IHOLD_IRUN:
		{
			uint32_t data = api_motor_driver_tmc5160_ihold_irun_value(p_self->I_HOLD, p_self->I_RUN, p_self->I_HOLDDELAY);
			api_motor_driver_tmc5160_build_SPI_telegram(e_api_motor_driver_tmc5160_register_address_IHOLD_IRUN, data, p_self->data_byte);
			p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_write_IHOLD_IRUN;
		}
		case e_api_motor_driver_tmc5160_SPI_state_write_IHOLD_IRUN:
			result_internal = HAL_SPI_Transmit_IT(p_self->hspi, p_self->data_byte, BYTE_SIZE);
			if (HAL_OK == result_internal)
			{
				p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_calculate_data_GSTAT;
			}
			else if (HAL_ERROR == result_internal)
			{
				p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_failure_release_slave;
			}
			else
			{
				//Nothing to do, it will be in the same state as the message is not yet sent
			}
			break;
		case e_api_motor_driver_tmc5160_SPI_state_calculate_data_GSTAT:
		{
			uint32_t data = DATA_GSTAT_REGISTER_CLEAR_FLAGS;
			api_motor_driver_tmc5160_build_SPI_telegram(e_api_motor_driver_tmc5160_register_address_GSTAT, data, p_self->data_byte);
			p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_write_GSTAT;
		}
		case e_api_motor_driver_tmc5160_SPI_state_write_GSTAT:
			result_internal = HAL_SPI_Transmit_IT(p_self->hspi, p_self->data_byte, BYTE_SIZE);
			if (HAL_OK == result_internal)
			{
				p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_normal_release_slave;
			}
			else if (HAL_ERROR == result_internal)
			{
				p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_failure_release_slave;
			}
			else
			{
				//Nothing to do, it will be in the same state as the message is not yet sent
			break;
		case e_api_motor_driver_tmc5160_SPI_state_normal_release_slave:
			result_state_machine = HAL_OK;
			p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_idle;
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);   // Pull CS HIGH for making SPI Inactive
			break;
		case e_api_motor_driver_tmc5160_SPI_state_failure_release_slave:
			result_state_machine = HAL_ERROR;
			p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_idle;
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);   // Pull CS HIGH for making SPI Inactive
			break;
		default:
			result_state_machine = HAL_ERROR;
			p_self->state_initialize = e_api_motor_driver_tmc5160_SPI_state_idle;
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);   // Pull CS HIGH for making SPI Inactive
			break;
	}
	return result_state_machine;
}


HAL_StatusTypeDef api_motor_driver_tmc5160_set_velocity(const struct s_api_motor_driver_t * p_motor_driver, int16_t rpm, e_api_motor_driver_motor_direction_t direction)
{
    if (p_motor_driver == NULL)
    {
        return HAL_ERROR;
    }
	s_api_motor_driver_tmc5160_t * p_self = (s_api_motor_driver_tmc5160_t*)p_motor_driver;
	HAL_StatusTypeDef result_state_machine = HAL_BUSY;
	HAL_StatusTypeDef result_internal = HAL_BUSY;
	switch(p_self -> state_set_velocity)
	{
		case e_api_motor_driver_tmc5160_SPI_state_idle:
			p_self->state_set_velocity = e_api_motor_driver_tmc5160_SPI_state_acquiring_slave;
		case e_api_motor_driver_tmc5160_SPI_state_acquiring_slave:
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET); // Pull CS LOW for Making SPI Active
			p_self->state_set_velocity = e_api_motor_driver_tmc5160_SPI_state_calculate_VACTUAL;
		case e_api_motor_driver_tmc5160_SPI_state_calculate_VACTUAL:
		{
			int32_t vactual = (MICROSTEPS_PER_REVOLUTION * rpm) / SECONDS_PER_MINUTE;  // Velocity calculation
			if (direction == e_api_motor_driver_motor_direction_counter_clockwise)
			{
			    vactual = -vactual;
			}
			api_motor_driver_tmc5160_build_SPI_telegram(e_api_motor_driver_tmc5160_register_address_VACTUAL,(uint32_t)vactual, p_self->data_byte);
			p_self->state_set_velocity = e_api_motor_driver_tmc5160_SPI_state_write_VACTUAL;
		}
		case e_api_motor_driver_tmc5160_SPI_state_write_VACTUAL:
			result_internal = HAL_SPI_Transmit_IT(p_self->hspi, p_self->data_byte, BYTE_SIZE);
			if (HAL_OK == result_internal)
			{
				p_self->state_set_velocity = e_api_motor_driver_tmc5160_SPI_state_normal_release_slave;
			}
			else if (HAL_ERROR == result_internal)
			{
				p_self->state_set_velocity = e_api_motor_driver_tmc5160_SPI_state_failure_release_slave;
			}
			else
			{
				//Nothing to do, it will be in the same state as the message is not yet sent
			}
			break;
		case e_api_motor_driver_tmc5160_SPI_state_normal_release_slave:
			result_state_machine = HAL_OK;
			p_self->state_set_velocity = e_api_motor_driver_tmc5160_SPI_state_idle;
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);   // Pull CS HIGH for making SPI Inactive
			break;
		case e_api_motor_driver_tmc5160_SPI_state_failure_release_slave:
			result_state_machine = HAL_ERROR;
			p_self->state_set_velocity = e_api_motor_driver_tmc5160_SPI_state_idle;
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);   // Pull CS HIGH for making SPI Inactive
			break;
		default:
			result_state_machine = HAL_ERROR;
			p_self->state_set_velocity = e_api_motor_driver_tmc5160_SPI_state_idle;
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);   // Pull CS HIGH for making SPI Inactive
			break;
	}
	return result_state_machine;
}
}
