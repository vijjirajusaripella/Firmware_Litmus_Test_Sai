/*
 * api_motor_driver_tmc5160.h
 *
 *  Created on: Jun 28, 2025
 *      Author: vijji
 */

#ifndef INC_API_MOTOR_DRIVER_TMC5160_H_
#define INC_API_MOTOR_DRIVER_TMC5160_H_



#include "api_motor_driver.h"
#include "stm32f4xx_hal.h"

// Numeric Constants
#define BYTE_SIZE 5  // Number of Bytes per SPI Message
typedef enum e_api_motor_driver_tmc5160_IHOLD_t
{
    e_api_motor_driver_tmc5160_IHOLD_1_32  = 0x00, // 1/32 of Full current
    e_api_motor_driver_tmc5160_IHOLD_2_32  = 0x01, // 2/32 of Full current
    e_api_motor_driver_tmc5160_IHOLD_32_32 = 0x1F  // Full current
} e_api_motor_driver_tmc5160_IHOLD_t;

typedef enum e_api_motor_driver_tmc5160_IRUN_t
{
    e_api_motor_driver_tmc5160_IRUN_1_32  = 0x00,  // 1/32 of Full current
    e_api_motor_driver_tmc5160_IRUN_2_32  = 0x01,  // 2/32 of Full current
    e_api_motor_driver_tmc5160_IRUN_31_32  = 0x1E, // 31/32 of Full current
    e_api_motor_driver_tmc5160_IRUN_32_32  = 0x1F // Full current
} e_api_motor_driver_tmc5160_IRUN_t;

typedef enum e_api_motor_driver_tmc5160_IHOLDDELAY_t
{
    e_api_motor_driver_tmc5160_IHOLDDELAY_0 = 0x00, // Instant power down
    e_api_motor_driver_tmc5160_IHOLDDELAY_1 = 0x01, // 1 × 2^18 clocks per step
    e_api_motor_driver_tmc5160_IHOLDDELAY_10 = 0x0A, // 10 × 2^18 clocks per step
    e_api_motor_driver_tmc5160_IHOLDDELAY_15 = 0x0F  // Max delay: 15 × 2^18 clocks
} e_api_motor_driver_tmc5160_IHOLDDELAY_t;

// This enum holds the possible states of the Functions for TMC5160

typedef enum e_api_motor_driver_tmc5160_SPI_state_t
{
	e_api_motor_driver_tmc5160_SPI_state_idle = 0,
	e_api_motor_driver_tmc5160_SPI_state_acquiring_slave = 1,
	e_api_motor_driver_tmc5160_SPI_state_calculate_data_IHOLD_IRUN,
	e_api_motor_driver_tmc5160_SPI_state_write_IHOLD_IRUN,
	e_api_motor_driver_tmc5160_SPI_state_calculate_VACTUAL,
	e_api_motor_driver_tmc5160_SPI_state_write_VACTUAL,
	e_api_motor_driver_tmc5160_SPI_state_calculate_data_GSTAT,
	e_api_motor_driver_tmc5160_SPI_state_write_GSTAT,
	e_api_motor_driver_tmc5160_SPI_state_calculate_datagram,
	e_api_motor_driver_tmc5160_SPI_state_normal_release_slave,
	e_api_motor_driver_tmc5160_SPI_state_failure_release_slave,
}e_api_motor_driver_tmc5160_SPI_state_t;
// Struct for TMC5160-specific configuration
typedef struct s_api_motor_driver_tmc5160_t
{
	s_api_motor_driver_t api_motor_driver;
	// SPI handle for communication with the motor driver
    SPI_HandleTypeDef *hspi;
    e_api_motor_driver_tmc5160_SPI_state_t state_initialize;
    e_api_motor_driver_tmc5160_SPI_state_t state_set_velocity;
    uint64_t data_byte[BYTE_SIZE];
    e_api_motor_driver_motor_direction_t direction;
    e_api_motor_driver_tmc5160_IHOLD_t I_HOLD;
    e_api_motor_driver_tmc5160_IRUN_t I_RUN;
    e_api_motor_driver_tmc5160_IHOLDDELAY_t I_HOLDDELAY;
} s_api_motor_driver_tmc5160_t;

// Construct and return a pointer to the driver instance
const s_api_motor_driver_t * s_api_motor_driver_tmc5160_try__construct(s_api_motor_driver_tmc5160_t * p_self,
												                       SPI_HandleTypeDef *hspi,
												                       e_api_motor_driver_tmc5160_IHOLD_t I_HOLD,
												                       e_api_motor_driver_tmc5160_IRUN_t I_RUN,
												                       e_api_motor_driver_tmc5160_IHOLDDELAY_t I_HOLDDELAY);
uint32_t api_motor_driver_tmc5160_ihold_irun_value(uint8_t ihold, uint8_t irun, uint8_t iholddelay);
#endif /* INC_API_MOTOR_DRIVER_TMC5160_H_ */
