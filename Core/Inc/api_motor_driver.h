/*
 * api_motor_driver.h
 *
 *  Created on: Jun 28, 2025
 *      Author: vijji
 */

#ifndef INC_API_MOTOR_DRIVER_H_
#define INC_API_MOTOR_DRIVER_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

// Direction for the Motor rotation
typedef enum e_api_motor_driver_motor_direction_t
{
	e_api_motor_driver_motor_direction_clockwise = 0x00,
	e_api_motor_driver_motor_direction_counter_clockwise = 0x01
} e_api_motor_driver_motor_direction_t;

// Structure to store the motor driver data

typedef struct s_api_motor_driver_t
{
	HAL_StatusTypeDef (*initialize)(const struct s_api_motor_driver_t * const p_motor_driver);
	HAL_StatusTypeDef (*set_velocity)(const struct s_api_motor_driver_t * const p_motor_driver, int16_t rpm, e_api_motor_driver_motor_direction_t direction);
} s_api_motor_driver_t;

// function prototypes

/*
 * @brief Initializes the motor driver.
 * @parameter[input] p_motor_driver The pointer to the motor driver.
 * @return The result of the operation
 *  */
HAL_StatusTypeDef api_motor_driver_initialize(const struct s_api_motor_driver_t * p_motor_driver);
/*
 * @brief Initializes the motor driver.
 * @parameter[input] p_motor_driver The pointer to the motor driver.
 * @parameter[input] rpm and direction The parameters required for the motor movement.
 * @return The result of the operation
 *  */
HAL_StatusTypeDef api_motor_driver_set_velocity(const struct s_api_motor_driver_t * p_motor_driver, int16_t rpm, e_api_motor_driver_motor_direction_t direction);

#endif /* INC_API_MOTOR_DRIVER_H_ */
