/*
 * api_motor_driver.c
 *
 *  Created on: Jun 28, 2025
 *      Author: vijji
 */

#include "api_motor_driver.h"
#include "stm32f4xx_hal.h"
// Functions

HAL_StatusTypeDef api_motor_driver_initialize(const struct s_api_motor_driver_t *p_motor_driver)
{
	if (p_motor_driver && p_motor_driver->initialize)
	    {
	        return p_motor_driver->initialize(p_motor_driver);
	    }
	    return HAL_ERROR;
}

HAL_StatusTypeDef api_motor_driver_set_velocity(const struct s_api_motor_driver_t * p_motor_driver, int16_t rpm, e_api_motor_driver_motor_direction_t direction)
{
    if (p_motor_driver && p_motor_driver->set_velocity)
    {
        return p_motor_driver->set_velocity(p_motor_driver, rpm, direction);
    }
    return HAL_ERROR;
}
