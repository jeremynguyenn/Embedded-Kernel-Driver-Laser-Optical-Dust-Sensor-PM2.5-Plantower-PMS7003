

#ifndef DRIVER_PMSX003_BASIC_H
#define DRIVER_PMSX003_BASIC_H

#include "driver_pmsx003_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup pmsx003_example_driver pmsx003 example driver function
 * @brief    pmsx003 example driver modules
 * @ingroup  pmsx003_driver
 * @{
 */

/**
 * @brief pmsx003 basic example default definition
 */
#define PMSX003_BASIC_DEFAULT_MODE        PMSX003_MODE_PASSIVE        /**< passive mode */

/**
 * @brief  basic example init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t pmsx003_basic_init(void);

/**
 * @brief  basic example deinit
 * @return status code
 *         - 0 success
 *         - 1 deinit failed
 * @note   none
 */
uint8_t pmsx003_basic_deinit(void);

/**
 * @brief      basic example read
 * @param[out] *data pointer to a data structure
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t pmsx003_basic_read(pmsx003_data_t *data);

/**
 * @brief  basic sleep
 * @return status code
 *         - 0 success
 *         - 1 sleep failed
 * @note   none
 */
uint8_t pmsx003_basic_sleep(void);

/**
 * @brief  basic wake up
 * @return status code
 *         - 0 success
 *         - 1 wake up failed
 * @note   none
 */
uint8_t pmsx003_basic_wake_up(void);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
