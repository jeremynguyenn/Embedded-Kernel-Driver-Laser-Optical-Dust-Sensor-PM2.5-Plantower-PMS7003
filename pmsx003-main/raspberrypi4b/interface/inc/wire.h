

#ifndef WIRE_H
#define WIRE_H

#include <unistd.h>
#include <stdint.h>
#include <stdio.h>

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @defgroup wire wire function
 * @brief    wire function modules
 * @{
 */

/**
 * @brief  wire bus init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t wire_init(void);

/**
 * @brief  wire bus deint
 * @return status code
 *         - 0 success
 * @note   none
 */
uint8_t wire_deinit(void);

/**
 * @brief      wire bus read data
 * @param[out] *value pointer to a data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint8_t wire_read(uint8_t *value);

/**
 * @brief     wire bus write data
 * @param[in] value write data
 * @return    status code
 *            - 0 success
 *             - 1 write failed
 * @note      none
 */
uint8_t wire_write(uint8_t value);

/**
 * @brief  wire bus init
 * @return status code
 *         - 0 success
 *         - 1 init failed
 * @note   none
 */
uint8_t wire_clock_init(void);

/**
 * @brief  wire bus deint
 * @return status code
 *         - 0 success
 * @note   none
 */
uint8_t wire_clock_deinit(void);

/**
 * @brief     wire bus write data
 * @param[in] value write data
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t wire_clock_write(uint8_t value);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
