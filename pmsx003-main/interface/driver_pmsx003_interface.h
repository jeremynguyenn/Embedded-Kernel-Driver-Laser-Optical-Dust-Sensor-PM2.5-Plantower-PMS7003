

#ifndef DRIVER_PMSX003_INTERFACE_H
#define DRIVER_PMSX003_INTERFACE_H

#include "driver_pmsx003.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup pmsx003_interface_driver pmsx003 interface driver function
 * @brief    pmsx003 interface driver modules
 * @ingroup  pmsx003_driver
 * @{
 */

/**
 * @brief  interface uart init
 * @return status code
 *         - 0 success
 *         - 1 uart init failed
 * @note   none
 */
uint8_t pmsx003_interface_uart_init(void);

/**
 * @brief  interface uart deinit
 * @return status code
 *         - 0 success
 *         - 1 uart deinit failed
 * @note   none
 */
uint8_t pmsx003_interface_uart_deinit(void);

/**
 * @brief      interface uart read
 * @param[out] *buf pointer to a data buffer
 * @param[in]  len length of the data buffer
 * @return     status code
 *             - 0 success
 *             - 1 read failed
 * @note       none
 */
uint16_t pmsx003_interface_uart_read(uint8_t *buf, uint16_t len);

/**
 * @brief  interface uart flush
 * @return status code
 *         - 0 success
 *         - 1 uart flush failed
 * @note   none
 */
uint8_t pmsx003_interface_uart_flush(void);

/**
 * @brief     interface uart write
 * @param[in] *buf pointer to a data buffer
 * @param[in] len length of the data buffer
 * @return    status code
 *            - 0 success
 *            - 1 write failed
 * @note      none
 */
uint8_t pmsx003_interface_uart_write(uint8_t *buf, uint16_t len);

/**
 * @brief  interface reset gpio init
 * @return status code
 *         - 0 success
 *         - 1 reset gpio init failed
 * @note   none
 */
uint8_t pmsx003_interface_reset_gpio_init(void);

/**
 * @brief  interface reset gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 reset gpio deinit failed
 * @note   none
 */
uint8_t pmsx003_interface_reset_gpio_deinit(void);

/**
 * @brief     interface reset gpio write
 * @param[in] level gpio level
 * @return    status code
 *            - 0 success
 *            - 1 reset gpio write failed
 * @note      none
 */
uint8_t pmsx003_interface_reset_gpio_write(uint8_t level);

/**
 * @brief  interface set gpio init
 * @return status code
 *         - 0 success
 *         - 1 set gpio init failed
 * @note   none
 */
uint8_t pmsx003_interface_set_gpio_init(void);

/**
 * @brief  interface set gpio deinit
 * @return status code
 *         - 0 success
 *         - 1 set gpio deinit failed
 * @note   none
 */
uint8_t pmsx003_interface_set_gpio_deinit(void);

/**
 * @brief     interface set gpio write
 * @param[in] level gpio level
 * @return    status code
 *            - 0 success
 *            - 1 set gpio write failed
 * @note      none
 */
uint8_t pmsx003_interface_set_gpio_write(uint8_t level);

/**
 * @brief     interface delay ms
 * @param[in] ms time
 * @note      none
 */
void pmsx003_interface_delay_ms(uint32_t ms);

/**
 * @brief     interface print format data
 * @param[in] fmt format data
 * @note      none
 */
void pmsx003_interface_debug_print(const char *const fmt, ...);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
