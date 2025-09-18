
#ifndef DRIVER_PMSX003_READ_TEST_H
#define DRIVER_PMSX003_READ_TEST_H

#include "driver_pmsx003_interface.h"

#ifdef __cplusplus
extern "C"{
#endif

/**
 * @defgroup pmsx003_test_driver pmsx003 test driver function
 * @brief    pmsx003 test driver modules
 * @ingroup  pmsx003_driver
 * @{
 */

/**
 * @brief     read test
 * @param[in] times test times
 * @return    status code
 *            - 0 success
 *            - 1 test failed
 * @note      none
 */
uint8_t pmsx003_read_test(uint32_t times);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif
