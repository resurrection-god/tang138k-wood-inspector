/*
 * ******************************************************************************************
 * File     : app_log_config.h
 * Author   : Antigravity / User
 * Platform : Gowin GW5AST / Andes AE350 RISC-V SoC
 * Function : Subsystem Logging Configuration & Detailed I2C Diagnostic Macros
 * ******************************************************************************************
 */

#ifndef APP_LOG_CONFIG_H_
#define APP_LOG_CONFIG_H_

#include <stdio.h>

/*
 * Subsystem Log Switches (1 = Enabled, 0 = Disabled)
 * Set LOG_ENABLE_I2C to 1 for detailed camera bus diagnostics.
 */
#define LOG_ENABLE_I2C          0   // Detailed I2C camera bus transactions & probing logs

// Detailed I2C Logging Macros
#if LOG_ENABLE_I2C
    #define LOG_I2C(fmt, ...)       printf("[I2C] " fmt "\r\n", ##__VA_ARGS__)
    #define LOG_I2C_ERR(fmt, ...)   printf("[I2C ERR] " fmt "\r\n", ##__VA_ARGS__)
#else
    #define LOG_I2C(fmt, ...)       do {} while(0)
    #define LOG_I2C_ERR(fmt, ...)   do {} while(0)
#endif

#endif /* APP_LOG_CONFIG_H_ */
