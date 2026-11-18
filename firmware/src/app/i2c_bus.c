/*
 * i2c_bus.c - the I2C bus and the TWI multiplexer. Part of camera.c until
 * 24.09.2026 (i2c_bus_reset, i2c_mux_select, driver init in camera_init).
 */

#include "i2c_bus.h"

#include "FreeRTOS.h"
#include "semphr.h"

// Bus lock (i2c_bus.h). Until 24.09.2026 it was the global non-recursive
// xI2CMutex from main.c, which callers took by hand.
static SemaphoreHandle_t s_lock;
static uint8_t           s_mux = 0xFFU;
#include "board.h"
#include "platform.h"
#include "app_log_config.h"
#include <stdio.h>

#include "Driver_GPIO.h"
extern AE350_DRIVER_GPIO Driver_GPIO;
static AE350_DRIVER_GPIO* pDrv_GPIO = &Driver_GPIO;

#include "Driver_I2C.h"
extern AE350_DRIVER_I2C Driver_I2C;
static AE350_DRIVER_I2C* pDrv_I2C = &Driver_I2C;

void i2c_bus_init(void)
{
    s_lock = xSemaphoreCreateRecursiveMutex();
    configASSERT(s_lock != NULL);
    pDrv_I2C->Initialize(NULL);
    pDrv_I2C->PowerControl(AE350_POWER_FULL);
    pDrv_I2C->Control(AE350_I2C_BUS_SPEED, AE350_I2C_BUS_SPEED_STANDARD);
}

// Reset and recover I2C bus in case of BUSY lockup state
void i2c_bus_reset(void)
{
    LOG_I2C_ERR("Resetting I2C Controller...");
    pDrv_I2C->Control(AE350_I2C_ABORT_TRANSFER, 0);
    pDrv_I2C->PowerControl(AE350_POWER_OFF);
    pDrv_I2C->PowerControl(AE350_POWER_FULL);
    pDrv_I2C->Control(AE350_I2C_BUS_SPEED, AE350_I2C_BUS_SPEED_STANDARD);
}

// Drive the three multiplexer select lines.
//
// Bit 0 -> GPIO[6] -> FPGA port TWI_MUX_S0 -> pin P26
// Bit 1 -> GPIO[7] -> FPGA port TWI_MUX_S1 -> pin P19
// Bit 2 -> GPIO[8] -> FPGA port TWI_MUX_S2 -> pin N19
//
// WARNING: the port names in scanner.cst do NOT match the board
// schematic. The schematic calls P26 "TWI_MUX.S2" and N19 "TWI_MUX.S0", i.e. the
// outer two are swapped relative to this project's names. Treat the mux value
// below purely as {N19, P19, P26} and ignore the S0/S1/S2 labels.
//
// Measured on hardware: camera connector 0 needs P26=1, P19=1, N19=0, which is
// mux value 3 in this encoding. Camera connector 1 is mux value 7 {1,1,1}.
void i2c_mux_select(uint8_t mux)
{
    s_mux = mux;
    pDrv_GPIO->PinWrite(I2C_MUX_S0_PIN, (mux >> 0) & 1);  // P26
    pDrv_GPIO->PinWrite(I2C_MUX_S1_PIN, (mux >> 1) & 1);  // P19
    pDrv_GPIO->PinWrite(I2C_MUX_S2_PIN, (mux >> 2) & 1);  // N19
    board_delay_us(5000); // settling time after switching the multiplexer
}

bool i2c_bus_lock(uint32_t timeout_ms)
{
    return s_lock != NULL &&
           xSemaphoreTakeRecursive(s_lock, pdMS_TO_TICKS(timeout_ms)) == pdTRUE;
}

void i2c_bus_unlock(void)
{
    xSemaphoreGiveRecursive(s_lock);
}
