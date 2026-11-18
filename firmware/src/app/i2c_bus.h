/*
 * i2c_bus.h - the CPU's I2C bus and the TWI multiplexer on it (74HC4051):
 * VEYE cameras. Who holds the bus: i2c_bus_lock().
 */

#ifndef APP_I2C_BUS_H_
#define APP_I2C_BUS_H_

#include <stdint.h>
#include <stdbool.h>

// The bus and multiplexer lock is recursive: camera functions take
// it themselves, and whoever needs an unbroken sequence (select a camera, stop
// capture, write, read back) takes it around the sequence; a repeated take by
// the same task does not block. Created by i2c_bus_init().
// false: not obtained within timeout_ms (or the bus is not created yet).
bool    i2c_bus_lock(uint32_t timeout_ms);
void    i2c_bus_unlock(void);

// TWI multiplexer select lines on the GPIO bus (all three are CPU-driven, so the
// multiplexer can be scanned instead of assuming a fixed address).
#define I2C_MUX_S0_PIN        6   // GPIO[6] -> TWI_MUX_S0 (pin P26)
#define I2C_MUX_S1_PIN        7   // GPIO[7] -> TWI_MUX_S1 (pin P19)
#define I2C_MUX_S2_PIN        8   // GPIO[8] -> TWI_MUX_S2 (pin N19)

// I2C driver: power and 100 kHz speed. Called by camera_init().
void i2c_bus_init(void);
// Reset a controller stuck in BUSY.
void i2c_bus_reset(void);
// Set the multiplexer position (bits {N19, P19, P26}, see i2c_bus.c).
void i2c_mux_select(uint8_t mux);
// Last position set (0xFF = not set yet).

#endif /* APP_I2C_BUS_H_ */
