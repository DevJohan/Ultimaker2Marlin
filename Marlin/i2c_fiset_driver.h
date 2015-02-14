#ifndef MARLIN_I2C_FISET_DRIVER_H_
#define MARLIN_I2C_FISET_DRIVER_H_

#include <inttypes.h>

void fiset_init();
void plan_read_fiset();
bool fiset_data_ready();
int16_t get_fiset_data();
uint8_t get_fiset_gain();
uint16_t get_fiset_magnitude();

#endif /* MARLIN_I2C_FISET_DRIVER_H_ */
