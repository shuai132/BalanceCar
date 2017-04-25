#ifndef _LSM303AGR_H_
#define _LSM303AGR_H_
#include <stdint.h>
#include "LSM303AGR_ACC_driver.h"
#include "LSM303AGR_MAG_driver.h"

int init_LSM303AGR_acc(LSM303AGR_ACC_FS_t fs, LSM303AGR_ACC_ODR_t odr, u8_t fifo_thld);
int init_LSM303AGR_mag(LSM303AGR_MAG_ODR_t odr);

#endif /*_LSM303AGR_H_*/


