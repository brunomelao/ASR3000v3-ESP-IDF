#ifndef AQUISICAO_H
#define AQUISICAO_H
#include "defs.h"

#define G 9.80665
void bmp_acquire(dados *data, bmp280_t *bmp280);
void gps_init();
void mpu6050_acquire(dados *data, mpu6050_dev_t *dev_mpu);
void voltage_acquire(dados *data, adc_oneshot_unit_handle_t *adc_handle, adc_cali_handle_t *cali_handle);
#endif // AQUISICAO_H
