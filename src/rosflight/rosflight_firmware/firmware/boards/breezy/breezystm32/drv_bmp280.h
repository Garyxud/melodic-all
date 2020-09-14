#ifndef DRV_BMP280_H
#define DRV_BMP280_H

#include <stdbool.h>
#include <stdint.h>

bool bmp280_init();

// Blocking I2C update functions
void bmp280_update(void);
void bmp280_read(float* pres, float* temp);

// Asynchronous bmp280 functions
bool bmp280_present(void);
void bmp280_async_update(void);
void bmp280_async_read(float* pres, float* temp);

#endif // DRV_BMP280_H
