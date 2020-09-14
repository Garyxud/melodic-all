#include <stdint.h>

#include <breezystm32.h>

#include "drv_bmp280.h"
#include "drv_i2c.h"

#define BMP280_DEFAULT_ADDR 0x76

#define BMP280_DEFAULT_CHIP_ID               0x58

#define BMP280_CHIP_ID_REG                   0xD0  /* Chip ID Register */
#define BMP280_RST_REG                       0xE0  /* Softreset Register */
#define BMP280_STAT_REG                      0xF3  /* Status Register */
#define BMP280_CTRL_MEAS_REG                 0xF4  /* Ctrl Measure Register */
#define BMP280_CONFIG_REG                    0xF5  /* Configuration Register */
#define BMP280_PRESSURE_MSB_REG              0xF7  /* Pressure MSB Register */
#define BMP280_PRESSURE_LSB_REG              0xF8  /* Pressure LSB Register */
#define BMP280_PRESSURE_XLSB_REG             0xF9  /* Pressure XLSB Register */
#define BMP280_TEMPERATURE_MSB_REG           0xFA  /* Temperature MSB Reg */
#define BMP280_TEMPERATURE_LSB_REG           0xFB  /* Temperature LSB Reg */
#define BMP280_TEMPERATURE_XLSB_REG          0xFC  /* Temperature XLSB Reg */
#define BMP280_FORCED_MODE                   0x01
#define BMP280_NORMAL_MODE                   0x03

#define BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG             0x88
#define BMP280_PRESSURE_TEMPERATURE_CALIB_DATA_LENGTH       24
#define BMP280_DATA_FRAME_SIZE               6

#define BMP280_OVERSAMP_SKIPPED          0x00
#define BMP280_OVERSAMP_1X               0x01
#define BMP280_OVERSAMP_2X               0x02
#define BMP280_OVERSAMP_4X               0x03
#define BMP280_OVERSAMP_8X               0x04
#define BMP280_OVERSAMP_16X              0x05


typedef struct bmp280_calib_param_s {
  uint16_t dig_T1; /* calibration T1 data */
  int16_t dig_T2; /* calibration T2 data */
  int16_t dig_T3; /* calibration T3 data */
  uint16_t dig_P1; /* calibration P1 data */
  int16_t dig_P2; /* calibration P2 data */
  int16_t dig_P3; /* calibration P3 data */
  int16_t dig_P4; /* calibration P4 data */
  int16_t dig_P5; /* calibration P5 data */
  int16_t dig_P6; /* calibration P6 data */
  int16_t dig_P7; /* calibration P7 data */
  int16_t dig_P8; /* calibration P8 data */
  int16_t dig_P9; /* calibration P9 data */
  int32_t t_fine; /* calibration t_fine data */
} bmp280_calib_param_t;

static bmp280_calib_param_t bmp280_cal;

static bool sensor_present = false;
static uint32_t pressure_raw;
static uint32_t temperature_raw;
static float pressure;
static float temperature;
static uint8_t buffer[BMP280_DATA_FRAME_SIZE];
static bool new_data = false;

bool bmp280_present()
{
  return sensor_present;
}

bool bmp280_init()
{
  // wait for chip to power up
  while(millis() < 20);

  uint8_t buf;
    if (!i2cRead(BMP280_DEFAULT_ADDR, BMP280_CHIP_ID_REG, 1, &buf))
    return false;
  else
    sensor_present = true;

  // read calibration
  i2cRead(BMP280_DEFAULT_ADDR, BMP280_TEMPERATURE_CALIB_DIG_T1_LSB_REG, 24, (uint8_t *)&bmp280_cal);

  // set oversampling + power mode (forced), and start sampling
  i2cWrite(BMP280_DEFAULT_ADDR, BMP280_CTRL_MEAS_REG, (BMP280_OVERSAMP_8X << 2 | BMP280_OVERSAMP_1X << 5 | BMP280_NORMAL_MODE));

  return true;
}

// Returns temperature in DegC, resolution is 0.01 DegC. Output value of "5123" equals 51.23 DegC
// t_fine carries fine temperature as global value
static int32_t bmp280_compensate_T(int32_t adc_T)
{
    int32_t var1, var2, T;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_cal.dig_T1 << 1))) * ((int32_t)bmp280_cal.dig_T2)) >> 11;
    var2  = (((((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp280_cal.dig_T1))) >> 12) * ((int32_t)bmp280_cal.dig_T3)) >> 14;
    bmp280_cal.t_fine = var1 + var2;
    T = (bmp280_cal.t_fine * 5 + 128) >> 8;

    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of "24674867" represents 24674867/256 = 96386.2 Pa = 963.862 hPa
static uint32_t bmp280_compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)bmp280_cal.t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)bmp280_cal.dig_P6;
    var2 = var2 + ((var1*(int64_t)bmp280_cal.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_cal.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp280_cal.dig_P3) >> 8) + ((var1 * (int64_t)bmp280_cal.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp280_cal.dig_P1) >> 33;
    if (var1 == 0)
        return 0;
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp280_cal.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp280_cal.dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_cal.dig_P7) << 4);
    return (uint32_t)p;
}

void bmp280_calculate(void)
{
  temperature = (float)(bmp280_compensate_T(temperature_raw))/100.0;
  pressure = (float)(bmp280_compensate_P(pressure_raw))/256.0;
  new_data = false;
}

void bmp280_update(void)
{
  // read data from sensor
  i2cRead(BMP280_DEFAULT_ADDR, BMP280_PRESSURE_MSB_REG, BMP280_DATA_FRAME_SIZE, buffer);
  pressure_raw = (int32_t)((((uint32_t)(buffer[0])) << 12) | (((uint32_t)(buffer[1])) << 4) | ((uint32_t)buffer[2] >> 4));
  temperature_raw = (int32_t)((((uint32_t)(buffer[3])) << 12) | (((uint32_t)(buffer[4])) << 4) | ((uint32_t)buffer[5] >> 4));
  new_data = true;
}

void bmp280_read(float* pres, float* temp)
{
  if (new_data)
  {
    bmp280_calculate();
  }

  *pres = pressure;
  *temp = temperature;
}

/// ASYNC Methods
static volatile uint8_t async_read_status;
static void bmp280_async_read_cb(uint8_t result)
{
  if (result != I2C_JOB_ERROR)
  {
    new_data = true;
    pressure_raw = (int32_t)((((uint32_t)(buffer[0])) << 12) | (((uint32_t)(buffer[1])) << 4) | ((uint32_t)buffer[2] >> 4));
    temperature_raw = (int32_t)((((uint32_t)(buffer[3])) << 12) | (((uint32_t)(buffer[4])) << 4) | ((uint32_t)buffer[5] >> 4));
  }
}

void bmp280_async_update()
{
  static uint32_t next_update_ms = 0;
  uint32_t now_ms = millis();
  if (now_ms > next_update_ms)
  {
    i2c_queue_job(READ,
                  BMP280_DEFAULT_ADDR,
                  BMP280_PRESSURE_MSB_REG,
                  buffer,
                  6,
                  &async_read_status,
                  &bmp280_async_read_cb);
    next_update_ms += 20; // Read at 50 Hz
  }
}

void bmp280_async_read(float *pres, float *temp)
{
  if (new_data)
  {
    bmp280_calculate();
  }

  *pres = pressure;
  *temp = temperature;
}
