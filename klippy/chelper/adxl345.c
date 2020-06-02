// ADXL345 sensor support
//
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <math.h>
#include <pigpiod_if2.h>
#include <stdlib.h> // malloc
#include <string.h> // memset
#include <time.h>
#include "accel_values.h" // struct accel_values
#include "compiler.h" // __visible
#include "pyhelper.h" // errorf

#define SPI_CHANNEL 0
#define SPI_FREQUENCY 2000000
#define SPI_FLAGS 3 // POL & PHA

// ADXL345 register addresses
#define BW_RATE 0x2C
#define POWER_CTL 0x2D
#define DATA_FORMAT 0x31

// Multi-byte transfer
#define MULTI_BYTE 0x40

// Read bit
#define READ_BIT 0x80

// ADXL345 register values
#define DATA_RATE_3200 0xF
#define DATA_RANGE_16G_FULL 0xB
#define MEASURE_MODE 0x8
#define STANDBY_MODE 0x0
#define DATAX0 0x32

#define SCALE_MULTIPLIER (0.004 * 9806.65) // 4mg/LSB * Earth gravity
#define READ_RATE 3200

struct adxl345 {
    int pi, handle;
};

int
adxl345_read(struct adxl345 *acc, double *ax, double *ay, double *az)
{
    char buf[7];
    buf[0] = DATAX0 | READ_BIT | MULTI_BYTE;
    int res = spi_xfer(acc->pi, acc->handle, buf, buf, 7);
    if (res != 7)
        return res;
    *ax = SCALE_MULTIPLIER * (int16_t)(buf[1] | (buf[2] << 8));
    *ay = SCALE_MULTIPLIER * (int16_t)(buf[3] | (buf[4] << 8));
    *az = SCALE_MULTIPLIER * (int16_t)(buf[5] | (buf[6] << 8));
    return 0;
}

struct accel_values * __visible
adxl345_measure(struct adxl345 *acc, double duration)
{
    char buf[2];
    int n = ceil(duration * READ_RATE), res;
    if (n <= 0)
        return NULL;
    struct accel_values *values = accel_values_alloc(n);
    double delay = 1. / READ_RATE;
    buf[0] = POWER_CTL | MULTI_BYTE;
    buf[1] = MEASURE_MODE;
    if ((res = spi_write(acc->pi, acc->handle, buf, 2)) != 2)
        goto transfer_error;
    double ax, ay, az;
    // Cold read
    if ((res = adxl345_read(acc, &ax, &ay, &az)) != 0)
        goto transfer_error;
    double t0 = time_time();
    for (int i = 0; i < n; ++i) {
        double t = time_time();
        if ((res = adxl345_read(acc, &ax, &ay, &az)) != 0)
            goto transfer_error;
        values->ax[i] = ax;
        values->ay[i] = ay;
        values->az[i] = az;
        values->t[i] = t - t0;

        double sleep = (i+1) * delay - (time_time()-t0);
        if (sleep > 0)
            time_sleep(sleep);
    }

    buf[0] = POWER_CTL | MULTI_BYTE;
    buf[1] = STANDBY_MODE;
    if ((res = spi_write(acc->pi, acc->handle, buf, 2)) != 2)
        goto transfer_error;
    return values;

transfer_error:
    errorf("SPI transmissions failure: %d", res);
    buf[0] = POWER_CTL | MULTI_BYTE;
    buf[1] = STANDBY_MODE;
    spi_write(acc->pi, acc->handle, buf, 2);
    accel_values_free(values);
    return NULL;
}

struct adxl345 * __visible
adxl345_init()
{
    int pi = pigpio_start(NULL, NULL);
    if (pi < 0) {
        errorf("PIGPIO initalization failed: %d", pi);
        return NULL;
    }
    int handle = spi_open(pi, SPI_CHANNEL, SPI_FREQUENCY, SPI_FLAGS);
    if (handle < 0) {
        pigpio_stop(pi);
        errorf("SPI initalization failed: %d", handle);
        return NULL;
    }
    char buf[2];
    buf[0] = BW_RATE | MULTI_BYTE;
    buf[1] = DATA_RATE_3200;
    int res;
    if ((res = spi_write(pi, handle, buf, 2)) != 2)
        goto init_error;

    buf[0] = DATA_FORMAT | MULTI_BYTE;
    buf[1] = DATA_RANGE_16G_FULL;
    if ((res = spi_write(pi, handle, buf, 2)) != 2)
        goto init_error;


    struct adxl345 *acc = malloc(sizeof(*acc));
    memset(acc, 0, sizeof(*acc));
    acc->pi = pi;
    acc->handle = handle;

    // Read a few cold values
    accel_values_free(adxl345_measure(acc, 0.1));
    return acc;

init_error:
    errorf("SPI transmissions failure: %d", res);
    spi_close(pi, handle);
    pigpio_stop(pi);
    return NULL;
}

void __visible
adxl345_free(struct adxl345 *acc)
{
    if (acc == NULL)
        return;
    spi_close(acc->pi, acc->handle);
    pigpio_stop(acc->pi);
    free(acc);
}
