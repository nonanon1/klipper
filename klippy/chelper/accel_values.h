#ifndef ACCEL_VALUES_H
#define ACCEL_VALUES_H

struct accel_values {
    int n;
    double *t, *ax, *ay, *az;
};

struct accel_values *accel_values_alloc(int n);
void accel_values_free(struct accel_values *acc);

#endif // accel_values.h
