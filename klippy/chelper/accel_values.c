// Acceleration measurements wrappers
//
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include "accel_values.h" // struct accel_values
#include "compiler.h" // __visible

struct accel_values * __visible
accel_values_alloc(int n)
{
    struct accel_values* acc = malloc(sizeof(*acc));
    memset(acc, 0, sizeof(*acc));
    acc->n = n;
    acc->t = malloc(n * sizeof(*acc->t));
    memset(acc->t, 0, n * sizeof(*acc->t));
    acc->ax = malloc(n * sizeof(*acc->ax));
    memset(acc->ax, 0, n * sizeof(*acc->ax));
    acc->ay = malloc(n * sizeof(*acc->ay));
    memset(acc->ay, 0, n * sizeof(*acc->ay));
    acc->az = malloc(n * sizeof(*acc->az));
    memset(acc->az, 0, n * sizeof(*acc->az));
    return acc;
}

void __visible
accel_values_free(struct accel_values *acc)
{
    if (acc == NULL)
        return;
    free(acc->t);
    free(acc->ax);
    free(acc->ay);
    free(acc->az);
    free(acc);
}
