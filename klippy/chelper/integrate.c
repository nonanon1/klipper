// Helpers to integrate the smoothing weight function
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "compiler.h"
#include "integrate.h"

#include <stdlib.h>
#include <string.h>

/****************************************************************
 * Generic smoother integration
 ****************************************************************/

// Integrate t^0 * w, with 4th order w
static inline double
i2wt0(const struct smoother *sm, double t)
{
    double t2 = t*t;
    double v = (1./3.) * sm->c2;
    v = sm->c0 + v * t2;
    return v * t;
}

// Integrate t^1 * w, with 4th order w
static inline double
i2wt1(const struct smoother *sm, double t)
{
    double t2 = t*t;
    double v = (1./4.) * sm->c2;
    v = (1./2.) * sm->c0 + v * t2;
    return v * t2;
}

// Integrate t^2 * w, with 4th order w
static inline double
i2wt2(const struct smoother *sm, double t)
{
    double t2 = t*t;
    double v = (1./5.) * sm->c2;
    v = (1./3.) * sm->c0 + v * t2;
    return v * t2 * t;
}

static double
integrate_2th_order(const struct smoother *sm, double start, double end
                    , double a0, double a1, double a2)
{
    double res = a2 * (i2wt2(sm, end) - i2wt2(sm, start));
    res += a1 * (i2wt1(sm, end) - i2wt1(sm, start));
    res += a0 * (i2wt0(sm, end) - i2wt0(sm, start));
    return res;
}

// Integrate t^0 * w, with 4th order w
static inline double
i4wt0(const struct smoother *sm, double t)
{
    double t2 = t*t;
    double v = (1./5.) * sm->c4;
    v = (1./3.) * sm->c2 + v * t2;
    v = sm->c0 + v * t2;
    return v * t;
}

// Integrate t^1 * w, with 4th order w
static inline double
i4wt1(const struct smoother *sm, double t)
{
    double t2 = t*t;
    double v = (1./6.) * sm->c4;
    v = (1./4.) * sm->c2 + v * t2;
    v = (1./2.) * sm->c0 + v * t2;
    return v * t2;
}

// Integrate t^2 * w, with 4th order w
static inline double
i4wt2(const struct smoother *sm, double t)
{
    double t2 = t*t;
    double v = (1./7.) * sm->c4;
    v = (1./5.) * sm->c2 + v * t2;
    v = (1./3.) * sm->c0 + v * t2;
    return v * t2 * t;
}

static double
integrate_4th_order(const struct smoother *sm, double start, double end
                    , double a0, double a1, double a2)
{
    double res = a2 * (i4wt2(sm, end) - i4wt2(sm, start));
    res += a1 * (i4wt1(sm, end) - i4wt1(sm, start));
    res += a0 * (i4wt0(sm, end) - i4wt0(sm, start));
    return res;
}

// Integrate t^0 * w, with 6th order w
static inline double
i6wt0(const struct smoother *sm, double t)
{
    double t2 = t*t;
    double v = (1./7.) * sm->c6;
    v = (1./5.) * sm->c4 + v * t2;
    v = (1./3.) * sm->c2 + v * t2;
    v = sm->c0 + v * t2;
    return v * t;
}

// Integrate t^1 * w, with 6th order w
static inline double
i6wt1(const struct smoother *sm, double t)
{
    double t2 = t*t;
    double v = (1./8.) * sm->c6;
    v = (1./6.) * sm->c4 + v * t2;
    v = (1./4.) * sm->c2 + v * t2;
    v = (1./2.) * sm->c0 + v * t2;
    return v * t2;
}

// Integrate t^2 * w, with 6th order w
static inline double
i6wt2(const struct smoother *sm, double t)
{
    double t2 = t*t;
    double v = (1./9.) * sm->c6;
    v = (1./7.) * sm->c4 + v * t2;
    v = (1./5.) * sm->c2 + v * t2;
    v = (1./3.) * sm->c0 + v * t2;
    return v * t2 * t;
}

static double
integrate_6th_order(const struct smoother *sm, double start, double end
                    , double a0, double a1, double a2)
{
    double res = a2 * (i6wt2(sm, end) - i6wt2(sm, start));
    res += a1 * (i6wt1(sm, end) - i6wt1(sm, start));
    res += a0 * (i6wt0(sm, end) - i6wt0(sm, start));
    return res;
}

// Integrate (pos + start_v*t + half_accel*t^2) with smoothing weight function
// over the range [start; end] with T == -toff
double
integrate_weighted(const struct smoother *sm, double pos
                   , double start_v, double half_accel
                   , double start, double end, double toff)
{
    // Substitute the integration variable tnew = t + toff to simplify integrals
    pos += (half_accel * toff - start_v) * toff;
    start_v -= 2. * half_accel * toff;
    start += toff; end += toff;
    return sm->integrate_cb(sm, start, end, pos, start_v, half_accel);
}

/****************************************************************
 * Smoother-specific initialization
 ****************************************************************/

static void
init_2ord_shortest(struct smoother *sm, double target_freq, double damping_ratio)
{
    // Shortest smoother reducing vibrations to 0 at target frequency which
    // does not excite higher-frequency vibrations
    sm->integrate_cb = &integrate_2th_order;
    double hst = .29630246 / target_freq;
    sm->hst = hst;
    double v = 1. / hst;
    double inv_hst2 = v * v;
    sm->c0 = 0.2183076974181258 * v;
    v *= inv_hst2;
    sm->c2 = 2.154923092254376 * v;
}

static void
init_2ord_allp(struct smoother *sm, double target_freq, double damping_ratio)
{
    // Smoother reducing vibrations to 0 at target frequency
    sm->integrate_cb = &integrate_2th_order;
    double hst = .331293106 / target_freq;
    sm->hst = hst;
    double v = 1. / hst;
    double inv_hst2 = v * v;
    sm->c0 = 0.;
    v *= inv_hst2;
    sm->c2 = 1.5 * v;
}

static void
init_sifp_05(struct smoother *sm, double target_freq, double damping_ratio)
{
    // SI-type with 5% vibration tolerance with full period duration
    sm->integrate_cb = &integrate_4th_order;
    double hst = .5 / target_freq;
    sm->hst = hst;
    double v = 1. / hst;
    double inv_hst2 = v * v;
    sm->c0 = 1.226407107944368 * v;
    v *= inv_hst2;
    sm->c2 = -9.681726703406114 * v;
    v *= inv_hst2;
    sm->c4 = 12.50417563262201 * v;
}

static void
init_siaf_05(struct smoother *sm, double target_freq, double damping_ratio)
{
    // A 4th-order positive smoother suppressing all vibrations above
    // the target frequency by at least 95% (5% vibrations tolerance)
    sm->integrate_cb = &integrate_4th_order;
    double hst = 0.682156695 / target_freq;
    sm->hst = hst;
    double v = 1. / hst;
    double inv_hst2 = v * v;
    sm->c0 = 0.7264076297522936 * v;
    v *= inv_hst2;
    sm->c2 = -1.00906293169719 * v;
    v *= inv_hst2;
    sm->c4 = 0.5497334040671973 * v;
}

static void
init_dfsf_05(struct smoother *sm, double target_freq, double damping_ratio)
{
    // Acceleration displacement-free smoother suppressing all vibrations above
    // near the target frequency by at least 95% (5% vibrations tolerance)
    sm->integrate_cb = &integrate_6th_order;
    double hst = 0.879442505 / target_freq;
    sm->hst = hst;
    double v = 1. / hst;
    double inv_hst2 = v * v;
    sm->c0 = 1.693005551405153 * v;
    v *= inv_hst2;
    sm->c2 = -18.8720117988809 * v;
    v *= inv_hst2;
    sm->c4 = 59.4391940955727 * v;
    v *= inv_hst2;
    sm->c6 = -47.53121639625473 * v;
}

static void
init_dfaf_05(struct smoother *sm, double target_freq, double damping_ratio)
{
    // Acceleration displacement-free smoother suppressing all vibrations above
    // the target frequency by at least 95% (5% vibrations tolerance)
    sm->integrate_cb = &integrate_6th_order;
    double hst = 1.089438525 / target_freq;
    sm->hst = hst;
    double v = 1. / hst;
    double inv_hst2 = v * v;
    sm->c0 = 1.42427487336909 * v;
    v *= inv_hst2;
    sm->c2 = -5.783771970272312 * v;
    v *= inv_hst2;
    sm->c4 = 7.766315293352271 * v;
    v *= inv_hst2;
    sm->c6 = -3.847297593641651 * v;
}

static void
init_dfaf_02(struct smoother *sm, double target_freq, double damping_ratio)
{
    // Acceleration displacement-free smoother suppressing all vibrations above
    // the target frequency by at least 98% (2% vibrations tolerance)
    sm->integrate_cb = &integrate_6th_order;
    double hst = 1.282011392 / target_freq;
    sm->hst = hst;
    double v = 1. / hst;
    double inv_hst2 = v * v;
    sm->c0 = 1.57525352661564 * v;
    v *= inv_hst2;
    sm->c2 = -7.728603566914598 * v;
    v *= inv_hst2;
    sm->c4 = 11.55794321405673 * v;
    v *= inv_hst2;
    sm->c6 = -5.674486863182988 * v;
}

static void
init_dfaf_01(struct smoother *sm, double target_freq, double damping_ratio)
{
    // Acceleration displacement-free smoother suppressing all vibrations above
    // the target frequency by at least 99% (1% vibrations tolerance)
    sm->integrate_cb = &integrate_6th_order;
    double hst = 1.727828982 / target_freq;
    sm->hst = hst;
    double v = 1. / hst;
    double inv_hst2 = v * v;
    sm->c0 = 1.561217589994576 * v;
    v *= inv_hst2;
    sm->c2 = -7.310414825115637 * v;
    v *= inv_hst2;
    sm->c4 = 10.09765353406272 * v;
    v *= inv_hst2;
    sm->c6 = -4.507603485713351 * v;
}

typedef void (*init_smoother_callback)(struct smoother *sm, double target_freq
                                       , double damping_ratio);

static init_smoother_callback init_smoother_callbacks[] = {
    [SIFP05] = &init_sifp_05,
    [SIAF05] = &init_siaf_05,
    [DFSF05] = &init_dfsf_05,
    [DFAF05] = &init_dfaf_05,
    [DFAF02] = &init_dfaf_02,
    [DFAF01] = &init_dfaf_01,
};

struct smoother *
alloc_smoother(int smoother_type, double target_freq, double damping_ratio)
{
    if (smoother_type >= ARRAY_SIZE(init_smoother_callbacks)
            || smoother_type < 0)
        return NULL;
    struct smoother *sm = malloc(sizeof(*sm));
    memset(sm, 0, sizeof(*sm));
    init_smoother_callbacks[smoother_type](sm, target_freq, damping_ratio);
    return sm;
}
