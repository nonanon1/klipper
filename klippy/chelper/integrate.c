// Helpers to integrate the smoothing weight function.
//
// Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "integrate.h"
#include "scurve.h"

#include <stdlib.h> // malloc
#include <string.h> // memset

static const double w_antideriv_coeffs[] = {
    1./1.,
    1./2.,
    1./3.,
    1./4.,
    1./5.,
    1./6.,
    1./7.,
    1./8.,
    1./9.,
    1./10.,
};

// Integrate t^n * (t^2-h^2)^2
static inline double
iwtn(const struct smoother *sm, int n, double t)
{
    const double *coeffs = w_antideriv_coeffs + n;
    double v = coeffs[2] * sm->c2;
    v = coeffs[1] * sm->c1 + v * t;
    for (; n >= 0; --n)
        v *= t;
    return v * t;
}

// Integrate scurve s(t) with a smoothing weight function w(t)
// over the range [start; end] with T == -toff
double
integrate_weighted(const struct smoother *sm, double pos, struct scurve *s
                   , double start, double end, double toff)
{
    double toff2 = toff * toff;
    // Calculate s(t) * w(t) integral as either expansion of s(t) or w(t)
    // over powers of t. w(t) expansion becomes numerically unstable when
    // abs(toff) >> hst, and s(t) - when abs(toff) >> total_accel_t.
    // Note that when abs(toff) >> hst, it means that abs(toff) ~ move_t, so
    // it is not possible that abs(toff) >> total_accel_t at the same time.
    if (toff2 > sm->h2) {
        pos += scurve_eval(s, -toff);
        scurve_offset(s, -toff);

        start += toff; end += toff;
        double res = s->c6 * (iwtn(sm, 6, end) - iwtn(sm, 6, start));
        res += s->c5 * (iwtn(sm, 5, end) - iwtn(sm, 5, start));
        res += s->c4 * (iwtn(sm, 4, end) - iwtn(sm, 4, start));
        res += s->c3 * (iwtn(sm, 3, end) - iwtn(sm, 3, start));
        res += s->c2 * (iwtn(sm, 2, end) - iwtn(sm, 2, start));
        res += s->c1 * (iwtn(sm, 1, end) - iwtn(sm, 1, start));
        res += pos * (iwtn(sm, 0, end) - iwtn(sm, 0, start));
        return res;
    } else {
        double res = sm->c2 *
            (scurve_tn_antiderivative(s, 2, end)
             - scurve_tn_antiderivative(s, 2, start));
        res += (2. * sm->c2 * toff + sm->c1) *
            (scurve_tn_antiderivative(s, 1, end)
             - scurve_tn_antiderivative(s, 1, start));
        res += (sm->c2 * toff2 + sm->c1 * toff) *
            (scurve_tn_antiderivative(s, 0, end)
             - scurve_tn_antiderivative(s, 0, start));
        start += toff; end += toff;
        res += pos * (iwtn(sm, 0, end) - iwtn(sm, 0, start));
        return res;
    }
}

struct smoother *
alloc_smoother(double target_freq, double damping_ratio)
{
    struct smoother *sm = malloc(sizeof(*sm));
    memset(sm, 0, sizeof(*sm));

    double dr2 = damping_ratio * damping_ratio;
    double hst = .5 * (0.662586 - 0.0945695 * dr2) / target_freq;
    sm->hst = hst;
    sm->h2 = hst * hst;
    double inv_hst = 1. / hst;
    double v = inv_hst * inv_hst;
    sm->c1 = (1.681147871689192 - 1.318310718147036 * dr2) * damping_ratio * v;
    v *= inv_hst;
    sm->c2 = 1.5 * v;
    return sm;
}
