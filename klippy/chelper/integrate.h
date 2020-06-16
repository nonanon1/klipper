#ifndef INTEGRATE_H
#define INTEGRATE_H

struct scurve;

struct smoother {
    double c2, c1;
    double hst, h2;
};

struct smoother *alloc_smoother(double target_freq, double damping_ratio);
double integrate_weighted(const struct smoother *sm,
                          double pos, struct scurve *s,
                          double start, double end, double toff);

#endif // integrate.h
