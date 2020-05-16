#ifndef INTEGRATE_H
#define INTEGRATE_H

enum SMOOTHER_TYPE {
    SIFP05 = 1,
    SIAF05 = 2,
    DFSF05 = 3,
    DFAF05 = 4,
    DFAF02 = 5,
    DFAF01 = 6,
};

struct smoother;

typedef double (*integrate_callback)(const struct smoother *sm
                                     , double start, double end
                                     , double a0, double a1, double a2);
struct smoother {
    double c6, c4, c2, c0;
    integrate_callback integrate_cb;
    double hst;
};


struct smoother *alloc_smoother(int smoother_type, double target_freq,
                                double damping_ratio);
double integrate_weighted(const struct smoother *sm,
                          double pos, double start_v, double half_accel,
                          double start, double end, double toff);

#endif // integrate.h
