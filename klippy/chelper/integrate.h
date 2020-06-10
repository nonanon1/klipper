#ifndef INTEGRATE_H
#define INTEGRATE_H

enum SMOOTHER_TYPE {
    ZVSF = 1,
    ZVDEF = 2,
    EISF05 = 3,
    EIAF05 = 4,
    DFSF05 = 5,
    DFAF05 = 6,
    DFAF02 = 7,
    DFAF01 = 8,
};

struct smoother;

typedef double (*integrate_callback)(const struct smoother *sm
                                     , double start, double end
                                     , double a0, double a1, double a2);
struct smoother {
    double c6, c5, c4, c3, c2, c1, c0;
    integrate_callback integrate_cb;
    double hst;
};


struct smoother *alloc_smoother(int smoother_type, double target_freq,
                                double damping_ratio);
double integrate_weighted(const struct smoother *sm,
                          double pos, double start_v, double half_accel,
                          double start, double end, double toff);

#endif // integrate.h
