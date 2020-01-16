// Extruder stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stddef.h> // offsetof
#include <stdlib.h> // malloc
#include <string.h> // memset
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "scurve.h" // scurve_eval, scurve_integrate
#include "trapq.h" // move_get_distance

// Without pressure advance, the extruder stepper position is:
//     extruder_position(t) = nominal_position(t)
// When pressure advance is enabled, additional filament is pushed
// into the extruder during acceleration (and retracted during
// deceleration). The formula is:
//     pa_position(t) = (nominal_position(t)
//                       + pressure_advance * nominal_velocity(t))
// Which is then "smoothed" using a weighted average:
//     smooth_position(t) = (
//         definitive_integral(pa_position(x) * (smooth_time/2 - abs(t-x)) * dx,
//                             from=t-smooth_time/2, to=t+smooth_time/2)
//         / ((smooth_time/2)**2))

// Calculate the definitive integral of the motion formula:
//   position(t) = base + s(t) + pa * s'(t), with s(t) - Bezier S-Curve
static double
extruder_integrate(struct move *m, double start, double end)
{
    // Calculate base position and velocity with pressure advance
    double pressure_advance = m->axes_r.y;
    double pa_add = pressure_advance * scurve_diff(&m->s, start, end);
    double base = m->start_pos.x * (end - start);
    double integral = scurve_integrate(&m->s, start, end);
    return base + integral + pa_add;
}

// Calculate the definitive integral of time weighted position:
//   weighted_position(t) = t * (base + s(t) + pa * s'(t))
static double
extruder_integrate_time(struct move *m, double start, double end)
{
    // Calculate base position and velocity with pressure advance
    double pressure_advance = m->axes_r.y;
    double pa_add = pressure_advance
        * scurve_deriv_t_integrate(&m->s, start, end);
    double base = .5 * m->start_pos.x * (end * end - start * start);
    double integral = scurve_integrate_t(&m->s, start, end);
    return base + integral + pa_add;
}

// Calculate the definitive integral of extruder for a given move
static double
pa_move_integrate(struct move *m, double start, double end, double time_offset)
{
    if (start < 0.)
        start = 0.;
    if (end > m->move_t)
        end = m->move_t;
    double iext = extruder_integrate(m, start, end);
    double wgt_ext = extruder_integrate_time(m, start, end);
    return wgt_ext - time_offset * iext;
}

// Calculate the definitive integral of the extruder over a range of moves
static double
pa_range_integrate(struct move *m, double move_time, double hst)
{
    // Calculate integral for the current move
    double res = 0., start = move_time - hst, end = move_time + hst;
    res += pa_move_integrate(m, start, move_time, start);
    res -= pa_move_integrate(m, move_time, end, end);
    // Integrate over previous moves
    struct move *prev = m;
    while (unlikely(start < 0.)) {
        prev = list_prev_entry(prev, node);
        start += prev->move_t;
        res += pa_move_integrate(prev, start, prev->move_t, start);
    }
    // Integrate over future moves
    while (unlikely(end > m->move_t)) {
        end -= m->move_t;
        m = list_next_entry(m, node);
        res -= pa_move_integrate(m, 0., end, end);
    }
    return res;
}

struct extruder_stepper {
    struct stepper_kinematics sk;
    double half_smooth_time, inv_half_smooth_time2;
};

static double
extruder_calc_position(struct stepper_kinematics *sk, struct move *m
                       , double move_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    double hst = es->half_smooth_time;
    if (!hst)
        // Pressure advance not enabled
        return m->start_pos.x + move_get_distance(m, move_time);
    // Apply pressure advance and average over smooth_time
    double area = pa_range_integrate(m, move_time, hst);
    return area * es->inv_half_smooth_time2;
}

void __visible
extruder_set_smooth_time(struct stepper_kinematics *sk, double smooth_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    double hst = smooth_time * .5;
    es->half_smooth_time = hst;
    es->sk.gen_steps_pre_active = es->sk.gen_steps_post_active = hst;
    if (! hst)
        return;
    es->inv_half_smooth_time2 = 1. / (hst * hst);
}

struct stepper_kinematics * __visible
extruder_stepper_alloc(void)
{
    struct extruder_stepper *es = malloc(sizeof(*es));
    memset(es, 0, sizeof(*es));
    es->sk.calc_position_cb = extruder_calc_position;
    es->sk.active_flags = AF_X;
    return &es->sk;
}

// Populate a 'struct move' with an extruder velocity trapezoid
void __visible
extruder_add_move(struct trapq *tq, double print_time, double start_e_pos
                  , double extrude_r, double pressure_advance
                  , const struct trap_accel_decel *accel_decel)
{
    // NB: acceleration compensation reduces duration of moves in the beginning
    // of acceleration move group, and increases it in case of deceleration.
    // The extruder kinematics does not follow acceleration compensation, so
    // print_time must be adjusted accordingly to track the start and the
    // duration of the non-compensated moves.
    if (accel_decel->total_accel_t)
        print_time += accel_decel->uncomp_accel_offset_t
            - accel_decel->accel_offset_t;
    else if (accel_decel->total_decel_t)
        print_time += accel_decel->uncomp_decel_offset_t
            - accel_decel->decel_offset_t;

    // Generate trap_accel_decel for extruder
    struct trap_accel_decel new_accel_decel = *accel_decel;
    // Disable acceleration compensation
    new_accel_decel.accel_comp = 0.;
    new_accel_decel.accel_t = new_accel_decel.uncomp_accel_t;
    new_accel_decel.accel_offset_t = new_accel_decel.uncomp_accel_offset_t;
    new_accel_decel.decel_t = new_accel_decel.uncomp_decel_t;
    new_accel_decel.decel_offset_t = new_accel_decel.uncomp_decel_offset_t;

    new_accel_decel.start_accel_v *= extrude_r;
    new_accel_decel.cruise_v *= extrude_r;
    new_accel_decel.effective_accel *= extrude_r;
    new_accel_decel.effective_decel *= extrude_r;

    // Queue movement (x is extruder movement, y is movement with pa)
    trapq_append(tq, print_time,
                 start_e_pos, 0., 0.,
                 1., pressure_advance, 0., &new_accel_decel);
}
