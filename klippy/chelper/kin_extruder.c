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
#include "trapq.h" // move_get_distance

struct extruder_stepper {
    struct stepper_kinematics sk;
    double pressure_advance_factor, half_smooth_time, inv_smooth_time;
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
    // Calculate average position over smooth_time
    double area = trapq_integrate(m, 'x', move_time - hst, move_time + hst);
    double base_pos = area * es->inv_smooth_time;
    // Calculate position 'half_smooth_time' in the past
    double start_time = move_time - hst;
    struct move *sm = trapq_find_move(m, &start_time);
    double start_dist = move_get_distance(sm, start_time);
    double pa_start_pos = sm->start_pos.y + (sm->axes_r.y ? start_dist : 0.);
    // Calculate position 'half_smooth_time' in the future
    double end_time = move_time + hst;
    struct move *em = trapq_find_move(m, &end_time);
    double end_dist = move_get_distance(em, end_time);
    double pa_end_pos = em->start_pos.y + (em->axes_r.y ? end_dist : 0.);
    // Calculate position with pressure advance
    return base_pos + (pa_end_pos - pa_start_pos) * es->pressure_advance_factor;
}

void __visible
extruder_set_pressure(struct stepper_kinematics *sk
                      , double pressure_advance, double half_smooth_time)
{
    struct extruder_stepper *es = container_of(sk, struct extruder_stepper, sk);
    if (! half_smooth_time) {
        es->pressure_advance_factor = es->half_smooth_time = 0.;
        return;
    }
    es->sk.scan_past = es->sk.scan_future = half_smooth_time;
    es->half_smooth_time = half_smooth_time;
    es->inv_smooth_time = .5 / half_smooth_time;
    es->pressure_advance_factor = pressure_advance * es->inv_smooth_time;
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
extruder_add_move(struct trapq *tq, double print_time
                  , double start_e_pos, double extrude_pa_pos
                  , double extrude_r, double is_pa
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
                 start_e_pos, extrude_pa_pos, 0.,
                 1., is_pa, 0., &new_accel_decel);
}
