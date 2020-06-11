# Positional smoother on cartesian XY axes
#
# Copyright (C) 2019-2020  Kevin O'Connor <kevin@koconnor.net>
# Copyright (C) 2020  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import chelper

MAX_ACCEL_COMPENSATION = 0.005

class SmoothAxis:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.connect)
        self.toolhead = None
        self.damping_ratio_x = config.getfloat(
                'damping_ratio_x', 0., minval=0., maxval=1.)
        self.damping_ratio_y = config.getfloat(
                'damping_ratio_y', 0., minval=0., maxval=1.)
        smooth_x = config.getfloat('smooth_x', 0., minval=0., maxval=.200)
        smooth_y = config.getfloat('smooth_y', 0., minval=0., maxval=.200)
        accel_comp_x = config.getfloat(
                'accel_comp_x', (3. * smooth_x / (4. * math.pi))**2,
                minval=0., maxval=MAX_ACCEL_COMPENSATION)
        accel_comp_y = config.getfloat(
                'accel_comp_y', (3. * smooth_y / (4. * math.pi))**2,
                minval=0., maxval=MAX_ACCEL_COMPENSATION)
        self.smoother_freq_x = config.getfloat(
                'smoother_freq_x',
                (1. / (2. * math.pi * math.sqrt(accel_comp_x))
                    if accel_comp_x > 0 else 0.),
                minval=0.)
        self.smoother_freq_y = config.getfloat(
                'smoother_freq_y',
                (1. / (2. * math.pi * math.sqrt(accel_comp_y))
                    if accel_comp_y > 0 else 0.),
                minval=0.)
        self.stepper_kinematics = []
        self.orig_stepper_kinematics = []
        # Stepper kinematics code
        ffi_main, ffi_lib = chelper.get_ffi()
        self._set_time = ffi_lib.smooth_axis_set_time
        self._set_sk = ffi_lib.smooth_axis_set_sk
        # Register gcode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("SET_SMOOTH_AXIS",
                               self.cmd_SET_SMOOTH_AXIS,
                               desc=self.cmd_SET_SMOOTH_AXIS_help)
    def connect(self):
        self.toolhead = self.printer.lookup_object("toolhead")
        kin = self.toolhead.get_kinematics()
        # Lookup stepper kinematics
        ffi_main, ffi_lib = chelper.get_ffi()
        steppers = kin.get_steppers()
        for s in steppers:
            sk = ffi_main.gc(ffi_lib.smooth_axis_alloc(), ffi_lib.free)
            orig_sk = s.set_stepper_kinematics(sk)
            res = ffi_lib.smooth_axis_set_sk(sk, orig_sk)
            if res < 0:
                s.set_stepper_kinematics(orig_sk)
                continue
            s.set_trapq(self.toolhead.get_trapq())
            self.stepper_kinematics.append(sk)
            self.orig_stepper_kinematics.append(orig_sk)
        # Configure initial values
        config_smoother_freq_x = self.smoother_freq_x
        config_smoother_freq_y = self.smoother_freq_y
        self.smoother_freq_x = self.smoother_freq_y = 0.
        self._set_smoothing(config_smoother_freq_x, config_smoother_freq_y,
                            self.damping_ratio_x, self.damping_ratio_y)
    def _set_smoothing(self, smoother_freq_x, smoother_freq_y
                       , damping_ratio_x, damping_ratio_y):
        max_old_freq = max(self.smoother_freq_x, self.smoother_freq_y)
        old_smooth_time = 1. / (3. * max_old_freq) if max_old_freq > 0 else 0.
        max_new_freq = max(smoother_freq_x, smoother_freq_y)
        new_smooth_time = 1. / (3. * max_new_freq) if max_new_freq > 0 else 0.
        self.toolhead.note_step_generation_scan_time(new_smooth_time,
                                                     old_delay=old_smooth_time)
        self.smoother_freq_x = smoother_freq_x
        self.smoother_freq_y = smoother_freq_y
        self.damping_ratio_x = damping_ratio_x
        self.damping_ratio_y = damping_ratio_y
        if smoother_freq_x > 0:
            accel_comp_x = 1. / (2. * math.pi * smoother_freq_x)**2
            smooth_x = 2. / (3 * smoother_freq_x)
        else:
            accel_comp_x = smooth_x = 0.
        if smoother_freq_y > 0:
            accel_comp_y = 1. / (2. * math.pi * smoother_freq_y)**2
            smooth_y = 2. / (3 * smoother_freq_y)
        else:
            accel_comp_y = smooth_y = 0.
        ffi_main, ffi_lib = chelper.get_ffi()
        for sk in self.stepper_kinematics:
            ffi_lib.smooth_axis_set_time(sk, smooth_x, smooth_y)
            ffi_lib.smooth_axis_set_damping_ratio(sk, damping_ratio_x,
                                                  damping_ratio_y)
            ffi_lib.smooth_axis_set_accel_comp(sk, accel_comp_x, accel_comp_y)
    cmd_SET_SMOOTH_AXIS_help = "Set cartesian time smoothing parameters"
    def cmd_SET_SMOOTH_AXIS(self, gcmd):
        damping_ratio_x = gcmd.get_float(
                'DAMPING_RATIO_X', self.damping_ratio_x, minval=0., maxval=1.)
        damping_ratio_y = gcmd.get_float(
                'DAMPING_RATIO_Y', self.damping_ratio_y, minval=0., maxval=1.)
        smoother_freq_x = gcmd.get_float(
                'SMOOTHER_FREQ_X', self.smoother_freq_x, minval=0.)
        smoother_freq_y = gcmd.get_float(
                'SMOOTHER_FREQ_Y', self.smoother_freq_y, minval=0.)
        self._set_smoothing(smoother_freq_x, smoother_freq_y,
                            damping_ratio_x, damping_ratio_y)
        gcmd.respond_info("smoother_freq_x:%.3f smoother_freq_y:%.3f "
                          "damping_ratio_x:%.6f damping_ratio_y:%.6f" % (
                              smoother_freq_x, smoother_freq_y,
                              damping_ratio_x, damping_ratio_y))

def load_config(config):
    return SmoothAxis(config)
