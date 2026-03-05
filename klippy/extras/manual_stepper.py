# Support for a manual controlled stepper
#
# Copyright (C) 2019-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
import stepper
from . import force_move
from enum import Enum

class ManualStepper:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        if config.get('endstop_pin', None) is not None:
            self.can_home = True
            self.rail = stepper.LookupRail(
                config, need_position_minmax=False, default_position_endstop=0.)
            self.steppers = self.rail.get_steppers()
        else:
            self.can_home = False
            self.rail = stepper.PrinterStepper(config)
            self.steppers = [self.rail]
        self.velocity = config.getfloat('velocity', 5., above=0.)
        self.accel = self.homing_accel = config.getfloat('accel', 0., minval=0.)
        self.next_cmd_time = 0.
        self.commanded_pos = 0.
        self.pos_min = config.getfloat('position_min', None)
        self.pos_max = config.getfloat('position_max', None)
        # Setup iterative solver
        self.motion_queuing = self.printer.load_object(config, 'motion_queuing')
        self.trapq = self.motion_queuing.allocate_trapq()
        self.trapq_append = self.trapq_append_helper
        self.rail.setup_itersolve('cartesian_stepper_alloc', b'x')
        self.rail.set_trapq(self.trapq)
        # Registered with toolhead as an axtra axis
        self.axis_gcode_id = None
        self.instant_corner_v = 0.
        self.gaxis_limit_velocity = self.gaxis_limit_accel = 0.
        # Register commands
        stepper_name = self.name.split()[1]
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command('MANUAL_STEPPER', "STEPPER",
                                   stepper_name, self.cmd_MANUAL_STEPPER,
                                   desc=self.cmd_MANUAL_STEPPER_help)
    def trapq_append_helper(self, *args, **kwargs):
        print(f">>>>>>>>>> Calling trapq_append with: {args}")
        return self.motion_queuing.lookup_trapq_append()(*args, **kwargs)
    def get_name(self):
        return self.name
    def sync_print_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time
    def get_toolhead_last_move_time(self):
        return self.printer.lookup_object('toolhead').get_last_move_time()
    def sync_toolhead_next_move_time(self):
        # only set if next_cmd_time is in the future
        if self.next_cmd_time > self.get_toolhead_last_move_time():
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.dwell(self.next_cmd_time - self.get_toolhead_last_move_time())
    def do_enable(self, enable):
        stepper_names = [s.get_name() for s in self.steppers]
        stepper_enable = self.printer.lookup_object('stepper_enable')
        stepper_enable.set_motors_enable(stepper_names, enable)
    def do_set_position(self, setpos):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        self.commanded_pos = setpos
        self.rail.set_position([self.commanded_pos, 0., 0.])
    def _submit_move(self, move_start_time, end_pos, speed, accel, sync, easing):
        def move_constant_speed(dist):
            return force_move.calc_move_time(dist, speed, 0)
        
        def move_accel_once(dist, max_speed):
            nonlocal speed
            axis_ratio = 1.
            if dist < 0.:
                axis_ratio = -1.
                dist = -dist
            if not accel or not dist:
                return axis_r, 0., dist / max_speed, max_speed

            max_cruise_v2 = dist * accel
            if max_cruise_v2 < max_speed**2:
                max_speed = math.sqrt(max_cruise_v2)

            accel_time = max_speed / accel
            accel_dist = (accel_time * max_speed) / 2.
            cruise_time = (dist - accel_dist) / max_speed
            return axis_ratio, accel_time, cruise_time, max_speed

        if sync == "AFTER_PREV":
            move_start_time = self.get_toolhead_last_move_time()

        start_pos = self.commanded_pos
        move_dist = end_pos - start_pos

        print(f">>>>>>>>>> cp:{start_pos} dist:{move_dist}")
        # this is only allowed during constant speed travel
        if sync == "WITH_PREV" and not easing:
            print(f">>>>>>>>>> synchronising move with previous G1 command")
            toolhead_last_move_time = self.get_toolhead_last_move_time()
            toolhead_move_time = toolhead_last_move_time - move_start_time

            # try to do constant v move
            axis_r, accel_t, cruise_t, cruise_v = move_constant_speed(move_dist)
            manual_move_time = accel_t*2 + cruise_t
            
            if toolhead_move_time <= manual_move_time:
                print(">>>>>>>>>> constant v")
                self.trapq_append(self.trapq, move_start_time,
                                  0, cruise_t, 0,
                                  start_pos, 0., 0., axis_r, 0., 0.,
                                  0., cruise_v, 0)
                return move_start_time + cruise_t
            else:
                print(">>>>>>>>>> decel accel")
                # recalc timings for half distance move with acceleration
                axis_r, accel_t, cruise_t, cruise_v = move_accel_once(move_dist / 2, speed)
                half_segment_t = cruise_t + accel_t
                dwell_t = max(toolhead_move_time - half_segment_t*2, 0)
                self.trapq_append(self.trapq, move_start_time,
                                  0, cruise_t, accel_t,
                                  start_pos, 0., 0., axis_r, 0., 0.,
                                  cruise_v, cruise_v, accel)
                self.trapq_append(self.trapq, move_start_time + half_segment_t + dwell_t,
                                  accel_t, cruise_t, 0,
                                  start_pos + move_dist / 2, 0., 0., axis_r, 0., 0.,
                                  0, cruise_v, accel)
                return move_start_time + half_segment_t + dwell_t + half_segment_t
        else:
            print(">>>>>>>>>> not synchronising move with previous G1 command")
            if easing == "BEGIN":
                print(">>>>>>>>>> easing=BEGIN")
                axis_r, accel_t, cruise_t, cruise_v = move_accel_once(move_dist, speed)
                self.trapq_append(self.trapq, move_start_time,
                                  accel_t, cruise_t, 0,
                                  start_pos, 0., 0., axis_r, 0., 0.,
                                  0., cruise_v, accel)
                return move_start_time + accel_t + cruise_t
            if easing == "END":
                print(">>>>>>>>>> easing=END")
                axis_r, accel_t, cruise_t, cruise_v = move_accel_once(move_dist, speed)
                self.trapq_append(self.trapq, move_start_time,
                                  0, cruise_t, accel_t,
                                  start_pos, 0., 0., axis_r, 0., 0.,
                                  cruise_v, cruise_v, accel)
                return move_start_time + accel_t + cruise_t
            else: # cruising only
                print(">>>>>>>>>> easing=None")
                axis_r, accel_t, cruise_t, cruise_v = move_constant_speed(move_dist)
                self.trapq_append(self.trapq, move_start_time,
                                  0, cruise_t, 0,
                                  start_pos, 0., 0., axis_r, 0., 0.,
                                  0., cruise_v, 0)
                return move_start_time + cruise_t
    def do_move(self, movepos, speed, accel, sync, easing):
        # sanity check
        if self.next_cmd_time == 0:
            self.next_cmd_time = self.get_toolhead_last_move_time()

        self.next_cmd_time = self._submit_move(self.next_cmd_time, movepos, speed, accel, sync, easing)
        self.motion_queuing.note_mcu_movequeue_activity(self.next_cmd_time)
        self.commanded_pos = movepos
        self.sync_toolhead_next_move_time()
    def do_homing_move(self, movepos, speed, accel,
                       probe_pos, triggered, check_trigger):
        if not self.can_home:
            raise self.printer.command_error(
                "No endstop for this manual stepper")
        self.homing_accel = accel
        pos = [movepos, 0., 0., 0.]
        endstops = self.rail.get_endstops()
        phoming = self.printer.lookup_object('homing')
        phoming.manual_home(self, endstops, pos, speed,
                            probe_pos, triggered, check_trigger)
        self.sync_print_time()
    cmd_MANUAL_STEPPER_help = "Command a manually configured stepper"
    def cmd_MANUAL_STEPPER(self, gcmd):
        if gcmd.get('GCODE_AXIS', None) is not None:
            return self.command_with_gcode_axis(gcmd)
        if self.axis_gcode_id is not None:
            raise gcmd.error("Must unregister from gcode axis first")
        enable = gcmd.get_int('ENABLE', None)
        if enable is not None:
            self.do_enable(enable)
        setpos = gcmd.get_float('SET_POSITION', None)
        if setpos is not None:
            self.do_set_position(setpos)
        speed = gcmd.get_float('SPEED', self.velocity, above=0.)
        accel = gcmd.get_float('ACCEL', self.accel, minval=0.)
        homing_move = gcmd.get('STOP_ON_ENDSTOP', None)
        if homing_move is not None:
            old_map = {'-2': 'try_inverted_home', '-1': 'inverted_home',
                       '1': 'home', '2': 'try_home'}.get(homing_move)
            if old_map is not None:
                pconfig = self.printer.lookup_object('configfile')
                pconfig.deprecate_gcode("MANUAL_STEPPER", "STOP_ON_ENDSTOP",
                                        homing_move)
                homing_move = old_map
            is_try = homing_move.startswith('try_')
            homing_move = homing_move[is_try*4:]
            is_inverted = homing_move.startswith('inverted_')
            homing_move = homing_move[is_inverted*9:]
            if homing_move not in ["probe", "home"]:
                raise gcmd.error("Unknown STOP_ON_ENDSTOP request")
            is_probe = (homing_move == "probe")
            movepos = gcmd.get_float('MOVE')
            if ((self.pos_min is not None and movepos < self.pos_min)
                or (self.pos_max is not None and movepos > self.pos_max)):
                raise gcmd.error("Move out of range")
            self.do_homing_move(movepos, speed, accel,
                                is_probe, not is_inverted, not is_try)
        elif gcmd.get_float('MOVE', None) is not None:
            movepos = gcmd.get_float('MOVE')
            if ((self.pos_min is not None and movepos < self.pos_min)
                or (self.pos_max is not None and movepos > self.pos_max)):
                raise gcmd.error("Move out of range")
            sync = gcmd.get('SYNC', None)
            easing = gcmd.get('EASING', None)
            self.do_move(movepos, speed, accel, sync, easing)
        elif gcmd.get_int('SYNC', 0):
            self.sync_print_time()
    # Register as a gcode axis
    def command_with_gcode_axis(self, gcmd):
        gcode_move = self.printer.lookup_object("gcode_move")
        toolhead = self.printer.lookup_object('toolhead')
        gcode_axis = gcmd.get('GCODE_AXIS').upper()
        instant_corner_v = gcmd.get_float('INSTANTANEOUS_CORNER_VELOCITY', 1.,
                                          minval=0.)
        limit_velocity = gcmd.get_float('LIMIT_VELOCITY', 999999.9, above=0.)
        limit_accel = gcmd.get_float('LIMIT_ACCEL', 999999.9, above=0.)
        if self.axis_gcode_id is not None:
            if gcode_axis:
                raise gcmd.error("Must unregister axis first")
            # Unregister
            toolhead.remove_extra_axis(self)
            self.axis_gcode_id = None
            return
        if (len(gcode_axis) != 1 or not gcode_axis.isupper()
            or gcode_axis in "XYZEFN"):
            if not gcode_axis:
                # Request to unregister already unregistered axis
                return
            raise gcmd.error("Not a valid GCODE_AXIS")
        for ea in toolhead.get_extra_axes():
            if ea is not None and ea.get_axis_gcode_id() == gcode_axis:
                raise gcmd.error("Axis '%s' already registered" % (gcode_axis,))
        self.axis_gcode_id = gcode_axis
        self.instant_corner_v = instant_corner_v
        self.gaxis_limit_velocity = limit_velocity
        self.gaxis_limit_accel = limit_accel
        toolhead.add_extra_axis(self, self.commanded_pos)
    def process_move(self, print_time, move, ea_index):
        axis_r = move.axes_r[ea_index]
        start_pos = move.start_pos[ea_index]
        accel = move.accel * axis_r
        start_v = move.start_v * axis_r
        cruise_v = move.cruise_v * axis_r
        self.trapq_append(self.trapq, print_time,
                          move.accel_t, move.cruise_t, move.decel_t,
                          start_pos, 0., 0.,
                          1., 0., 0.,
                          start_v, cruise_v, accel)
        self.commanded_pos = move.end_pos[ea_index]
    def check_move(self, move, ea_index):
        # Check move is in bounds
        movepos = move.end_pos[ea_index]
        if ((self.pos_min is not None and movepos < self.pos_min)
            or (self.pos_max is not None and movepos > self.pos_max)):
            raise move.move_error()
        # Check if need to limit maximum velocity and acceleration
        axis_ratio = move.move_d / abs(move.axes_d[ea_index])
        limit_velocity = self.gaxis_limit_velocity * axis_ratio
        limit_accel = self.gaxis_limit_accel * axis_ratio
        if not move.is_kinematic_move and self.accel:
            limit_accel = min(limit_accel, self.accel * axis_ratio)
        move.limit_speed(limit_velocity, limit_accel)
    def calc_junction(self, prev_move, move, ea_index):
        diff_r = move.axes_r[ea_index] - prev_move.axes_r[ea_index]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2
    def get_axis_gcode_id(self):
        return self.axis_gcode_id
    def get_trapq(self):
        return self.trapq
    # Toolhead wrappers to support homing
    def flush_step_generation(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
    def get_position(self):
        return [self.commanded_pos, 0., 0., 0.]
    def set_position(self, newpos, homing_axes=""):
        self.do_set_position(newpos[0])
    # def get_last_move_time(self):
    #     self.sync_print_time()
    #     return self.next_cmd_time
    def dwell(self, delay):
        self.next_cmd_time += max(0., delay)
    # def drip_move(self, newpos, speed, drip_completion):
    #     # Submit move to trapq
    #     self.sync_print_time()
    #     start_time = self.next_cmd_time
    #     end_time = self._submit_move(start_time, newpos[0],
    #                                  speed, self.homing_accel)
    #     # Drip updates to motors
    #     self.motion_queuing.drip_update_time(start_time, end_time,
    #                                          drip_completion)
    #     # Clear trapq of any remaining parts of movement
    #     self.motion_queuing.wipe_trapq(self.trapq)
    def get_kinematics(self):
        return self
    def get_steppers(self):
        return self.steppers
    def calc_position(self, stepper_positions):
        return [stepper_positions[self.rail.get_name()], 0., 0.]

def load_config_prefix(config):
    return ManualStepper(config)
