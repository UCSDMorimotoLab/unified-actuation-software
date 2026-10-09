#!/usr/bin/env python3

# Author: ML lab continuum-actuation team 
# University of California, San Diego

# Usage
# > clear; ros2 run actuation_control_python move_sine_command_motor_pos.py TDCR

# Description:
# Setpoint generator that sweeps one or more motors through a sine wave and
# publishes it on setpoint_ms (sensor_msgs/JointState) under the CR namespace.
# It does NOT talk to hardware -- hook it up to a position controller on the same
# namespace:
#
#   ros2 run actuation_control_python move_sine_command_motor_pos.py  TDCR
#   ros2 run actuation_control_python abs_motor_pos_control.py         TDCR
#     (or abs_motor_pos_pay_in_out_control.py, if it subscribes to setpoint_ms)
#
# The published waveform is zero-mean:
#   setpoint[i] = envelope(t) * A[i] * sin(2*pi*f[i]*t + phase[i])
# The downstream controller (abs_motor_pos_control) integrates the difference
# between successive setpoint_ms messages and applies it as a RELATIVE move, so
# the sweep runs about whatever position the motors are in when it starts -- it
# does NOT jump to an absolute "home" first. envelope(t) is a raised-cosine ramp
# (RAMP_S) that forces the waveform to start and end at exactly 0, so there is no
# jump on start/stop and the motors return to their starting position when the
# sweep finishes. After DURATION_S the node keeps publishing 0 (holding the start
# position) and prints "sine_completed" once.
#
# Each motor gets its own amplitude / frequency / phase, so the wave is in
# whatever quantity that motor drives:
#   - stepper 'S*' motors  -> translation, amplitude in mm
#   - BLDC    'B*' motors  -> rotation,    amplitude in rad
# Set amplitude = 0 for any motor that should stay put. For an antagonistic
# pay-in / pay-out pair, give the two motors the same amplitude/frequency and a
# 180 deg phase offset (or opposite-sign amplitudes).
#
# Topics (relative to the CR namespace, e.g. /TDCR):
#   pub  setpoint_ms  sensor_msgs/JointState  zero-mean sine offset (see above)


import argparse
import crtk
import sys
import sensor_msgs.msg
import numpy as np
import threading

# custom import functions
from actuation_control import ros2_message_conversion as msg_convert

################################# User defined parameters ####################################
'''  CHANGE THIS AS NECESSARY '''
motorIDs = ['S2', 'S3']

# per-motor sine parameters (one entry per motorID, same order)
amplitudes    = [5.0, 5.0]   # mm for 'S*' motors, rad for 'B*' motors (0 = hold)
frequencies   = [0.25, 0.25] # Hz
phases_deg    = [0.0, 180.0]  # deg -- use 180 on the partner for pay-in/pay-out

# waveform timing
DURATION_S = 20.0            # total run time of the sine sweep
RAMP_S     = 2.0             # raised-cosine ease-in / ease-out time (<= DURATION_S / 2)
###############################################################################################

class move_sine_command_motor_pos:
    def __init__(self, ral, CR_namespace):
        self.ral = ral
        self.cr = self.CR(ral.create_child(CR_namespace))

        # initialize control cycle parameters
        self.servo_rate = 50  # Hz
        self.sleep_rate = ral.create_rate(self.servo_rate)

        # validate + cache waveform parameters
        n = len(motorIDs)
        if not (len(amplitudes) == len(frequencies) == len(phases_deg) == n):
            raise ValueError('amplitudes, frequencies and phases_deg must each have '
                             'one entry per motorID (%d)' % n)

        self.amp = np.asarray(amplitudes, dtype=float)
        self.freq = np.asarray(frequencies, dtype=float)
        self.phase = np.deg2rad(np.asarray(phases_deg, dtype=float))
        self.ramp_s = min(RAMP_S, DURATION_S / 2.0)

        # runtime state
        self.start_ns = None
        self.trajectory_complete = False

        # wait for the user to press Enter before moving
        self.waiting_for_input = True
        self.listener_thread = threading.Thread(target=self.wait_for_enter, daemon=True)
        self.listener_thread.start()

        active = ', '.join(f'{motorIDs[i]}: A={self.amp[i]:g}, f={self.freq[i]:g} Hz'
                           for i in range(n) if self.amp[i] != 0.0)
        print('Sine setpoint generator started. Press Enter to start the sweep.')
        print('  sweep runs about the current motor position (relative offset).')
        print(f'  duration {DURATION_S:g} s, ramp {self.ramp_s:g} s | {active}')

    def wait_for_enter(self):
        input('Press Enter to start the sine sweep...')
        self.waiting_for_input = False
        print('Sine sweep started.')

    def envelope(self, t):
        '''Raised-cosine ramp: 0 -> 1 over [0, ramp_s], 1 -> 0 over [end - ramp_s, end].'''
        if self.ramp_s <= 0.0:
            return 1.0
        if t < self.ramp_s:
            return 0.5 * (1.0 - np.cos(np.pi * t / self.ramp_s))
        if t > DURATION_S - self.ramp_s:
            return 0.5 * (1.0 - np.cos(np.pi * (DURATION_S - t) / self.ramp_s))
        return 1.0

    def setpoint_at(self, t):
        # zero-mean offset about the motors' position at sweep start; the
        # downstream controller integrates successive setpoint_ms as relative moves
        env = max(0.0, self.envelope(t))
        return env * self.amp * np.sin(2.0 * np.pi * self.freq * t + self.phase)

    def run(self):

        while True:

            if self.waiting_for_input:
                self.sleep_rate.sleep()
                continue

            if self.start_ns is None:
                self.start_ns = self.ral.now().nanoseconds

            t = (self.ral.now().nanoseconds - self.start_ns) / 1e9

            if not self.trajectory_complete and t < DURATION_S:
                setpoint_mp = self.setpoint_at(t)
            else:
                # sweep finished -- waveform is back at 0, so keep publishing 0 to
                # hold the motors at the position they started the sweep from
                if not self.trajectory_complete:
                    self.trajectory_complete = True
                    print('sine_completed')
                setpoint_mp = np.zeros(len(motorIDs))

            self.cr.update_setpoint_ms(setpoint_mp.tolist())

            self.sleep_rate.sleep()

    class CR:
        def __init__(self, ral):
            self.ral = ral
            self.setpoint_ms_pub = ral.publisher('setpoint_ms',
                                                 sensor_msgs.msg.JointState,
                                                 latch=True, queue_size = 1)

        def update_setpoint_ms(self, curr_setpoint_mp):

            setpoint_ms_msg = msg_convert.convert_np_array_to_JointPosition(self.ral, curr_setpoint_mp, motorIDs)
            self.setpoint_ms_pub.publish(setpoint_ms_msg)

def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('CR', type = str, help = 'ROS namespace for CRTK device')

    app_args = crtk.ral.parse_argv(sys.argv[1:]) # process and remove ROS args
    args = parser.parse_args(app_args)

    example_name = type(move_sine_command_motor_pos).__name__
    ral = crtk.ral(example_name)

    example = move_sine_command_motor_pos(ral, args.CR)
    ral.spin_and_execute(example.run)

if __name__ == '__main__':
    main()
