#!/usr/bin/env python3

# Author: ML lab continuum-actuation team 
# University of California, San Diego

# Usage
# > clear; ros2 run actuation_control_python keyboard_command_motor_pos_pay_in_out.py TDCR

# Description:
# Keyboard teleop that generates absolute motor position setpoints for
# antagonistic (pay-in / pay-out) motor PAIRS and publishes them on setpoint_ms
# (sensor_msgs/JointState) under the CR namespace. Same idea as
# keyboard_command_motor_pos.py, but a key drives two motors at once: one paid in,
# the other paid out by the same amount.
#
# Does NOT talk to hardware -- feed its setpoint_ms into abs_motor_pos_control.py
# on the same namespace, which closes the position loop (BLOCKING flag there
# picks Profile Position vs CSP). The underlying motor control is identical to the
# plain keyboard node; only the setpoint pattern differs.
#
# motorIDs must be an even-length list; motors are grouped into consecutive pairs
# (0,1), (2,3), (4,5), ... Key 0 -> pair (0,1), key 1 -> pair (2,3), etc.
#
# Controls (hold keys):
#   0,1,2,...   select antagonistic pair (key i -> motors 2i, 2i+1)
#   w / s       pay motor 2i in / out (motor 2i+1 moves the opposite way)
#   z           slow mode (0.1x step size) while held
# Step size: 'S' (stepper) motors move self.step_size mm per cycle; 'B' (BLDC)
# motors move self.bldc_scaled_step rad per cycle.
#
# Topics:
#   pub  setpoint_ms  sensor_msgs/JointState  commanded motor positions


import argparse
import crtk
import sys
import sensor_msgs.msg
import numpy as np

from inputs import get_key
import threading

# custom import functions
from actuation_control import ros2_message_conversion as msg_convert

################################# User defined parameters ####################################
'''  CHANGE THIS AS NECESSARY '''
motorIDs = ['S2', 'S3']  # even-length; consecutive entries form antagonistic pairs
###############################################################################################

class keyboard_command_motor_pos_pay_in_out:
    def __init__(self,ral, CR_namespace):
        self.ral = ral
        self.cr = self.CR(ral.create_child(CR_namespace))

        # initialize control cycle parameters
        self.servo_rate = 1/0.10 # ~10 Hz -- otherwise keyboard input would skip like crazy!
        self.sleep_rate = ral.create_rate(self.servo_rate)

        # keyboard related variables
        self.held_keys = set()
        self.lock = threading.Lock()
        self.listener_thread = threading.Thread(target=self.key_listener, daemon=True)
        self.listener_thread.start()

        # Direct step size for Stepper
        self.step_size = .2  # mm

        # Scaled step size for BLDC
        self.bldc_scaled_step = 2 * np.pi / 5 / 10 * self.step_size / 10

        # build pair map: key '0' -> indices (0,1), '1' -> (2,3), etc.
        n_pairs = len(motorIDs) // 2
        self.pair_map = {str(i): (2 * i, 2 * i + 1) for i in range(n_pairs)}

        self.setpoint_mp = [0.0] * len(motorIDs)

        pair_info = ', '.join(f"key {k} -> ({motorIDs[v[0]]}, {motorIDs[v[1]]})"
                              for k, v in self.pair_map.items())
        print(f'Keyboard pay-in/out teleop started. Hold pair key + "w"/"s" (+ "z" slow). Pairs: {pair_info}')

    def key_listener(self):
        while not self.ral.is_shutdown():
            events = get_key()
            with self.lock:
                for event in events:
                    if event.ev_type == 'Key':
                        key = event.code.replace('KEY_', '').lower()
                        if event.state == 1:
                            self.held_keys.add(key)
                        elif event.state == 0:
                            self.held_keys.discard(key)

    def step_motor(self, idx, signed_dir):
        if 'S' in motorIDs[idx]:
            self.setpoint_mp[idx] += signed_dir * self.step_size
        elif 'B' in motorIDs[idx]:
            self.setpoint_mp[idx] += signed_dir * self.bldc_scaled_step

    def run(self):
        while True:
            with self.lock:
                active_pair = None
                direction = 0.0

                for key in self.held_keys:
                    if key in self.pair_map:
                        active_pair = self.pair_map[key]

                if 'w' in self.held_keys:
                    direction = 1.0
                elif 's' in self.held_keys:
                    direction = -1.0

                scale_speed = 0.1 if 'z' in self.held_keys else 1.0

            if active_pair is not None and direction != 0.0:
                i_in, i_out = active_pair
                self.step_motor(i_in,   direction * scale_speed)   # pay in
                self.step_motor(i_out, -direction * scale_speed)   # pay out
                print(self.setpoint_mp)

            # publish setpoint_ms
            self.cr.update_setpoint_ms(self.setpoint_mp)

            self.sleep_rate.sleep()

    class CR:
        def __init__(self,ral):
             self.ral = ral
             self.setpoint_ms_pub = ral.publisher('setpoint_ms',
                                                  sensor_msgs.msg.JointState,
                                                  latch=True, queue_size = 1)


        def update_setpoint_ms(self,curr_setpoint_mp):

            setpoint_ms_msg = msg_convert.convert_np_array_to_JointPosition(self.ral,curr_setpoint_mp,motorIDs)
            self.setpoint_ms_pub.publish(setpoint_ms_msg)


def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('CR', type = str, help = 'ROS namespace for CRTK device')

    app_args = crtk.ral.parse_argv(sys.argv[1:]) # process and remove ROS args
    args = parser.parse_args(app_args)

    example_name = type(keyboard_command_motor_pos_pay_in_out).__name__
    ral = crtk.ral(example_name)

    example = keyboard_command_motor_pos_pay_in_out(ral, args.CR)
    ral.spin_and_execute(example.run)

if __name__ == '__main__':
    main()
