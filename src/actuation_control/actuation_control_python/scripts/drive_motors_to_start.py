#!/usr/bin/env python3

# Author: ML lab continuum-actuation team 
# University of California, San Diego

# Usage
# > clear; ros2 run actuation_control_python drive_motors_to_start.py CTR

# Description:
# One-shot routine to move the CTR from its home configuration into a starting
# pose for teleop. Run right after a power cycle: assumes all modules are at the
# front frame and all tube angles are zero (tube curvatures aligned in -y).
# Ramps an absolute setpoint vector at self.servo_rate (5 Hz), extending the
# translation stages outward in a collision-free order (this is the inverse of
# drive_motors_to_home.py), and publishes it on setpoint_ms
# (sensor_msgs/JointState) under the CR namespace. Rotation is homed manually.
# Waits for the user to press Enter before starting; holds the final setpoint
# once the trajectory completes.
#
# Does NOT talk to hardware directly -- feed its setpoint_ms into
# abs_motor_pos_control.py, which closes the position loop.
#
# The ramp starts from all zeros (home) and steps toward cmd_values:
# [t1, t2, t3 (mm), rot1, rot2, rot3 (deg-magnitude)].
#
# Topics (relative to the CR namespace, e.g. /CTR):
#   pub  setpoint_ms  sensor_msgs/JointState  commanded absolute motor positions
#
# TODO: RETEST ON HARDWARE

import argparse
import crtk
import sys
import sensor_msgs.msg
import threading

# custom import functions
from actuation_control import ros2_message_conversion as msg_convert

################################# User defined parameters ####################################
'''  CHANGE THIS AS NECESSARY '''
motorIDs   = ['S1', 'S6', 'S5', 'B2', 'B1', 'B3', 'B4']
cmd_values = [-75.0, -35.0, -20.0, 0.0, 2.5, 0.0, 0.0]
###############################################################################################

class drive_motors_to_start:
    def __init__(self, ral, CR_namespace):
        self.ral = ral
        self.cr = self.CR(ral.create_child(CR_namespace))

        # initialize control cycle parameters
        self.servo_rate = 5  # Hz -- rate the ramped setpoint is updated
        self.sleep_rate = ral.create_rate(self.servo_rate)

        # step size per control cycle
        self.stepper_step = 0.2       # mm
        self.bldc_step = 0.017453     # rad, equivalent to 1 deg

        # ramp bookkeeping -- start at home (all zeros) and step out to cmd_values
        self.cmds = [0.0] * len(motorIDs)
        self.count = 0
        self.trajectory_complete = False

        self.full_count_t1 = int(abs(cmd_values[0]) / self.stepper_step)
        self.full_count_t2 = int(abs(cmd_values[1]) / self.stepper_step)
        self.full_count_t3 = int(abs(cmd_values[2]) / self.stepper_step)

        self.full_count_rot1 = int(abs(cmd_values[3]) / self.bldc_step)
        self.full_count_rot2 = int(abs(cmd_values[4]) / self.bldc_step)
        self.full_count_rot3 = int(abs(cmd_values[5]) / self.bldc_step)

        self.total_count = max(self.full_count_t1, self.full_count_t2, self.full_count_t3,
                               self.full_count_rot1, self.full_count_rot2, self.full_count_rot3)

        # wait for the user to press Enter before moving
        self.waiting_for_input = True
        self.listener_thread = threading.Thread(target=self.wait_for_enter, daemon=True)
        self.listener_thread.start()

        print('Start-pose node started. Press Enter to start the trajectory.')

    def wait_for_enter(self):
        input('Press Enter to start the start-pose trajectory...')
        self.waiting_for_input = False
        print('Start-pose trajectory started.')

    def update_motor_command(self):
        self.count += 1

        for i in range(len(motorIDs) - 4):
            if self.count <= self.full_count_t3:
                self.cmds[i] += -1.0 * self.stepper_step
            elif self.full_count_t3 < self.count <= self.full_count_t2:
                if i < 2:
                    self.cmds[i] += -1.0 * self.stepper_step
            elif self.full_count_t2 < self.count <= self.full_count_t1:
                if i < 1:
                    self.cmds[i] += -1.0 * self.stepper_step

        if self.count <= self.full_count_rot1:
            self.cmds[3] += 1.0 * self.bldc_step
        if self.count <= self.full_count_rot2:
            self.cmds[4] += 1.0 * self.bldc_step
        if self.count <= self.full_count_rot3:
            self.cmds[5] += 1.0 * self.bldc_step

    def run(self):

        while True:

            if self.waiting_for_input:
                self.sleep_rate.sleep()
                continue

            if not self.trajectory_complete:
                self.update_motor_command()
                print(self.cmds)

                if self.count >= self.total_count:
                    self.trajectory_complete = True
                    print('Start-pose trajectory complete. Holding final setpoint.')

            # publish current setpoint
            self.cr.update_setpoint_ms(self.cmds)

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

    example_name = type(drive_motors_to_start).__name__
    ral = crtk.ral(example_name)

    example = drive_motors_to_start(ral, args.CR)
    ral.spin_and_execute(example.run)

if __name__ == '__main__':
    main()
