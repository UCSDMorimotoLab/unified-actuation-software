#!/usr/bin/env python3

# Author: ML lab continuum-actuation team 
# University of California, San Diego 
# Date : 2025-10-21

# Usage
# > clear; ros2 run actuation_control_python abs_motor_pos_control.py TDCR

# Description:
# Low-level position-only motor controller. Subscribes to setpoint_ms
# (sensor_msgs/JointState) under the CR namespace and drives the motors listed in
# motorIDs, publishing the measured motor positions back on measured_ms at
# self.servo_rate (100 Hz). No tension / load cell feedback -- pure position loop.
#
# Two control modes, selected by the BLOCKING flag below:
#   BLOCKING = True  -> Nanotec Profile Position mode. Each new setpoint is applied
#                       as a relative point-to-point move and the driver waits for
#                       "target reached" before the next. Good for discrete moves;
#                       poor at tracking a continuously changing setpoint.
#   BLOCKING = False -> Nanotec Cyclic Synchronous Position (CSP) mode. The driver
#                       streams the absolute target every cycle without waiting.
#                       Use this to track a trajectory (move_sine_command_motor_pos).
#
# Topics (relative to the CR namespace, e.g. /TDCR):
#   sub  setpoint_ms  sensor_msgs/JointState  target motor positions
#   pub  measured_ms  sensor_msgs/JointState  measured motor positions
#
# Pair with keyboard_command_motor_pos.py (or any node publishing setpoint_ms).


import argparse
import crtk
import sys
import sensor_msgs.msg
import numpy as np

# custom import functions
from actuation_control import nanolib_driver_code
from actuation_control import ros2_message_conversion as msg_convert 

################################# User defined parameters ####################################
'''  CHANGE THIS AS NECESSARY '''
motorIDs = ['S2','S3']

# True  -> blocking control (Profile Position mode + move/wait handshake)
# False -> non-blocking control (Cyclic Synchronous Position / CSP mode, streamed)
BLOCKING = False
###############################################################################################

class abs_motor_pos_control:
    def __init__(self,ral, CR_namespace):
        self.ral = ral 
        self.cr = self.CR(ral.create_child(CR_namespace))

        # initialize objs
        self.nanotec_driver = nanolib_driver_code.NanotecDriver(motorIDs, blocking=BLOCKING)

        # initialize control cycle parameters
        self.servo_rate = 100
        self.sleep_rate = ral.create_rate(self.servo_rate)

        # initialize variables
        self.cr_prev_mp = None
        self.csp_base = None   # raw motor pose captured when CSP tracking starts
        self.motor_res = 1e-5 #in mm

    def run(self):

        while True:

            if self.cr.setpoint_mp is None:
                self.sleep_rate.sleep()
                continue

            if BLOCKING:
                self.run_blocking()
            else:
                self.run_csp()

            self.sleep_rate.sleep() # hold the servo rate

    def run_blocking(self):
        # Profile Position mode: apply the change since the last setpoint as a
        # relative move and wait for it to finish.
        if self.cr_prev_mp is None:
            self.cr_prev_mp = self.cr.setpoint_mp
            self.nanotec_driver.drive_all_motors_relative_wait_between(self.cr.setpoint_mp)
            return

        setpoint_diff = (self.cr.setpoint_mp - self.cr_prev_mp)

        if np.any(setpoint_diff):
            # print(setpoint_diff)
            self.nanotec_driver.drive_all_motors_relative_wait_between(setpoint_diff)

        # log current measured motor states
        curr_measured_mp = self.nanotec_driver.measure_motor_abs_pos()
        self.cr.update_measured_ms(curr_measured_mp)

        # refresh prev mp
        self.cr_prev_mp = self.cr.setpoint_mp

    def run_csp(self):
        # CSP mode: stream an absolute target every cycle, no waiting.
        # abs_target = (motor pose when tracking started) + current setpoint.
        if self.csp_base is None:
            self.csp_base = np.array(self.nanotec_driver.measure_motor_abs_pos_raw(),
                                     dtype=float)

        abs_target = self.csp_base + self.cr.setpoint_mp
        self.nanotec_driver.drive_all_motors_absolute_no_wait(abs_target)

        # log current measured motor states
        curr_measured_mp = self.nanotec_driver.measure_motor_abs_pos()
        self.cr.update_measured_ms(curr_measured_mp)

    class CR: 
        def __init__(self,ral):
             self.ral = ral

             # non crtk subscribers/publishers
             self.measured_ms_pub = ral.publisher('measured_ms',
                                                  sensor_msgs.msg.JointState,
                                                  latch=True, queue_size = 1)
             self.setpoint_ms_sub = ral.subscriber('setpoint_ms',
                                                   sensor_msgs.msg.JointState,
                                                   self.setpoint_ms_cb,
                                                   queue_size =10)
            
             self.setpoint_mp = None 

        
        def update_measured_ms(self,curr_measured_mp):
            '''
            '''

            measured_ms_msg = msg_convert.convert_np_array_to_JointState(self.ral,curr_measured_mp,[],motorIDs)
            self.measured_ms_pub.publish(measured_ms_msg)
        
        def setpoint_ms_cb(self,curr_setpoint_ms):
            '''
            '''
            
            self.setpoint_mp = np.array(curr_setpoint_ms.position)
        
def main():

    parser = argparse.ArgumentParser()
    parser.add_argument('CR', type = str, help = 'ROS namespace for CRTK device')

    app_args = crtk.ral.parse_argv(sys.argv[1:]) # process and remove ROS args
    args = parser.parse_args(app_args) 

    example_name = type(abs_motor_pos_control).__name__
    ral = crtk.ral(example_name)
    
    example = abs_motor_pos_control(ral, args.CR)
    ral.spin_and_execute(example.run)

if __name__ == '__main__':
    main()

