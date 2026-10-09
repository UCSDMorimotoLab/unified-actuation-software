#!/usr/bin/env python3

# Author: ML lab continuum-actuation team 
# University of California, San Diego 
# Date : 2025-10-21

# Usage
# > clear; ros2 run actuation_control_python keyboard_command_motor_pos.py TDCR

# Description:
# Keyboard teleop that generates absolute motor position setpoints. Runs a
# background thread reading held keys (via `inputs`) and, on each control cycle
# (~10 Hz), integrates the active key into an internal setpoint vector, then
# publishes it on setpoint_ms (sensor_msgs/JointState) under the CR namespace.
# Does NOT talk to hardware directly -- feed its setpoint_ms into
# abs_motor_pos_control.py, which closes the position loop.
#
# Controls (hold keys):
#   0-5        select motor by index into motorIDs
#   w / s      increment / decrement the selected motor's setpoint
#   z          slow mode (0.1x step size) while held
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
motorIDs = ['S2', 'S3']
###############################################################################################

class keyboard_command_motor_pos:
    def __init__(self,ral, CR_namespace):
        self.ral = ral 
        self.cr = self.CR(ral.create_child(CR_namespace))

        # initialize control cycle parameters 
        self.servo_rate = 1/0.10 # 20Hz log -- otherwise keyboard input would skip like crazy! 
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

        self.setpoint_mp =  [0.0] * len(motorIDs)  

        print('Keyboard teleop node started. Hold motor key [0–5] + "w"/"s".')
    
    def key_listener(self):
        while ~self.ral.is_shutdown():
            events = get_key()
            with self.lock:
                for event in events:
                    if event.ev_type == 'Key':
                        key = event.code.replace('KEY_', '').lower()
                        if event.state == 1:
                            self.held_keys.add(key)
                        elif event.state == 0:
                            self.held_keys.discard(key)
    
    def run(self):
        while True: 
            with self.lock:
                active_motor = None
                direction = 0.0

                for key in self.held_keys:
                    if key in {'0', '1', '2', '3', '4', '5'}:
                        active_motor = int(key)

                if 'w' in self.held_keys:
                    direction = 1.0
                elif 's' in self.held_keys:
                    direction = -1.0

                if 'z' in self.held_keys:
                    scale_speed = 0.1
                else:
                    scale_speed = 1.0


            if active_motor is not None and direction != 0.0:
                if 'S' in motorIDs[active_motor]:
                    self.setpoint_mp[active_motor] += direction * self.step_size* scale_speed

                    print(self.setpoint_mp)
                    # print(self.step_size)
                    # print(direction * self.step_size)

                elif 'B' in self.motorIDs[active_motor]:  
                    self.setpoint_mp[active_motor] += direction * self.bldc_scaled_step * scale_speed
            
            # publish setpoint_ms
            self.cr.update_setpoint_ms(self.setpoint_mp)

            self.sleep_rate.sleep() # ensure 100 Hz control rate 

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

    example_name = type(keyboard_command_motor_pos).__name__
    ral = crtk.ral(example_name)
    
    example = keyboard_command_motor_pos(ral, args.CR)
    ral.spin_and_execute(example.run)

if __name__ == '__main__':
    main()

