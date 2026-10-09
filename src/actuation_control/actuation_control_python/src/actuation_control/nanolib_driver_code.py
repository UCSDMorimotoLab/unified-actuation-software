#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Author: ML lab continuum-actuation team 
# University of California, San Diego


############ import statement for ROS #########################
from actuation_control import nanolib_helper
from actuation_control import motor_helper
from actuation_control import switch_helper

import time

#################################################################################################
# TODO YTX @Miheer: can you make the translation unit into m instead of mm?
# TODO @YTX and @AM: this is where we can put all the robot dependent actuation code here! (like tendon compensation for instance)
# TODO @YTX: need to robustify the homing function! 


#################################################################################################

class NanotecDriver():
# this can be the main class that calls the motor and limit switch helpers 

    def __init__(self,motorIDs,blocking=True):
        self.motorIDs = motorIDs
        # blocking=True  -> Profile Position mode + move/wait handshake
        # blocking=False -> Cyclic Synchronous Position (CSP) mode, streamed
        self.blocking = blocking
        # setup nanotec controller
        self.setup_controller()
        self.setup_motor()
        self.bus_hw_id = ''

    def setup_controller(self):
        ##################################################
        # usage: 
        # inputs:
        # outputs: 
        ##################################################

        # setup nanolib 
        self.nanolib_helper = nanolib_helper.NanolibHelper()
        #  create access to the nanolib
        self.nanolib_helper.setup()
        # its possible to set the logging level to a different level
        self.nanolib_helper.set_logging_level(nanolib_helper.Nanolib.LogLevel_Off)
        # list all hardware available, decide for the first one
        bus_hardware_ids = self.nanolib_helper.get_bus_hardware()
        if bus_hardware_ids.empty():
            raise Exception('No bus hardware found.')
        
        connected = False
        print('\nWaiting for EtherCat...\n')  # connect to wired EtherCAT by default
        print('\nSee other connection options y/n\n')
        while not connected:
            start_time = time.time()
            line_num = 0
            for line_num, bus_hardware_id in enumerate(bus_hardware_ids):
                if bus_hardware_id.getProtocol() == 'EtherCAT' and bus_hardware_id.getName()[-20:] == "(Ethernet interface)":
                    # Use the selected bus hardware
                    bus_hw_id = bus_hardware_ids[line_num]
                    connected = True
            wait_time = time.time() - start_time
            if wait_time > 5:
                print('\nAvailable bus hardware:\n')
                # just for better overview: print out available hardware
                for line_num, bus_hardware_id in enumerate(bus_hardware_ids):
                    print('{}. {} with protocol: {}'.format(line_num, bus_hardware_id.getName(), bus_hardware_id.getProtocol()))
                print('\nPlease select (type) bus hardware number and press [ENTER]:', end ='')
                line_num = int(input())
                print('')
                if ((line_num < 0) or (line_num >= bus_hardware_ids.size())):
                    raise Exception('Invalid selection!')
                # Use the selected bus hardware
                bus_hw_id = bus_hardware_ids[line_num]
                connected = True     
                    
        # create bus hardware options for opening the hardware
        bus_hw_options = self.nanolib_helper.create_bus_hardware_options(bus_hw_id)
        # now able to open the hardware itself
        self.nanolib_helper.open_bus_hardware(bus_hw_id, bus_hw_options)

        self.bus_hw_id = bus_hw_id

        print("The Nanotec Motorcontrollers Have Been Connected via EtherCAT Protocol.")

    def setup_motor(self):
        # TODO - what is difference between setup_motor here and setup_motor in motor_helper.py -- should these have different names? 
        # TODO YTX @Miheer: suggested modifications to the MotorHelper Class
        # - reduce the number of inputs to MotorHelper Class construction, for instance, 
        # if we know the motor is S4, do we need to also specify that it is a stepper, and 
        # set the units to mm? 
        # - could also be good to change the naming convention from 'S4' to something like 'CTR1R1'
        # for the rotation of tube 1 on CTR1, 'CTR1T1', transmission of tube 1 on CTR1 etc. Not super high priority though
        # maybe something to do once we have two systems up and running 

        # Note to self: this function will have to be modified given modifications to the 
        # MotorHelper Class 
        operating_mode = 'position' if self.blocking else 'csp'

        for motorID in self.motorIDs:
            # example motorID: 'S1' , 'B2'
            motor_type = self.get_motor_type(motorID)
            device_id = self.get_device_id(motorID)
            print('motor_type', motor_type, 'device_id', device_id, 'operating_mode', operating_mode)
            if motor_type == 'stepper':
                # # TODO @YTX, MP: think of a more elegant way to dealing with this issue 
                # current_pos_trans = input("input stepper %s: -beta transmission " % (motorID))
                # current_pos_rot = input("input stepper %s: -alpha rotation " % (motorID))
                print(self.nanolib_helper)
                # raise AssertionError
                motor_helper.MotorHelper(self.nanolib_helper,self.bus_hw_id,
                            motorID, device_id,
                            'stepper','mm',
                            operating_mode,
                            'open_loop')
            if motor_type == 'bldc':
                # current_pos_trans = input("input stepper %s: -beta transmission " % (motorID))
                # current_pos_rot = input("input stepper %s: -alpha rotation " % (motorID))
                motor_helper.MotorHelper(self.nanolib_helper,self.bus_hw_id,
                            motorID, device_id,
                            'bldc','rad',
                            operating_mode,
                            'closed_loop') #YTX: change to closed_loop
                
        #switch_A = SwitchHelper('SwitchA', MotorHelper, 3)
        #switch_B = SwitchHelper('SwitchB', MotorHelper, 5)
        #switch_C = SwitchHelper('SwitchC', MotorHelper, 7)

        # print(MotorHelper.motor_registry)
    

    def get_motor_type(self,motorID):
        if 'S' in motorID:
            motor_type = 'stepper'
        elif 'B' in motorID:
            motor_type = 'bldc'
        else:
            raise Exception('Invalid motorID')
        return motor_type 

    def get_device_id(self,motorID):
        motor_device_pair = {'S1': 1, 'S2': 2,
                             'S3': 3, 'S4': 4,
                             'S5': 5, 'S6': 6,
                             'S7': 7, 'B1': 8,
                             'B2': 9, 'B3': 10,
                             'B4': 11, 'B5': 12}
        device_id = motor_device_pair.get(motorID)
        return device_id
    
    def disconnect_controller(self):
        motor_helper.MotorHelper.disconnect_motor

    def home_motors(self):
        motor_helper.MotorHelper.home_motors(self.motorIDs)
        time.sleep(5)

    def drive_all_motors_relative_wait_between(self,curr_setpoint_jp):
        # print(curr_setpoint_jp)
        motor_helper.MotorHelper.move_all_motors_relative_wait_between(self.motorIDs, curr_setpoint_jp)

    def drive_all_motors_absolute_no_wait(self, abs_targets):
        # CSP mode: stream absolute joint targets to 0x607A, non-blocking
        motor_helper.MotorHelper.set_all_target_positions_no_wait(self.motorIDs, abs_targets)

    def measure_motor_abs_pos(self):
        measured_motor_abs_pos = motor_helper.MotorHelper.get_motors_pos(self.motorIDs)
        return measured_motor_abs_pos

    def measure_motor_abs_pos_raw(self):
        # same as measure_motor_abs_pos() but without the sign flip -- matches the
        # frame of drive_all_motors_absolute_no_wait(), use it to seed CSP
        return motor_helper.MotorHelper.get_motors_pos_raw(self.motorIDs)

    def measure_motor_current(self):
        measured_motor_current = motor_helper.MotorHelper.get_motors_current(self.motorIDs)
        return measured_motor_current 