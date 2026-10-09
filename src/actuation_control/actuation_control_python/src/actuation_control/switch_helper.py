# Author: ML lab continuum-actuation team 
# University of California, San Diego

from nanotec_nanolib import Nanolib
from actuation_control import nanolib_helper
 
import numpy as np
import math
import time
from timeit import default_timer as timer

class SwitchHelper:
    def __init__(self, switch_id, motor_helper_class, device_id):
        self.motor_helper_class = motor_helper_class
        self.switch_id = switch_id
        self.device_id = device_id

        self.connect_switch()
        self.setup_switch()

    def connect_switch(self):
        # connect switch to device id
        if self.device_id in self.motor_helper_class.id_registry:
            cls = self.motor_helper_class.id_registry[self.device_id]
            self.object_dictionary = cls.get_controller_object()['object_dictionary']
            self.nanolib_helper = cls.get_controller_object()['nanolib_helper']
        else:
            print(f"Device ID {self.device_id} not found. Limit switch objects can only be created for pre-existing motors.")

    def setup_switch(self):
        # Turn Switch On
        pin_type = 0  # digital input
        usage_of_pins = self.nanolib_helper.write_number_od(self.object_dictionary, pin_type, Nanolib.OdIndex(0x3272, 0x0E))  # assign pin type to pin 39
        usage_of_pins = self.nanolib_helper.write_number_od(self.object_dictionary, pin_type, Nanolib.OdIndex(0x3272, 0x0F))  # assign pin type to pin 41
        #save = self.nanolib_helper.write_number_od(self.object_dictionary, 0x65766173, Nanolib.OdIndex(0x1010, 0x03))

        digital_inputs_control = self.nanolib_helper.write_number_od(self.object_dictionary, 1, Nanolib.OdIndex(0x3240, 0x08)) # turn on input routing
        digital_input_routing = self.nanolib_helper.write_number_od(self.object_dictionary, 15, Nanolib.OdIndex(0x3242, 0x01)) # route pin 41 (routing number 15) to bit 1 of 60FD positive limit switch
        digital_input_routing = self.nanolib_helper.write_number_od(self.object_dictionary, 14, Nanolib.OdIndex(0x3242, 0x02)) # route pin 39 (routing number 14) to bit 0 of 60FD negative limit switch
        limit_switch_error_option_code = self.nanolib_helper.write_number_od(self.object_dictionary, 6, Nanolib.OdIndex(0x3701, 0x00)) # quick stop when limit switch is activated
        save = self.nanolib_helper.write_number_od(self.object_dictionary, 0x65766173, Nanolib.OdIndex(0x1010, 0x03))
       
        



      
        
    
    


