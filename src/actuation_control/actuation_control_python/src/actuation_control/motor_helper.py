# -*- coding: utf-8 -*-

# Author: ML lab continuum-actuation team 
# University of California, San Diego

############ import statement for ROS #########################
from nanotec_nanolib import Nanolib
from actuation_control import binary_operations
import math
import time
from timeit import default_timer as timer

from concurrent.futures import ThreadPoolExecutor, as_completed

class MotorHelper:
    # Class-level dictionary to hold motor instances
    motor_registry = {}
    id_registry = {}
    
    def __init__(self, nanolib_helper, bus_hw_id, 
                 motor_id, device_id, motor_type,  
                 unit, operating_mode, control_mode):
        
        self.nanolib_helper = nanolib_helper
        self.bus_hw_id = bus_hw_id
        self.motor_id = motor_id
        self.device_id = device_id  #TODO - define what each one of these is doing
        self.motor_type = motor_type
        self.unit = unit
        self.operating_mode = operating_mode
        self.control_mode = control_mode


        self.unit_multiplier = 1  # default value
        self.positioning_type = 'relative' # default value

        # self.current_pos_trans = current_pos_trans # keeps track of current motor position, YTX: make this as an input for initialization
        # self.current_pos_rot = current_pos_rot # keeps track of current motor position, YTX: make this as an input for initialization
        self.module_width = 44.00 # Width of a given module [mm] - #TODO - update? Need to reflect limit switch
        self.spacing_size = 10.00 # Initial spacing to be applied between units on homing [mm]

        # Register the motor instance
        MotorHelper.motor_registry[motor_id] = self
        MotorHelper.id_registry[device_id] = self

        self.connect_motor()
        self.setup_motor()
    
    def get_module_width(self):
        return self.module_width

    def connect_motor(self):
        # either scan the whole bus for devices (in case the bus supports scanning)
        device_ids = self.nanolib_helper.scan_bus(self.bus_hw_id)  # scan for device ids
        for id in device_ids:
            # print("Found Device: {}".format(id.toString()))
            id_value = id.getDeviceId()
            if id_value == self.device_id:  # if id_value matches what was chosen
                self.device_handle = self.nanolib_helper.create_device(id)
                # now connect to the device
                self.nanolib_helper.connect_device(self.device_handle)
                self.object_dictionary = self.nanolib_helper.get_device_object_dictionary(self.device_handle)
            elif id == len(device_ids) - 1:  # if end of loop has been reach without a match
                raise ValueError("Could not find device id" + self.device_id)
                

    def disconnect_motor(self):
        # cleanup and close everything
        self.nanolib_helper.disconnect_device(self.device_handle)

    def setup_motor(self): # set units for velocity as well! 
        # Set Motor Unit and Unit Multiplier 
        if self.unit == 'mm':
            unit_pos = 0xFA010000     # 10^-6 m  # YTX @NoahJones: this is not an option from the documentation, do you mean FA. TODO - resolve comment to left
            self.unit_multiplier = 1000 # to convert from user inputted mm to 10^-6 m  
        elif self.unit == 'rad':
            unit_pos = 0xFD100000        # 10^-3 rad
            self.unit_multiplier = 1000  # multiplier to convert user inputted radians
        else:
            raise Exception('Invalid motor unit, must be mm or rad for operation, and rev for testing')
        
        # set SI units for motor position 
        self.nanolib_helper.write_number_od(self.object_dictionary, unit_pos, Nanolib.OdIndex(0x60A8, 0x00))
        
        # Set Motor Control Mode: close/open loop
        if self.control_mode == "closed_loop":
            control = 0b1
        elif self.control_mode == "open_loop":
            # open loop control
            control = 0b0
        # else:
        #     raise Exception('Invalid motor control mode')                                                                                      ion('Either +closed/open loop control is allowed')
        # set control mode
        self.nanolib_helper.write_number_od(self.object_dictionary, control, Nanolib.OdIndex(0x3202, 0x00))

        # Setting Motor Data
        # TODO: might need to be careful about setting the ramp acceleration/decceleration  
        if self.motor_type == 'stepper':
            motor_type = 0b0000000 
            # set motor type 
            self.nanolib_helper.write_number_od(self.object_dictionary, motor_type | control, Nanolib.OdIndex(0x3202, 0x00))
            max_peak_current= 600  # [mA]
            # set max motor peak current  
            self.nanolib_helper.write_number_od(self.object_dictionary, max_peak_current, Nanolib.OdIndex(0x2031, 0x00))
            # set rated current 
            rated_curent = 600 # [mA]
            self.nanolib_helper.write_number_od(self.object_dictionary, rated_curent, Nanolib.OdIndex(0x203B, 0x01))
            # set max current in %
            max_current = 1000
            self.nanolib_helper.write_number_od(self.object_dictionary, max_current, Nanolib.OdIndex(0x6073, 0x00))
            # set feed constant 
            linear_feed = 2*self.unit_multiplier  # 2mm/rev stepper resolution (motor/shaft dependent) * 200 steps/rev (given constant), in 10^-6 m
            self.nanolib_helper.write_number_od(self.object_dictionary, linear_feed, Nanolib.OdIndex(0x6092, 0x01))
            shaft_rev = 1
            self.nanolib_helper.write_number_od(self.object_dictionary, shaft_rev, Nanolib.OdIndex(0x6092, 0x02))
            # set n of pole pairs -> 50 for steppers
            num_pole_pairs = 50
            self.nanolib_helper.write_number_od(self.object_dictionary, num_pole_pairs, Nanolib.OdIndex(0x2030, 0x00))
            # set motor polarity
            polarity_reversed = 0b10000000 # ccw + (direction of insertion), cw - (direction of retraction)
            self.nanolib_helper.write_number_od(self.object_dictionary, polarity_reversed, Nanolib.OdIndex(0x607E, 0x00))
            # set motor velocity 
            velocity = 300 # unit: rev/min, need to check with tubes in
            self.nanolib_helper.write_number_od(self.object_dictionary, velocity, Nanolib.OdIndex(0x6081, 0x00))
            # set accel ramp
            max_accel = 5000 
            self.nanolib_helper.write_number_od(self.object_dictionary, max_accel, Nanolib.OdIndex(0x60C5, 0x00))
            self.nanolib_helper.write_number_od(self.object_dictionary, max_accel, Nanolib.OdIndex(0x6083, 0x00))
            # set deccel ramp
            max_decel = 5000
            self.nanolib_helper.write_number_od(self.object_dictionary, max_decel, Nanolib.OdIndex(0x60C6, 0x00))
            self.nanolib_helper.write_number_od(self.object_dictionary, max_decel, Nanolib.OdIndex(0x6084, 0x00))
            # set position setpoint window time 
            window_time = 100 # [ms] 1
            self.nanolib_helper.write_number_od(self.object_dictionary, window_time, Nanolib.OdIndex(0x6068, 0x00))
            # disregard position window 
            window  = 10 #[1e-3 m] 1000
            self.nanolib_helper.write_number_od(self.object_dictionary, window, Nanolib.OdIndex(0x6067, 0x00))

            # enable internal "measured" values (run autosetup in the GUI first, should be enabled!)
            # enable_pos_fbk = 0b1
            # self.nanolib_helper.write_number_od(self.object_dictionary, enable_pos_fbk, Nanolib.OdIndex(0x3203, 0x01))


        elif self.motor_type == 'bldc':
            motor_type = 0b1000000
            # set motor type
            self.nanolib_helper.write_number_od(self.object_dictionary, motor_type | control, Nanolib.OdIndex(0x3202, 0x00))
            # turn on current reduction 
            enable_curr_reduc = 0b1000
            self.nanolib_helper.write_number_od(self.object_dictionary, motor_type | enable_curr_reduc | control, Nanolib.OdIndex(0x3202, 0x00))

            # ensure feedback selection is set properly 
            self.nanolib_helper.write_number_od(self.object_dictionary, 0, Nanolib.OdIndex(0x3203, 0x01))
            self.nanolib_helper.write_number_od(self.object_dictionary, 0, Nanolib.OdIndex(0x3203, 0x02))
            self.nanolib_helper.write_number_od(self.object_dictionary, 0, Nanolib.OdIndex(0x3203, 0x03))
            self.nanolib_helper.write_number_od(self.object_dictionary, 0, Nanolib.OdIndex(0x3203, 0x04))
            
            # set max permissible bldc currents
            max_permis_motor_current = 456  # [mA]
            self.nanolib_helper.write_number_od(self.object_dictionary, max_permis_motor_current, Nanolib.OdIndex(0x2031, 0x00))
            # set max rated currents
            motor_rated_current = 455 # [mA]
            self.nanolib_helper.write_number_od(self.object_dictionary, motor_rated_current, Nanolib.OdIndex(0x203B, 0x01))
            # set duration of max current 
            max_duration_of_max_current = 100  # [ms]
            self.nanolib_helper.write_number_od(self.object_dictionary, max_duration_of_max_current, Nanolib.OdIndex(0x203B, 0x02))
            # set max current percentage
            max_current = 600  # 10ths of percent (100% = 1000)
            self.nanolib_helper.write_number_od(self.object_dictionary, max_current, Nanolib.OdIndex(0x6073, 0x00))
            ################################################################################################################
            # set idle current behavior (NOTE: very important for dealign with BLDC overheating issues, should set to as low as possible)
            # set max idle current duration
            max_idle_curr_time = 10 # ms
            self.nanolib_helper.write_number_od(self.object_dictionary, max_idle_curr_time, Nanolib.OdIndex(0x2036, 0x00))
            # set max idle current  
            max_idle_curr = 10 # ms
            self.nanolib_helper.write_number_od(self.object_dictionary, max_idle_curr, Nanolib.OdIndex(0x2037, 0x00))
            ###################################################################################################################
            # set gear ratio 
            gear_reduction = 190 # gearhead 19:1 worm 10:1
            self.nanolib_helper.write_number_od(self.object_dictionary, gear_reduction, Nanolib.OdIndex(0x6091, 0x01))
            shaft_rev = 1
            self.nanolib_helper.write_number_od(self.object_dictionary, shaft_rev, Nanolib.OdIndex(0x6091, 0x02))
            # set n of pole pairs 
            num_pole_pairs = 1
            self.nanolib_helper.write_number_od(self.object_dictionary, num_pole_pairs, Nanolib.OdIndex(0x2030, 0x00))
            # set motor polarity
            polarity_reversed = 0b10000000 # ccw tube rotation is positive + cw tube rotation is negative 
            self.nanolib_helper.write_number_od(self.object_dictionary, polarity_reversed, Nanolib.OdIndex(0x607E, 0x00))
            # set motor velocity 
            velocity = 50 # unit: rev/min, need to check with tubes in
            self.nanolib_helper.write_number_od(self.object_dictionary, velocity, Nanolib.OdIndex(0x6081, 0x00))
            # set accel ramp
            max_accel = 100 
            self.nanolib_helper.write_number_od(self.object_dictionary, max_accel, Nanolib.OdIndex(0x60C5, 0x00))
            # set deccel ramp
            max_decel = 100
            self.nanolib_helper.write_number_od(self.object_dictionary, max_decel, Nanolib.OdIndex(0x60C6, 0x00))
            # set position setpoint window time 
            window_time = 1 # [ms]
            self.nanolib_helper.write_number_od(self.object_dictionary, window_time, Nanolib.OdIndex(0x6068, 0x00))
            # disregard position window 
            window  = 1000 #[1e-3 mm]
            self.nanolib_helper.write_number_od(self.object_dictionary, window, Nanolib.OdIndex(0x6067, 0x00))
              
        else:
            raise Exception('Invalid motor type, must be stepper or bldc')

        # Set positioning option
        positioning_option_code = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x60F2, 0x00))
        transition = positioning_option_code | 0b100000  # releases new setpoint as soon as possible for the controller
        self.nanolib_helper.write_number_od(self.object_dictionary, transition, Nanolib.OdIndex(0x60F2, 0x00))
        
        # Set Motor Operating Mode
        if self.operating_mode == 'position':
            # Profile Position mode (pp): point-to-point moves with a
            # new-setpoint / target-reached handshake -> blocking control.
            position = 1  # set operating mode to position
            self.nanolib_helper.write_number_od(self.object_dictionary, position, Nanolib.OdIndex(0x6060, 0x00))

            # Set initial positioning type as relative (need to home motors first)
            control_word = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x6040, 0x00))
            relative_mode = 0b1000000
            self.nanolib_helper.write_number_od(self.object_dictionary, control_word|relative_mode, Nanolib.OdIndex(0x6040, 0x00))
            # absolute_mode = 0b0000000
            # self.nanolib_helper.write_number_od(self.object_dictionary, control_word|absolute_mode, Nanolib.OdIndex(0x6040, 0x00))

        elif self.operating_mode == 'csp':
            # Cyclic Synchronous Position mode (mode 8): the drive continuously
            # follows the ABSOLUTE target written to 0x607A, no per-move
            # handshake -> non-blocking control. The caller is expected to
            # stream 0x607A every control cycle (see set_target_position_no_wait).
            csp = 8
            self.nanolib_helper.write_number_od(self.object_dictionary, csp, Nanolib.OdIndex(0x6060, 0x00))
            # interpolation time period 0x60C2 = value * 10^exponent seconds.
            # Should roughly match the controller's servo rate (20 ms -> 50 Hz).
            # NOTE: CSP normally expects a cyclic PDO / sync signal; when it is
            # fed via SDO writes only, this period and the following-error
            # window (0x6065) may need tuning, or use interpolated position
            # mode (7) instead if the drive will not engage CSP.
            self.nanolib_helper.write_number_od(self.object_dictionary, 20, Nanolib.OdIndex(0x60C2, 0x01))
            self.nanolib_helper.write_number_od(self.object_dictionary, -3, Nanolib.OdIndex(0x60C2, 0x02))

        else:
            raise Exception('invalid operating mode! (expected "position" or "csp")')

        # Set new setpoint to be immediately executed (to minimize control delay)
        control_word = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x6040, 0x00))
        move_immediately = 0b0100000
        # block_move = 0b0000000
        # control_word = control_word & ~(1 << 5)
        # self.nanolib_helper.write_number_od(self.object_dictionary, control_word|move_immediately, Nanolib.OdIndex(0x6040, 0x00)) 
        # self.nanolib_helper.write_number_od(self.object_dictionary, control_word, Nanolib.OdIndex(0x6040, 0x00)) 

        # Enable motor after setup
        self.reset_motor()
        if self.operating_mode == 'csp':
            # CSP follows 0x607A continuously -- seed it with the current actual
            # position so the drive holds still (does not jump) on enable
            actual_pos = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x6064, 0x00))
            self.nanolib_helper.write_number_od(self.object_dictionary, actual_pos, Nanolib.OdIndex(0x607A, 0x00))
        else:
            # need to clear any previously stored (relative) target value
            self.nanolib_helper.write_number_od(self.object_dictionary, 0, Nanolib.OdIndex(0x607A, 0x00))
        self.enable_motor()


        print("motors set up")
    
    def enable_motor(self):
        # enable motor based on state machine logic from manual, only concerns first 4 bits of control word
        # transitions from switched on disabled -> operation enabled
        control_word = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x6040, 0x00)) 
        current_state = control_word 
        current_state = current_state|0b0110  # ready to switch on
        self.nanolib_helper.write_number_od(self.object_dictionary, current_state, Nanolib.OdIndex(0x6040, 0x00))
        current_state = current_state|0b01  # switched on
        self.nanolib_helper.write_number_od(self.object_dictionary, current_state, Nanolib.OdIndex(0x6040, 0x00))
        current_state = current_state|0b01000  # operation enabled
        self.nanolib_helper.write_number_od(self.object_dictionary, current_state, Nanolib.OdIndex(0x6040, 0x00))
    
    def reset_motor(self):
        # disable motor based on state machine logic from manual, only concerns first 4 bits of control word
        # transitions from operation enabled -> switched on disabled
        disable_voltage = ~(0b1111) # only set first 4 bits to 0
        control_word = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x6040, 0x00)) 
        self.nanolib_helper.write_number_od(self.object_dictionary, control_word & disable_voltage, Nanolib.OdIndex(0x6040, 0x00)) # quick stop

    def get_motor_pos(self):
        raw_measured_pos = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x6064, 0x00))
        # print(uint32_to_int32(raw_measured_pos))
        # convert to ROS topic units 
        # first do pos/-neg value conversion fromo uint32 -> int32 (use helper function)
        # if steppers (do 1e-3mm -> mm conversion), if BLDCs (do 1e-3 rad -> rad)
        # must return of type list of floats (adhering to ROS message JointState message type)
        measured_pos = 0.001*binary_operations.uint32_to_int32(raw_measured_pos)
        return measured_pos

    def get_motor_current(self):
        raw_measured_current = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x2039, 0x05))
        # convert to ROS topic units 
        # first do pos/-neg value conversion fromo uint32 -> int32 (use helper function)
        # measured in mA
        # must return of type list of floats (adhering to ROS message JointState message type)
        measured_current = 1.00*binary_operations.uint32_to_int32(raw_measured_current)
        return measured_current 
    
    def move_motor_relative_and_wait(self,target_value): 
        # Move motor relative to current position. Wait for motor to reach target before commanding next.

        # Define Target Position/Angle
        target_value = math.floor(self.unit_multiplier*target_value)  # convert radians to correct exponent and round to integer below
        self.nanolib_helper.write_number_od(self.object_dictionary, target_value, Nanolib.OdIndex(0x607A, 0x00))

        # Setup travel command, -> 0
        control_word = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x6040, 0x00))
        # make sure bit 4 is set to zero
        bit4_mask = ~(1<<4)
        transition = control_word & bit4_mask
        self.nanolib_helper.write_number_od(self.object_dictionary, transition, Nanolib.OdIndex(0x6040, 0x00))

        # Check if Buffer is Ready
        status_word = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x6041, 0x00))
        buffer_not_ready = bin(status_word & int(0b1000000000000))
        if buffer_not_ready == '0b1000000000000':
            print("buffer not ready")
        # else:
            # print("buffer ready")

        # Trigger Travel Command by a rising edge 0 -> 1
        control_word = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x6040, 0x00))
        transition = control_word | 0b0000000000010000
        status_word = self.nanolib_helper.write_number_od(self.object_dictionary, transition, Nanolib.OdIndex(0x6040, 0x00))
        # print('Sending motor: ' + str(self.motor_id) + ' to position... ')

        # Check if Position Reached (this will make sure the measured motor position value is accurate!)
        reached_setpoint = 0
        start_time = time.time()
        while reached_setpoint == 0:
            status_word = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x6041, 0x00))
            reached_setpoint = binary_operations.check_bit(status_word,10)
            # print("setpoint not reached")
        end_time = time.time()
        # print("setpoint reached")
        # print(f" {end_time - start_time:.6f}s")
            
        # Check for motor fault
        error = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x1003, 0x01))
        if error != 0:
            error = error >> 24 # only takes the uppper 8 bits (which corresponds to the error number)
            print('Error number: ' + str(error))
            raise ValueError("Motor Stalled Due to Error")
            # some common error codes:
                # 3 Input voltage (+Ub) too low

    def set_target_position_no_wait(self, target_value):
        # CSP mode: write an ABSOLUTE target to 0x607A and return immediately.
        # The drive interpolates toward it on its own each cycle -- no
        # new-setpoint/target-reached handshake, no blocking. Fault checking is
        # left to the caller so this stays a single cheap write.
        target_value = math.floor(self.unit_multiplier * target_value)
        self.nanolib_helper.write_number_od(self.object_dictionary, target_value, Nanolib.OdIndex(0x607A, 0x00))

    def get_motor_fault(self):
        # 0 if no fault, else the Nanotec error number (upper 8 bits of 0x1003:01)
        error = self.nanolib_helper.read_number_od(self.object_dictionary, Nanolib.OdIndex(0x1003, 0x01))
        return (error >> 24) if error != 0 else 0

    @classmethod # serial motor calls, expects delays ~0.02s (x5 ~0.1s will be the total lag for the CTR system)
    def move_all_motors_relative_wait_between(cls, motor_ids,target_positions):
        # Move motor relative to current position. Wait for individual motor to reach target before commanding next.
        # This assumes that the delta commands given are not too large 
        # (e.g., for antagonistic control, need to pay out while pulling)

        # measured_pos_all = []
        # timing_log = {}
        # Iterate over the list of motor IDs and send the command
        for index, motor_id in enumerate(motor_ids):
            if motor_id in cls.motor_registry:
                # start_time = time.time()
                # print("motor_id", motor_id)
                cls.motor_registry[motor_id].move_motor_relative_and_wait(target_value=target_positions[index])
                # print("-")
                # end_time = time.time()
                # timing_log[motor_id] = (index,start_time, end_time)
                # measured_pos_all.append(curr_measured_pos)
            else:
                print(f"Motor ID {motor_id} not found.")

    @classmethod
    def set_all_target_positions_no_wait(cls, motor_ids, target_positions):
        # CSP mode: stream one ABSOLUTE target per motor, non-blocking.
        for index, motor_id in enumerate(motor_ids):
            if motor_id in cls.motor_registry:
                cls.motor_registry[motor_id].set_target_position_no_wait(target_positions[index])
            else:
                print(f"Motor ID {motor_id} not found.")

    @classmethod
    def get_motors_pos(cls,motor_ids):
        measured_pos_all = []
        for index, motor_id in enumerate(motor_ids):
            if motor_id in cls.motor_registry:
                # start_time = time.time()
                curr_measured_pos = cls.motor_registry[motor_id].get_motor_pos()
                # end_time = time.time()
                # timing_log[motor_id] = (index,start_time, end_time)
                curr_measured_pos = -curr_measured_pos
                measured_pos_all.append(curr_measured_pos)  # TODO - determine consistent direction for positive and neg in commands
            else:
                print(f"Motor ID {motor_id} not found.")

        return measured_pos_all

    @classmethod
    def get_motors_pos_raw(cls, motor_ids):
        # Raw actual position per motor (0x6064) WITHOUT the sign flip that
        # get_motors_pos() applies for the ROS message. Same frame/sign as the
        # 0x607A target, so use this to seed a CSP absolute setpoint.
        measured_pos_all = []
        for motor_id in motor_ids:
            if motor_id in cls.motor_registry:
                measured_pos_all.append(cls.motor_registry[motor_id].get_motor_pos())
            else:
                print(f"Motor ID {motor_id} not found.")
        return measured_pos_all
    
    @classmethod 
    def get_motors_current(cls,motor_ids):
        measured_current_all = []
        for index, motor_id in enumerate(motor_ids):
            if motor_id in cls.motor_registry:
                # start_time = time.time()
                curr_measured_curr = cls.motor_registry[motor_id].get_motor_current()
                # end_time = time.time()
                # timing_log[motor_id] = (index,start_time, end_time)
                measured_current_all.append(curr_measured_curr)  # TODO - determine consistent direction for positive and neg in commands
            else:
                print(f"Motor ID {motor_id} not found.")

        return measured_current_all