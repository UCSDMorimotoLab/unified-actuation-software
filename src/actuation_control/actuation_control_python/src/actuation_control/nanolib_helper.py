# -*- coding: utf-8 -*-

# Author: ML lab continuum-actuation team 
# University of California, San Diego

from nanotec_nanolib import Nanolib

# ============================================================ #

class ScanBusCallback(Nanolib.NlcScanBusCallback): # override super class
    def __init__(self):
        super().__init__()
    def callback(self, info, devicesFound, data):
        if info == Nanolib.BusScanInfo_Start :
            print('Scan started.')
        elif info == Nanolib.BusScanInfo_Progress :
            if (data & 1) == 0 :
                print('.', end='', flush=True)
        elif info == Nanolib.BusScanInfo_Finished :
            print('\nScan finished.')

        return Nanolib.ResultVoid()

callbackScanBus = ScanBusCallback() # Nanolib 2021

# ============================================================ #


class NanolibHelper(object):
    """
    A class used to demonstrate the use of the nanolib.

    Note: we know that python can infer the data type of the function parameter.
    We have written out the data types in the functions explicitly to help
    understanding which data type is necessary easier.

    Attributes
    ----------
    nanolib_accessor : Nanolib.INanoLibAccessor
        A pointer to the nanolib accessor in the nanolib
    """

    def setup(self):
        """Creates and stores the nanolib accessor.

        Note: call this function before calling another function
        """
        self.nanolib_accessor: Nanolib.NanoLibAccessor = Nanolib.getNanoLibAccessor()

    def get_bus_hardware(self):
        """Get a list of available bus hardware.

        Note: only supported bus hardware is taken into account.

        Returns
        -------
        list
            a list of Nanolib.BusHardwareId found
        """
        result = self.nanolib_accessor.listAvailableBusHardware()

        if(result.hasError()):
            raise Exception('Error: listAvailableBusHardware() - ' + result.getError())

        return result.getResult()

    def create_bus_hardware_options(self, bus_hw_id: Nanolib.BusHardwareId):
        """Create bus hardware options object.

        Returns
        ----------
        bus_hardware_option : Nanolib.BusHardwareOptions
             A set of options for opening the bus hardware
        """
     
        bus_hardware_option = Nanolib.BusHardwareOptions()
     
        # now add all options necessary for opening the bus hardware
        if (bus_hw_id.getProtocol() == Nanolib.BUS_HARDWARE_ID_PROTOCOL_CANOPEN):
            # in case of CAN bus it is the baud rate
            bus_hardware_option.addOption(
                Nanolib.CanBus().BAUD_RATE_OPTIONS_NAME,
				Nanolib.CanBaudRate().BAUD_RATE_1000K
            )

            if (bus_hw_id.getBusHardware() == Nanolib.BUS_HARDWARE_ID_IXXAT):
                # in case of HMS IXXAT we need also bus number
                bus_hardware_option.addOption(
                    Nanolib.Ixxat().ADAPTER_BUS_NUMBER_OPTIONS_NAME, 
                    Nanolib.IxxatAdapterBusNumber().BUS_NUMBER_0_DEFAULT
                )
		
        elif (bus_hw_id.getProtocol() == Nanolib.BUS_HARDWARE_ID_PROTOCOL_MODBUS_RTU):
            # in case of Modbus RTU it is the serial baud rate
            bus_hardware_option.addOption(
                Nanolib.Serial().BAUD_RATE_OPTIONS_NAME,
				Nanolib.SerialBaudRate().BAUD_RATE_19200
            )
            # and serial parity
            bus_hardware_option.addOption(
                Nanolib.Serial().PARITY_OPTIONS_NAME,
				Nanolib.SerialParity().EVEN
            )
        else:
            pass           

        return bus_hardware_option

    def open_bus_hardware(self, bus_hw_id: Nanolib.BusHardwareId, bus_hw_options: Nanolib.BusHardwareOptions):
        """Opens the bus hardware with given id and options.

        Parameters
        ----------
        bus_hw_id : Nanolib.BusHardwareId
            The bus hardware Id taken from function NanoLibHelper.get_bus_hardware()
        bus_hw_options : Nanolib.BusHardwareOptions
            The hardware options taken from NanoLibHelper.create_bus_hardware_options()
        """
        result = self.nanolib_accessor.openBusHardwareWithProtocol(bus_hw_id, bus_hw_options)

        if(result.hasError()):
            raise Exception('Error: openBusHardwareWithProtocol() - ' + result.getError())

    def close_bus_hardware(self, bus_hw_id: Nanolib.BusHardwareId):
        """Closes the bus hardware (access no longer possible after that).

        Note: the call of the function is optional because the nanolib will cleanup the
        bus hardware itself on closing.

        Parameters
        ----------
        bus_hw_id : Nanolib.BusHardwareId
            The bus hardware Id taken from function NanoLibHelper.get_bus_hardware()
        """
        result = self.nanolib_accessor.closeBusHardware(bus_hw_id)

        if(result.hasError()):
            raise Exception('Error: closeBusHardware() - ' + result.getError())

    def scan_bus(self, bus_hw_id: Nanolib.BusHardwareId):
        """Scans bus and returns all found device ids.

        CAUTION: open bus hardware first with NanoLibHelper.open_bus_hardware()

        Note: this functionality is not available on all bus hardwares. It is assumed that
        this example runs with CANopen where the scan is possible.

        Parameters
        ----------
        bus_hw_id : Nanolib.BusHardwareId
            The bus hardware to scan

        Returns
        ----------
        list : Nanolib.DeviceId
            List with found devices
        """      
        result = self.nanolib_accessor.scanDevices(bus_hw_id, callbackScanBus)

        if(result.hasError()):
            raise Exception('Error: scanDevices() - ' + result.getError())

        return result.getResult()

    def create_device(self, device_id:  Nanolib.DeviceId):
        """Create a Nanolib device from given device id.

        Parameters
        ----------
        device_id : Nanolib.DeviceId
            The bus device id

        Returns
        ----------
        device_handle : Nanolib.DeviceHandle
        """
        device_handle = self.nanolib_accessor.addDevice(device_id).getResult()
        
        return device_handle

    def connect_device(self, device_handle: Nanolib.DeviceHandle):
        """Connects Device with given device handle.

        Parameters
        ----------
        device_handle : Nanolib.DeviceHandle
            The device handle of the device connect to
        """
        result = self.nanolib_accessor.connectDevice(device_handle)

        if(result.hasError()):
            raise Exception('Error: connectDevice() - ' + result.getError())

    def disconnect_device(self, device_handle: Nanolib.DeviceHandle):
        """Disconnects Device with given device handle.

        Note: the call of the function is optional because the Nanolib will cleanup the
        devices on bus itself on closing.

        Parameters
        ----------
        device_handle : Nanolib.DeviceHandle
            The device handle of the device disconnect from
        """
        result = self.nanolib_accessor.disconnectDevice(device_handle)

        if(result.hasError()):
            raise Exception('Error: disconnectDevice() - ' + result.getError())

    def read_number(self, device_handle: Nanolib.DeviceHandle, od_index: Nanolib.OdIndex):
        """Reads out a number from given device

        Note: the interpretation of the data type is up to the user.

        Parameters
        ----------
        device_handle : Nanolib.DeviceHandle
            The handle of the device to read from
        od_index : Nanolib.OdIndex
            The index and sub-index of the object dictionary to read from

        Returns
        ----------
        int
            The number read from the device
        """
        result = self.nanolib_accessor.readNumber(device_handle, od_index)
        
        if(result.hasError()):
            raise Exception(self.create_error_message('read_number', device_handle, od_index, result.getError()))

        return result.getResult()

    def read_number_od(self, object_dictionary: Nanolib.ObjectDictionary, od_index: Nanolib.OdIndex):
        """Reads out a number from given device via the assigned object dictionary

        Parameters
        ----------
        object_dictionary : Nanolib.ObjectDictionary
            An assigned object dictionary
        od_index : Nanolib.OdIndex
            The index and sub-index of the object dictionary to read from

        Returns
        ----------
        int
            The number read from the device
        """
        result = self.get_object(object_dictionary, od_index).readNumber()
        
        if(result.hasError()):
            raise Exception(self.create_error_message('read_number', object_dictionary.getDeviceHandle(), od_index, result.getError()))

        return result.getResult()

    def write_number(self, device_handle: Nanolib.DeviceHandle, value, od_index: Nanolib.OdIndex, bit_length):
        """Writes given value to the device.

        Parameters
        ----------
        device_handle: Nanolib.DeviceHandle
            The handle of the device to write to
        value : int
            The value to write to the device
        od_index: Nanolib.OdIndex
            The index and sub-index of the object dictionary to write to
        bit_length : int
            The bit length of the object to write to, either 8, 16 or 32
            (see manual for all the bit lengths of all objects)
        """
        result = self.nanolib_accessor.writeNumber(device_handle, value, od_index, bit_length)

        if(result.hasError()):
            raise Exception(self.create_error_message('write_number', device_handle, od_index, result.getError()))   
        
    def write_number_od(self, object_dictionary: Nanolib.ObjectDictionary, value, od_index: Nanolib.OdIndex):
        """Writes given value to the device via assigned object dictionary

        Parameters
        ----------
        object_dictionary: Nanolib.ObjectDictionary
            An assigned object dictionary
        value : int
            The value to write to the device
        od_index: Nanolib.OdIndex
            The index and sub-index of the object dictionary to write to
        """
        result = self.get_object(object_dictionary, od_index).writeNumber(value)

        if(result.hasError()):
            raise Exception(self.create_error_message('write_number', object_dictionary.getDeviceHandle(), od_index, result.getError()))   

    def read_array(self, device_handle: Nanolib.DeviceHandle, od_index: Nanolib.OdIndex):
        """Reads out an od object array.

        Note: the interpretation of the data type is up to the user. Signed integer
        are interpreted as unsigned integer.

        Parameters
        ----------
        device_handle: Nanolib.DeviceHandle
            The handle of the device to read from
        od_index: Nanolib.OdIndex
            The index and sub-index of the object dictionary to read from

        Returns
        ----------
        list : int
            List of ints
        """
        result = self.nanolib_accessor.readNumberArray(device_handle, od_index.getIndex())
        
        if(result.hasError()):
            raise Exception(self.create_error_message('Error: cannot read array', device_handle, od_index, result.getError()))            

        return result.getResult()

    def read_string(self, device_handle: Nanolib.DeviceHandle, od_index: Nanolib.OdIndex):
        """Reads out string from device

        Parameters
        ----------
        device_handle: Nanolib.DeviceHandle
            The handle of the device to read from
        od_index: Nanolib.OdIndex
            The index and sub-index of the object dictionary to read from

        Returns
        ----------
        str
            The read out string
        """
        result = self.nanolib_accessor.readString(device_handle, od_index)

        if(result.hasError()):
            raise Exception(self.create_error_message('Error: cannot read string', device_handle, od_index, result.getError()))            

        return result.getResult()

    def read_string_od(self, object_dictionary: Nanolib.ObjectDictionary, od_index: Nanolib.OdIndex):
        """Reads out string from device

        Parameters
        ----------
        object_dictionary: Nanolib.ObjectDictionary
            An assigned object dictionary
        od_index: Nanolib.OdIndex
            The index and sub-index of the object dictionary to read from

        Returns
        ----------
        str
            The read out string
        """
        result = self.get_object(object_dictionary, od_index).readString()

        if(result.hasError()):
            raise Exception(self.create_error_message('Error: cannot read string', object_dictionary.getDeviceHandle(), od_index, result.getError()))            

        return result.getResult()

    def get_device_object_dictionary(self, device_handle: Nanolib.DeviceHandle):
        """Gets assigned object dictionary
        Parameters
        ----------
        device_handle: Nanolib.DeviceHandle
            The handle of the device
        Returns
        ----------
        Nanolib.ObjectDictionary
            The assigned object dictionary
        """
        result = self.nanolib_accessor.getAssignedObjectDictionary(device_handle)
        if(result.hasError()):
            raise Exception('Unable to get the assigned Object Dictionary - ' + result.getError())

        return result.getResult()


    def get_object_entry(self, object_dictionary: Nanolib.ObjectDictionary, index):
        """Gets object sub entry of given object dictionary
        Parameters
        ----------
        object_dictionary: Nanolib.ObjectDictionary
        index: Int
            The index of the object entry

        Returns
        ----------
        Nanolib.ObjectEntry
            Object dictionary entry
        """
        result = object_dictionary.getObjectEntry(index)

        if(result.hasError()):
            raise Exception('Unable to get Object Dictionary entry - ' + result.getError())

        return result.getResult()

    def get_object(self, object_dictionary: Nanolib.ObjectDictionary, od_index: Nanolib.OdIndex):
        """Gets object sub entry of given object dictionary
        Parameters
        ----------
        object_dictionary: Nanolib.ObjectDictionary
        od_index: Nanolib.OdIndex
            The index and sub-index of the object sub entry

        Returns
        ----------
        Nanolib.ObjectSubEntry
            Object dictionary sub entry
        """
        result = object_dictionary.getObject(od_index)

        if(result.hasError()):
            raise Exception('Unable to get Object Dictionary sub entry - ' + result.getError())

        return result.getResult()

    def set_logging_level(self, log_level):
        """Set the logging level

        Parameters
        ----------
        log_level
            The log level, can be
            - LogLevel_Off
            - LogLevel_Trace
            - LogLevel_Debug
            - LogLevel_Info (default)
            - LogLevel_Warn
            - LogLevel_Error
        """
        if(self.nanolib_accessor is None):
            raise Exception('Error: NanolibHelper().setup() is required')

        self.nanolib_accessor.setLoggingLevel(log_level)

    def get_profinet_dcp_interface(self):
        """
        """
        profinet_dcp_interface = self.nanolib_accessor.getProfinetDCP()
        
        return profinet_dcp_interface

    def create_error_message(self, function_name, device_handle: Nanolib.DeviceHandle, od_index: Nanolib.OdIndex, result_error_text):
        """Helper function for creating an error message from given objects

        Parameters
        ----------
        function : str
            The bus hardware to scan
        device_handle: Nanolib.DeviceHandle
            The handle of the device
        od_index: Nanolib.OdIndex
            The index and sub-index of the object dictionary
        result_error_text
            The error text of the result

        Returns
        ----------
        str
            The error string
        """
        result_device_id = self.nanolib_accessor.getDeviceId(device_handle)
        if result_device_id.hasError():
            device_id_str = "invalid handle"
        else :
            device_id_str = result_device_id.getResult().toString()
        return 'Running function \"{}\" on device {} at od index {} resulted in an error: {}'.format(function_name, device_id_str, od_index.toString(), result_error_text)
