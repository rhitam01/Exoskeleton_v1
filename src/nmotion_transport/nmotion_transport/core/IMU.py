import os
import sys
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(os.path.join(dir_path, "./lib")))

# import importlib
# py_major_version = sys.version_info[0]
# py_minor_version = sys.version_info[1]
# globals()["nmotion_transport"] = importlib.import_module(f'libnmotion_transport_python_{py_major_version}{py_minor_version}')
# globals()["nmotion_transport"] = importlib.import_module(f'libnmotion_transport_python')

import libnmotion_transport_python as nmotion_transport
from .UsbInterface import USBInterface
from typing import Any

class IMU():
    """! IMU class which wraps around the IMU C++ class
    
    This class provides functionality to interact with IMU devices through the CAN interface.
    """
    __imu: Any = None
    id: int = None
    
    def __init__(self, id: int, interface: USBInterface):
        """!
        IMU class constructor
        
        @param id CAN Node ID of the IMU device
        @param interface Initialised CANInterface or USBInterface object
        """
        self.id = id
        interface = interface
        self.__imu = nmotion_transport.IMU(id, interface.getInterface())
    
    def getRotationVectorData(self) -> 'tuple[float, float, float, float]':
        """!
        Retrieve the rotation vector data from the IMU
        
        @return Tuple containing (quat_i, quat_j, quat_k, quat_real) quaternion components
        """
        return self.__imu.getRotationVectorData()

    def getRawAccelerometerData(self) -> 'tuple[int16_t, int16_t, int16_t]':
        """!
        Retrieve the raw accelerometer data from the IMU
        
        @return Tuple containing raw accelerometer values (x, y, z)
        """
        return self.__imu.getRawAccelerometerData()
    
    def getRawGyroData(self) -> 'tuple[int16_t, int16_t, int16_t]':
        """!
        Retrieve the raw gyroscope data from the IMU
        
        @return Tuple containing raw gyro values (x, y, z)
        """
        return self.__imu.getRawGyroData()
    
    def getRawMagnetometerData(self) -> 'tuple[int16_t, int16_t, int16_t]':
        """!
        Retrieve the raw magnetometer data from the IMU
        
        @return Tuple containing raw magnetometer values (x, y, z)
        """
        return self.__imu.getRawMagnetometerData()

    def getLinearAccelerometerData(self) -> 'tuple[float, float, float, uint8_t]':
        """!
        Retrieve the linear accelerometer data from the IMU
        
        @return Tuple containing linear acceleration values (x, y, z) in m/s² and accuracy
        """
        return self.__imu.getLinearAccelerometerData()

    def getAccelerometerData(self) -> 'tuple[float, float, float, uint8_t]':
        """!
        Retrieve the accelerometer data from the IMU
        
        @return Tuple containing acceleration values (x, y, z) in m/s² and accuracy
        """
        return self.__imu.getAccelerometerData()
    
    def getGyroData(self) -> 'tuple[float, float, float, uint8_t]':
        """!
        Retrieve the gyroscope data from the IMU
        
        @return Tuple containing gyro values (x, y, z) in radians per second and accuracy
        """
        return self.__imu.getGyroData()

    def getMagnetometerData(self) -> 'tuple[float, float, float, uint8_t]':
        """!
        Retrieve the magnetometer data from the IMU
        
        @return Tuple containing magnetometer values (x, y, z) in uTesla and accuracy
        """
        return self.__imu.getMagnetometerData()

    def getUncalibratedGyroData(self) -> 'tuple[float, float, float]':
        """!
        Retrieve the uncalibrated gyroscope data from the IMU
        
        @return Tuple containing uncalibrated gyro values (x, y, z) in radians per second
        """
        return self.__imu.getUncalibratedGyroData()

    def getUncalibratedGyroBiasData(self) -> 'tuple[float, float, float]':
        """!
        Retrieve the uncalibrated gyroscope bias data from the IMU
        getEncoderRawData
        @return Tuple containing uncalibrated gyro bias values (x, y, z) in radians per second
        """
        return self.__imu.getUncalibratedGyroBiasData()

    def getEulerAngles(self) -> 'tuple[float, float, float]':
        """!
        Retrieve the Euler angles from the IMU
        
        @return Tuple containing Euler angles (yaw, pitch, roll) in degrees
        """
        return self.__imu.getEulerAngles()

    def getGravityData(self) -> 'tuple[float, float, float]':
        """!
        Retrieve the gravity data from the IMU
        
        @return Tuple containing gravity values (x, y, z) in g
        """
        return self.__imu.getGravityData()

    def enableRotationVector(self, time_interval: int) -> int:
        """!
        Enable rotation vector data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableRotationVector(time_interval)
    
    def disableRotationVector(self) -> int:
        """!
        Disable rotation vector data
        
        @return Return status
        """
        return self.__imu.disableRotationVector()
    
    def enableRawAccelerometer(self, time_interval: int) -> int:
        """!
        Enable raw accelerometer data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableRawAccelerometer(time_interval)
    
    def disableRawAccelerometer(self) -> int:
        """!
        Disable raw accelerometer data

        @return Return status
        """
        return self.__imu.disableRawAccelerometer()

    def enableRawGyro(self, time_interval: int) -> int:
        """!
        Enable raw gyroscope data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableRawGyro(time_interval)
    
    def disableRawGyro(self) -> int:
        """!
        Disable raw gyroscope data
        
        @return Return status
        """
        return self.__imu.disableRawGyro()

    def enableRawMagnetometer(self, time_interval: int) -> int:
        """!
        Enable raw magnetometer data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableRawMagnetometer(time_interval)
    
    def disableRawMagnetometer(self) -> int:
        """!
        Disable raw magnetometer data
        
        @return Return status
        """
        return self.__imu.disableRawMagnetometer()

    def enableLinearAccelerometer(self, time_interval: int) -> int:
        """!
        Enable linear accelerometer data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableLinearAccelerometer(time_interval)
    
    def disableLinearAccelerometer(self) -> int:
        """!
        Disable linear accelerometer data
        
        @return Return status
        """
        return self.__imu.disableLinearAccelerometer()

    def enableAccelerometer(self, time_interval: int) -> int:
        """!
        Enable accelerometer data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableAccelerometer(time_interval)
    
    def disableAccelerometer(self) -> int:
        """!
        Disable accelerometer data
        
        @return Return status
        """
        return self.__imu.disableAccelerometer()

    def enableGyro(self, time_interval: int) -> int:
        """!
        Enable gyroscope data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableGyro(time_interval)
    
    def disableGyro(self) -> int:
        """!
        Disable gyroscope data
        
        @return Return status
        """
        return self.__imu.disableGyro()

    def enableMagnetometer(self, time_interval: int) -> int:
        """!
        Enable magnetometer data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableMagnetometer(time_interval)
    
    def disableMagnetometer(self) -> int:
        """!
        Disable magnetometer data
        
        @return Return status
        """
        return self.__imu.disableMagnetometer()

    def enableUncalibratedGyro(self, time_interval: int) -> int:
        """!
        Enable uncalibrated gyroscope data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableUncalibratedGyro(time_interval)
    
    def disableUncalibratedGyro(self) -> int:
        """!
        Disable uncalibrated gyroscope data
        
        @return Return status
        """
        return self.__imu.disableUncalibratedGyro()
    
    def enableGravity(self, time_interval: int) -> int:
        """!
        Enable gravity data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableGravity(time_interval)
    
    def disableGravity(self) -> int:
        """!
        Disable gravity data
        
        @return Return status
        """
        return self.__imu.disableGravity()
        
    def setNodeId(self, id:int) -> int:
        """!
        Set the node id for the NIMU

        @param id Device ID of the NIMU

        @return Return Status
        """
        return self.__imu.setNodeId(id)

    def getNodeId(self) -> 'tuple[int, int]':
        """!
        Get current Node ID of the NIMU

        @return Tuple containing return status and the current Node ID.
        """
        return self.__imu.getNodeId()
    

    def saveConfigurations(self) -> int:
        return self.__imu.saveConfigurations()

    def eraseConfigurations(self) -> int:
        return self.__imu.eraseConfigurations()
