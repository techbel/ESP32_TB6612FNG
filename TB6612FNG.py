#
#      Copyright (C) 2024 techbel
#
#      This program is free software: you can redistribute it and/or modify
#      it under the terms of the GNU General Public License as published by
#      the Free Software Foundation, either version 3 of the License, or
#      (at your option) any later version.
#
#      This program is distributed in the hope that it will be useful,
#      but WITHOUT ANY WARRANTY; without even the implied warranty of
#      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#      GNU General Public License for more details.
#
#      You should have received a copy of the GNU General Public License
#      along with this program.  If not, see <http://www.gnu.org/licenses/>.
#


from machine import I2C, Pin
import time


class TB6612FNGCodes:
    """
    Operational codes used to send commands to the driver module via I2C interface.
    """

    # module address
    GMD_I2C_ADDRESS = 0x14

    # commands for modules
    GMD_CMD_BRAKE = 0x00
    GMD_CMD_STOP = 0x01
    GMD_CMD_CW = 0x02
    GMD_CMD_CCW = 0x03
    GMD_CMD_STANDBY = 0x04
    GMD_CMD_NOT_STANDBY = 0x05
    GMD_CMD_STEPPER_RUN = 0x06
    GMD_CMD_STEPPER_STOP = 0x07
    GMD_CMD_STEPPER_KEEP_RUN = 0x08
    GMD_CMD_SET_ADDR = 0x11
    

class TB6612FNGMotors:
    """
    The motor selection contains two values: MOTOR_CHA and MOTOR_CHB,
    used to choose which motor to operate via the motor driver.
    """

    MOTOR_CHA = 0
    MOTOR_CHB = 1



class TB6612FNGStepper:
    """
    An enum for stepper motor commands enum.
    """

    FULL_STEP = 0
    WAVE_DRIVE = 1
    HALF_STEP = 2
    MICRO_STEPPING = 3


# main class implementation
class MotorDriverTB6612FNG:
    """
    A control class for the Grove - Motor Driver(TB6612FNG).
    """
    _addr = None
    _buffer = None
    _i2c_bus = None

    def __init__(self, address=TB6612FNGCodes.GMD_I2C_ADDRESS):
        """
        Initializes the Motor Driver module with a specific or default address.
        Creates a connection to the I2C bus on ESP32.
        By default the address for this module is 0x14.
        :rtype: MotorDriverTB6612FNG class instance.
        """
        self._i2c_bus = I2C(0, scl=Pin(22), sda=Pin(21), freq=100000)
        self._addr = address
        self.standby()

    def standby(self):
        """
        Enter standby mode. Normally you don't need to call this, except that you have called notStandby() before.
        :return: nothing.
        """
        self._i2c_bus.writeto(self._addr, bytes(TB6612FNGCodes.GMD_CMD_STANDBY), 0)

    def not_standby(self):
        """
        Exit standby mode. Motor driver does't do any action at this mode.
        :return: nothing.
        """
        self._i2c_bus.writeto(self._addr, bytes(TB6612FNGCodes.GMD_CMD_NOT_STANDBY), 0)

    def set_i2c_addr(self, addr: int):
        """
        Set new address for this module.
        :param addr: The new address for motor driver. (0x01~0x7f)
        :return: nothing.
        """
        if addr == 0x00:
            return
        elif addr >= 0x80:
            return
        self._i2c_bus.writeto(self._addr, bytes(TB6612FNGCodes.GMD_CMD_SET_ADDR))
        self._addr = addr

    def dc_motor_run(self, chl: int, speed: int):
        """
        Output a specific amount of voltage to a specific motor. Voltage is between -255 and 255.
        The negative voltage means motor will go counter clockwise.
        :param chl: Selection which motor to run. MOTOR_CHA or MOTOR_CHB.
        :param speed: Speed from -255 to 255 to run this motor. (8bit voltage). Note that there is always a starting
                speed(a starting voltage) for motor. If the input voltage is 5V, the starting speed should larger
                than 100 or smaller than -100.
        :return: nothing.
        """
        self._buffer = bytearray(3)

        if speed > 255:
            speed = 255
        elif speed < -255:
            speed = -255

        if speed >= 0:
            self._buffer[0] = TB6612FNGCodes.GMD_CMD_CW
            self._buffer[2] = speed
        else:
            self._buffer[0] = TB6612FNGCodes.GMD_CMD_CCW
            self._buffer[2] = -speed

        self._buffer[1] = int(chl)

        self._i2c_bus.writeto(self._addr, self._buffer)

    def dc_motor_break(self, chl: int):
        """
        Brake, stop the motor immediately.
        :param chl: MOTOR_CHA or MOTOR_CHB.
        :return: nothing.
        """
        self._i2c_bus.writeto(self._addr, bytes([TB6612FNGCodes.GMD_CMD_BRAKE, chl]))
        
    def dc_motor_stop(self, chl: int):
        """
        Stop the motor slowly.
        :param chl: MOTOR_CHA or MOTOR_CHB.
        :return: nothing.
        """
        self._i2c_bus.writeto(self._addr, bytes([TB6612FNGCodes.GMD_CMD_STOP, chl]))

    def stepper_run(self, mode: int, steps: int, rpm: int):
        """
        Drive a stepper motor.
        :param mode:    4 driver mode: FULL_STEP,WAVE_DRIVE, HALF_STEP, MICRO_STEPPING,
                        for more information: https://en.wikipedia.org/wiki/Stepper_motor#/media/File:Drive.png
        :param steps:   The number of steps to run, range from -32768 to 32767.
                        When steps = 0, the stepper stops.
                        When steps > 0, the stepper runs clockwise. When steps < 0, the stepper runs anticlockwise.
        :param rpm:     Revolutions per minute, the speed of a stepper, range from 1 to 300.
                        Note that high rpm will lead to step lose, so rpm should not be larger than 150.
        :return:    nothing.
        """
        cw = 0
        self._buffer = bytearray(7)

        if steps > 0:
            cw = 1
        elif steps == 0:
            self.stepper_stop()
        elif steps == -32768:
            steps = 32767
        else:
            steps = -steps

        if rpm < 1:
            rpm = 1
        elif rpm > 300:
            rpm = 300

        ms_per_step = int(3000.0 / rpm)
  
        self._buffer[0] = TB6612FNGCodes.GMD_CMD_STEPPER_RUN
        self._buffer[1] = mode
        self._buffer[2] = cw
        self._buffer[3] = steps & 0xFF
        self._buffer[4] = (steps >> 8) & 0xFF
        self._buffer[5] = ms_per_step & 0xFF
        self._buffer[6] = (ms_per_step >> 8) & 0xFF

        self._i2c_bus.writeto(self._addr, self._buffer)

    def stepper_stop(self):
        """
        Stop a stepper motor.
        :return: nothing.
        """
        self._i2c_bus.writeto(self._addr, bytes(TB6612FNGCodes.GMD_CMD_STEPPER_STOP))

    def stepper_keep_run(self, mode: int, rpm: int, is_cw: int):
        """
        Keep a stepper motor running. Keeps moving(direction same as the last move, default to clockwise).
        :param mode: 4 driver mode: FULL_STEP,WAVE_DRIVE, HALF_STEP, MICRO_STEPPING,
                for more information: https://en.wikipedia.org/wiki/Stepper_motor#/media/File:Drive.png
        :param rpm: Revolutions per minute, the speed of a stepper, range from 1 to 300.
                Note that high rpm will lead to step lose, so rpm should not be larger than 150.
        :param is_cw: Set the running direction, true for clockwise and false for anti-clockwise.
        :return: nothing.
        """
        cw = 5 if is_cw else 4
        self._buffer = bytearray(5)

        if rpm < 1:
            rpm = 1
        elif rpm > 300:
            rpm = 300

        ms_per_step = int(3000.0 / rpm)

        self._buffer[0] = TB6612FNGCodes.GMD_CMD_STEPPER_KEEP_RUN
        self._buffer[1] = mode
        self._buffer[2] = cw
        self._buffer[3] = ms_per_step & 0xFF
        self._buffer[4] = (ms_per_step >> 8) & 0xFF

        self._i2c_bus.write_i2c_block_data(self._addr, self._buffer)

