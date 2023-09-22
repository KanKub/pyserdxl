from __future__ import annotations
from enum import Enum, IntEnum
import dynamixel_sdk as dxlsdk
import time
import numpy as np

__author__ = "Kan Keawhanam"
__email__ = "kan.kea57@gmail.com"
__version__ = "0.2.1"


class SerDxl:
    """Easy Dynamixel Interface for Python."""

    movement_wait_tick = 0.010  # sec
    _pulse_per_rev: int
    _DEG_TO_PULSE: float
    _RAD_TO_PULSE: float

    def __init__(
        self,
        port: str,
        bandrate: int = 57600,
        protocol_ver: float = 2.0,
        motor_id: int = 1,
        series: Enum = None,
    ) -> None:
        self.port_handler = dxlsdk.PortHandler(port)
        self.motor_id = motor_id
        self._protocol_version = protocol_ver
        self.packet_handler = dxlsdk.PacketHandler(self._protocol_version)
        self._bandrate = bandrate
        self.port_handler.setBaudRate(bandrate)
        self._reg_table = DXLTablePH
        if series == None:
            self.set_series(DXLTablePH)
        self.set_series(series)
        assert self.port_handler.openPort(), "Failed to open the port."

    def set_series(self, reg_table: Enum) -> None:
        self._reg_table = reg_table
        self.set_pulse_per_rev(reg_table.PULSE_PER_REV.value)

    def set_pulse_per_rev(self, val: int) -> None:
        self._pulse_per_rev = val
        self._DEG_TO_PULSE = val / 360.0
        self._RAD_TO_PULSE = val / np.pi / 2

    def set_operating_mode(self, mode: DXLOpsMode) -> None:
        self.write_reg(self._reg_table.OPERATING_MODE, int(mode))

    def get_operating_mode(self) -> None:
        return self.read_reg0(self._reg_table.OPERATING_MODE)

    def get_acceleration_prof(self) -> int:
        self.read_reg(self._reg_table.PROFILE_ACCELERATION)

    def get_velocity_prof(self) -> int:
        self.read_reg(self._reg_table.PROFILE_VELOCITY)

    def set_acceleration_prof(self, val: int) -> None:
        self.write_reg(self._reg_table.PROFILE_ACCELERATION, int(val))

    def set_velocity_prof(self, val: int) -> None:
        self.write_reg(self._reg_table.PROFILE_VELOCITY, int(val))

    def set_torque_enabled(self, enable: bool) -> None:
        self.write_reg(self._reg_table.TORQUE_ENABLE, int(enable))

    def get_torque_enabled(self, enable: bool) -> None:
        return self.read_reg0(self._reg_table.TORQUE_ENABLE)

    def get_position(self) -> int:
        # inline code for little performance imporvement.
        rxpackage, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.PRESENT_POSITION.value[0]
        )
        self._dxl_handle_error(result, error)
        return SerDxl.to_signed32(rxpackage)

    def get_goal_position(self) -> int:
        # inline code for little performance imporvement.
        rxpackage, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.GOAL_POSITION.value[0]
        )
        self._dxl_handle_error(result, error)
        return SerDxl.to_signed32(rxpackage)

    def get_current(self) -> int:
        # inline code for little performance imporvement.
        rxpackage, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.PRESENT_CURRENT.value[0]
        )
        self._dxl_handle_error(result, error)
        return SerDxl.to_signed16(rxpackage)
    
    def get_velocity(self) -> int:
        # inline code for little performance imporvement.
        rxpackage, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.PRESENT_VELOCITY.value[0]
        )
        self._dxl_handle_error(result, error)
        return SerDxl.to_signed32(rxpackage)
    
    def get_pwm(self) -> int:
        # inline code for little performance imporvement.
        rxpackage, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.PRESENT_PWM.value[0]
        )
        self._dxl_handle_error(result, error)
        return SerDxl.to_signed16(rxpackage)


    def get_goal_current(self) -> int:
        # inline code for little performance imporvement.
        rxpackage, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.GOAL_CURRENT.value[0]
        )
        self._dxl_handle_error(result, error)
        return SerDxl.to_signed16(rxpackage)
    
    def get_goal_pwm(self) -> int:
        # inline code for little performance imporvement.
        rxpackage, result, error = self.packet_handler.read2ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.GOAL_PWM.value[0]
        )
        self._dxl_handle_error(result, error)
        return rxpackage

    def set_goal_position(self, val: int) -> None:
        # inline code for little performance imporvement.
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.GOAL_POSITION.value[0], val
        )
        self._dxl_handle_error(result, error)

    def set_goal_current(self, val: int) -> int:
        # inline code for little performance imporvement.
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.GOAL_CURRENT.value[0], val
        )
        self._dxl_handle_error(result, error)

    def set_goal_pwm(self, val: int) -> int:
        # inline code for little performance imporvement.
        result, error = self.packet_handler.write2ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.GOAL_PWM.value[0], val
        )
        self._dxl_handle_error(result, error)

    def set_band_rate(self, val: DXLBandRate) -> None:
        rate = val.value
        self.write_reg(self._reg_table.BAND_RATE, rate[0])
        self.port_handler.setBaudRate(rate[1])

    def set_led(self, enable: bool) -> None:
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.motor_id,
            self._reg_table.LED.value[0],
            int(enable),
        )
        self._dxl_handle_error(result, error)

    def set_led_rgb(self, r: int, g: int, b: int) -> None:
        self.write_reg(self._reg_table.LED_RED, r)
        self.write_reg(self._reg_table.LED_GREEN, g)
        self.write_reg(self._reg_table.LED_BLUE, b)

    def set_return_delay_time(self, val: int) -> None:
        self.write_reg(self._reg_table.RETURN_DELAY_TIME, val)

    def set_moving_threshold(self, val: int) -> None:
        self.write_reg(self._reg_table.MOVING_THRESHOLD, val)

    def is_moving(self) -> bool:
        # inline code for little performance imporvement.
        rxpackage, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, self.motor_id, self._reg_table.MOVING.value[0]
        )
        self._dxl_handle_error(result, error)
        return rxpackage == 1

    def wait_for_init_movement(self, init_wait=0.05) -> None:
        time.sleep(init_wait)
        while self.is_moving():
            time.sleep(self.movement_wait_tick)

    def wait_for_movement(self) -> None:
        while self.is_moving():
            time.sleep(self.movement_wait_tick)

    def read_reg(self, name: Enum) -> int:
        addr = name.value[0]
        dat_len = name.value[1]
        return self.read_reg0(addr, dat_len)

    def write_reg(self, name: Enum, val: int) -> None:
        addr = name.value[0]
        dat_len = name.value[1]
        self.write_reg0(addr, dat_len, val)

    def read_reg0(self, addr: int, dat_len: int) -> None:
        rxpackage = None
        result = None
        error = None
        if dat_len == 1:
            rxpackage, result, error = self.packet_handler.read1ByteTxRx(
                self.port_handler, self.motor_id, addr
            )
        elif dat_len == 2:
            rxpackage, result, error = self.packet_handler.read2ByteTxRx(
                self.port_handler, self.motor_id, addr
            )
        elif dat_len == 4:
            rxpackage, result, error = self.packet_handler.read4ByteTxRx(
                self.port_handler, self.motor_id, addr
            )
        else:
            return None
        self._dxl_handle_error(result, error)
        return rxpackage

    def write_reg0(self, addr: int, dat_len: int, val: int) -> None:
        result = None
        error = None
        if dat_len == 1:
            result, error = self.packet_handler.write1ByteTxRx(
                self.port_handler, self.motor_id, addr, val
            )
        elif dat_len == 2:
            result, error = self.packet_handler.write2ByteTxRx(
                self.port_handler, self.motor_id, addr, val
            )
        elif dat_len == 4:
            result, error = self.packet_handler.write4ByteTxRx(
                self.port_handler, self.motor_id, addr, val
            )
        else:
            return
        self._dxl_handle_error(result, error)

    def move_to(self, val: int) -> None:
        self.goal_pos = val
        self.wait_for_init_movement()

    def move_to_deg(self, val: float) -> None:
        self.goal_pos_deg = val
        self.wait_for_init_movement()

    def move_to_rad(self, val: float) -> None:
        self.goal_pos_rad = val
        self.wait_for_init_movement()

    def set_min_position(self, val: float) -> bool:
        return self.write_reg(self._reg_table.MIN_POSITION_LIMIT, val)

    def set_max_position(self, val: float) -> bool:
        return self.write_reg(self._reg_table.MAX_POSITION_LIMIT, val)

    def get_min_position(self) -> int:
        return self.read_reg(self._reg_table.MIN_POSITION_LIMIT)

    def get_max_position(self) -> int:
        return self.read_reg(self._reg_table.MAX_POSITION_LIMIT)

    def close_port(self) -> None:
        self.port_handler.closePort()

    def _dxl_handle_error(self, result: int, error: int) -> None:
        if result != dxlsdk.COMM_SUCCESS:
            self.close_port()
        assert (
            result == dxlsdk.COMM_SUCCESS
        ), f"result = {result} :" + self.packet_handler.getRxPacketError(error)

    def _handle_error(self, complete: bool, msg: str = None) -> None:
        if msg is not None:
            print(msg)
        assert complete

    def deg_to_pulse(self, deg: float) -> int:
        return round(deg * self._DEG_TO_PULSE)

    def pluse_to_deg(self, val: int) -> float:
        return val / self._DEG_TO_PULSE

    def pluse_to_rad(self, val: int) -> float:
        return val / self._RAD_TO_PULSE

    def rad_to_pulse(self, rad: float) -> int:
        return round(rad * self._RAD_TO_PULSE)

    ## ============== [Properties] ==================

    @property
    def pos(self) -> int:
        return self.get_position()

    @property
    def pos_deg(self) -> float:
        return self.pluse_to_deg(self.get_position())

    @property
    def pos_rad(self) -> float:
        return self.pluse_to_rad(self.get_position())

    @property
    def current(self) -> int:
        return self.get_current()
    
    @property
    def vel(self) -> int:
        return self.get_velocity()
    
    @property
    def pwm(self) -> int:
        return self.get_pwm()
    
        # ======================

    @property
    def goal_pwm(self) -> int:
        return self.get_goal_pwm()
    
    @goal_pwm.setter
    def goal_pwm(self, val: int) -> None:
        self.set_goal_pwm(val)
        # ======================

    @property
    def goal_current(self) -> int:
        return self.pluse_to_rad(self.get_goal_current())

    @goal_current.setter
    def goal_current(self, val: int) -> None:
        self.set_goal_current(val)

        # ======================

    @property
    def goal_pos(self) -> int:
        return self.get_goal_position()

    @goal_pos.setter
    def goal_pos(self, val: int) -> None:
        self.set_goal_position(val)

        # ======================

    @property
    def goal_pos_deg(self) -> int:
        return self.pluse_to_deg(self.get_goal_position())

    @goal_pos_deg.setter
    def goal_pos_deg(self, val: int) -> None:
        self.set_goal_position(self.deg_to_pulse(val))

        # ======================

    @property
    def goal_pos_rad(self) -> int:
        return self.pluse_to_rad(self.get_goal_position())

    @goal_pos_rad.setter
    def goal_pos_rad(self, val: int) -> None:
        self.set_goal_position(self.rad_to_pulse(val))

        # ======================

    @property
    def accel_prof(self) -> int:
        return self.get_acceleration_prof()

    @accel_prof.setter
    def accel_prof(self, val: int) -> None:
        self.set_acceleration_prof(val)

        # ======================

    @property
    def vel_prof(self) -> int:
        return self.get_velocity_prof()

    @vel_prof.setter
    def vel_prof(self, val: int) -> None:
        self.set_velocity_prof(val)

        # ======================

    @property
    def ops_mode(self) -> int:
        return self.get_operating_mode()

    @ops_mode.setter
    def ops_mode(self, val: DXLOpsMode) -> None:
        self.set_operating_mode(val)

        # ======================

    @property
    def torque_en(self) -> int:
        return self.get_torque_enabled()

    @torque_en.setter
    def torque_en(self, val: bool) -> None:
        self.set_torque_enabled(val)

    ## ============== Min/Max pos ==================

    @property
    def min_pos(self) -> int:
        return self.get_min_position()

    @min_pos.setter
    def min_pos(self, val: int) -> None:
        self.set_min_position(val)

        # ======================

    @property
    def max_pos(self) -> int:
        return self.get_max_position()

    @max_pos.setter
    def max_pos(self, val: int) -> None:
        self.set_max_position(val)

        # ======================

    @property
    def min_pos_deg(self) -> int:
        return self.pluse_to_deg(self.get_min_position())

    @min_pos_deg.setter
    def min_pos_deg(self, deg: float) -> None:
        self.set_min_position(self.deg_to_pulse(deg))

        # ======================

    @property
    def max_pos_deg(self) -> int:
        return self.pluse_to_deg(self.get_max_position())

    @max_pos_deg.setter
    def max_pos_deg(self, deg: float) -> None:
        self.set_max_position(self.deg_to_pulse(deg))

    ## ============== [Utils] ==================

    @staticmethod
    def to_signed16(n):
        n = n & 0xffff
        return (n ^ 0x8000) - 0x8000

    @staticmethod
    def to_signed32(n):
        n = n & 0xffffffff
        return (n ^ 0x80000000) - 0x80000000

class DXLOpsMode(IntEnum):
    CURRENT_CONTROL = 0
    VELOCITY_CONTROL = 1
    POSITION_CONTROL = 3
    EXTENDED_POSITION_CONTROL = 4
    PWM_CONTROL = 16


class DXLBandRate(Enum):
    B_9600 = [0, 9600]
    B_57600 = [1, 57600]
    B_115200 = [2, 115200]
    B_1M = [3, 1e6]
    B_2M = [4, 2e6]
    B_3M = [5, 3e6]
    B_4M = [6, 4e6]
    B_4M5 = [7, 4.5e6]
    B_6M = [8, 6e6]
    B_10M5 = [9, 10.5e6]


class DXLTableXM(Enum):
    PULSE_PER_REV = 4096

    MODEL_NUMBER = [0, 2]
    MODEL_INFORMATION = [2, 4]
    FIRMWARE_VERSION = [6, 1]
    ID = [7, 1]
    BAND_RATE = [8, 1]
    RETURN_DELAY_TIME = [9, 1]
    DRIVE_MODE = [10, 1]
    OPERATING_MODE = [11, 1]
    SECONDARY_ID = [12, 1]
    PROTOCOL_TYPE = [13, 1]
    HOMING_OFFSET = [20, 4]
    MOVING_THRESHOLD = [24, 4]
    TEMPERATURE_LIMIT = [31, 1]
    MAX_VOLTAGE_LIMIT = [32, 2]
    MIN_VOLTAGE_LIMIT = [34, 2]
    PWM_LIMIT = [36, 2]
    CURRENT_LIMIT = [38, 2]
    ACCELERATION_LIMIT = [40, 4]
    VELOCITY_LIMIT = [44, 4]
    MAX_POSITION_LIMIT = [48, 4]
    MIN_POSITION_LIMIT = [52, 4]
    EXTERNAL_PORT_MODE_1 = [56, 1]
    EXTERNAL_PORT_MODE_2 = [57, 1]
    EXTERNAL_PORT_MODE_3 = [58, 1]
    EXTERNAL_PORT_MODE_4 = [59, 1]
    STARTUP_CONFIGURATION = [60, 1]
    SHUTDOWN = [63, 1]

    TORQUE_ENABLE = [64, 1]
    LED = [65, 1]
    STATUS_RETURN_LEVEL = [68, 1]
    REGISTERED_INSTRUCTION = [69, 1]
    HARDWARE_ERROR_STATUS = [70, 1]

    VELOCITY_I_GAIN = [76, 2]
    VELOCITY_P_GAIN = [78, 2]

    POSITION_D_GAIN = [80, 2]
    POSITION_I_GAIN = [82, 2]
    POSITION_P_GAIN = [84, 2]

    FEEDFORWARD_2ND_GAIN = [88, 2]
    FEEDFORWARD_1ST_GAIN = [90, 2]
    BUS_WATCHDOG = [98, 1]

    GOAL_PWM = [100, 2]
    GOAL_CURRENT = [102, 2]
    GOAL_VELOCITY = [104, 4]
    PROFILE_ACCELERATION = [108, 4]
    PROFILE_VELOCITY = [112, 4]
    GOAL_POSITION = [116, 4]
    REALTIME_TICK = [120, 2]
    MOVING = [122, 1]
    MOVING_STATUS = [123, 1]

    PRESENT_PWM = [124, 2]
    PRESENT_CURRENT = [126, 2]
    PRESENT_VELOCITY = [128, 4]
    PRESENT_POSITION = [132, 4]

    VELOCITY_TRAJECTORY = [136, 4]
    POSITION_TRAJECTORY = [140, 4]
    PRESENT_INPUT_VOLTAGE = [144, 2]
    PRESENT_TEMPERATURE = [146, 1]


class DXLTablePH(Enum):
    PULSE_PER_REV = 607500

    MODEL_NUMBER = [0, 2]
    MODEL_INFORMATION = [2, 4]
    FIRMWARE_VERSION = [6, 1]
    ID = [7, 1]
    BAND_RATE = [8, 1]
    RETURN_DELAY_TIME = [9, 1]
    DRIVE_MODE = [10, 1]
    OPERATING_MODE = [11, 1]
    SECONDARY_ID = [12, 1]
    PROTOCOL_TYPE = [13, 1]
    HOMING_OFFSET = [20, 4]
    MOVING_THRESHOLD = [24, 4]
    TEMPERATURE_LIMIT = [31, 1]
    MAX_VOLTAGE_LIMIT = [32, 2]
    MIN_VOLTAGE_LIMIT = [34, 2]
    PWM_LIMIT = [36, 2]
    CURRENT_LIMIT = [38, 2]
    ACCELERATION_LIMIT = [40, 4]
    VELOCITY_LIMIT = [44, 4]
    MAX_POSITION_LIMIT = [48, 4]
    MIN_POSITION_LIMIT = [52, 4]
    EXTERNAL_PORT_MODE_1 = [56, 1]
    EXTERNAL_PORT_MODE_2 = [57, 1]
    EXTERNAL_PORT_MODE_3 = [58, 1]
    EXTERNAL_PORT_MODE_4 = [59, 1]
    STARTUP_CONFIGURATION = [60, 1]
    SHUTDOWN = [63, 1]

    TORQUE_ENABLE = [512, 1]
    LED_RED = [513, 1]
    LED_GREEN = [514, 1]
    LED_BLUE = [515, 1]
    STATUS_RETURN_LEVEL = [516, 1]
    REGISTERED_INSTRUCTION = [517, 1]
    HARDWARE_ERROR_STATUS = [518, 1]

    VELOCITY_I_GAIN = [524, 2]
    VELOCITY_P_GAIN = [526, 2]

    POSITION_D_GAIN = [528, 2]
    POSITION_I_GAIN = [530, 2]
    POSITION_P_GAIN = [532, 2]

    FEEDFORWARD_2ND_GAIN = [536, 2]
    FEEDFORWARD_1ST_GAIN = [538, 2]
    BUS_WATCHDOG = [546, 1]

    GOAL_PWM = [548, 2]
    GOAL_CURRENT = [550, 2]
    GOAL_VELOCITY = [552, 4]
    PROFILE_ACCELERATION = [556, 4]
    PROFILE_VELOCITY = [560, 4]
    GOAL_POSITION = [564, 4]
    REALTIME_TICK = [568, 2]
    MOVING = [570, 1]
    MOVING_STATUS = [571, 1]

    PRESENT_PWM = [572, 2]
    PRESENT_CURRENT = [574, 2]
    PRESENT_VELOCITY = [576, 4]
    PRESENT_POSITION = [580, 4]

    VELOCITY_TRAJECTORY = [584, 4]
    POSITION_TRAJECTORY = [588, 4]
    PRESENT_INPUT_VOLTAGE = [592, 2]
    PRESENT_TEMPERATURE = [594, 1]
    EXTERNAL_PORT_DATA_1 = [600, 2]
    EXTERNAL_PORT_DATA_2 = [602, 2]
    EXTERNAL_PORT_DATA_3 = [604, 2]
    EXTERNAL_PORT_DATA_4 = [606, 2]
