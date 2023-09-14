from __future__ import annotations
from enum import Enum, IntEnum
import dynamixel_sdk as dxlsdk


class SerDXL:
    """Easy Dynamixel Interface for Python.
    Note: only tested on PH54
    """

    pulse_per_rev = 607500  # for PH42-020-S300-R

    def __init__(
        self, port: str, bandrate: int = 57600, protocol_ver: float = 2.0, motor_id=1
    ) -> None:
        self.port_handler = dxlsdk.PortHandler(port)
        self.motor_id = motor_id
        self._protocol_version = protocol_ver
        self.packet_handler = dxlsdk.PacketHandler(self._protocol_version)
        self._bandrate = bandrate
        self.port_handler.openPort()

    def set_operating_mode(self, mode: DXLOpsMode) -> None:
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.motor_id,
            DXLTableDefault.OPERATING_MODE[0],
            int(mode),
        )
        self._dxl_handle_error(result, error)

    def get_operating_mode(self) -> None:
        rxpackage, result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler,
            self.motor_id,
            DXLTableDefault.OPERATING_MODE[0],
        )
        self._dxl_handle_error(result, error)
        return rxpackage

    def set_torque_enabled(self, enable: bool) -> None:
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.motor_id,
            DXLTableDefault.TORQUE_ENABLE[0],
            int(enable),
        )
        self._dxl_handle_error(result, error)

    def get_position(self) -> int:
        rxpackage, result, error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, DXLTableDefault.PRESENT_POSITION[0]
        )
        self._dxl_handle_error(result, error)
        return rxpackage

    def set_position(self, val: int) -> None:
        result, error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, DXLTableDefault.PRESENT_POSITION[0], val
        )
        self._dxl_handle_error(result, error)

    def set_band_rate(self, val: DXLBandRate) -> None:
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.motor_id, DXLTableDefault.BAND_RATE[0], val
        )
        self._dxl_handle_error(result, error)

    def set_led(self, r: int, g: int, b: int) -> None:
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.motor_id,
            DXLTableDefault.LED_RED[0],
            r,
        )
        self._dxl_handle_error(result, error)
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.motor_id,
            DXLTableDefault.LED_GREEN[0],
            g,
        )
        self._dxl_handle_error(result, error)
        result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler,
            self.motor_id,
            DXLTableDefault.LED_BLUE[0],
            b,
        )
        self._dxl_handle_error(result, error)

    def read_reg(self, name: DXLTableDefault) -> int:
        addr = name[0]
        dat_len = name[1]
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

    def write_reg(self, name: DXLTableDefault, val: int) -> int:
        addr = name[0]
        dat_len = name[1]
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

    def _dxl_handle_error(self, result: int, error: int) -> None:
        print(self.packet_handler.getRxPacketError(error))
        assert result == 0

    def _handle_error(self, complete: bool, msg: str = None) -> None:
        if msg is not None:
            print(msg)
        assert complete

    @property
    def pos(self) -> int:
        return self.get_position()

    @pos.setter
    def pos(self, val: int) -> None:
        self.set_position(val)

    @property
    def ops_mode(self) -> int:
        return self.get_operating_mode()

    @ops_mode.setter
    def ops_mode(self, val: DXLOpsMode) -> None:
        self.set_operating_mode(val)


class DXLOpsMode(IntEnum):
    CURRENT_CONTROL = 0
    VELOCITY_CONTROL = 1
    POSITION_CONTROL = 3
    EXTENDED_POSITION_CONTROL = 4
    VOLTAGE_CONTROL = 16


class DXLBandRate(IntEnum):
    B_9600 = 0
    B_57600 = 1
    B_115200 = 2
    B_1M = 3
    B_2M = 4
    B_3M = 5
    B_4M = 6
    B_4M5 = 7
    B_6M = 8
    B_10M5 = 9


class DXLTableDefault(Enum):
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
