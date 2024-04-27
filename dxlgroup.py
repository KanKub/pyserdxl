from __future__ import annotations
from enum import Enum

import numpy as np
import dynamixel_sdk as dxlsdk
from numpy import ndarray
from .pyserdxl import DXLOpsMode, SerDxl, DXLTablePH

__author__ = "Kan Keawhanam"
__email__ = "kan.kea57@gmail.com"
__version__ = "0.2.1"


class DxlGroup:

    """Dynamixel motors group control interface.
    Note: Bulk read/write protocol are not implemented.
    """

    motors: list[SerDxl] = []

    def __init__(
        self,
        port: str,
        ids: list[int],
        bandrate: int = 57600,
        protocol_ver: float = 2.0,
        series: Enum = None,
    ) -> None:
        self.port_handler = dxlsdk.PortHandler(port)
        for motor_id in ids:
            self.motors.append(
                SerDxl(
                    None,
                    bandrate=bandrate,
                    protocol_ver=protocol_ver,
                    motor_id=motor_id,
                    series=series,
                    port_handler=self.port_handler,
                )
            )

    def set_position_p_gain(self, val: int) -> None:
        self.bulk_write_reg(DXLTablePH.POSITION_P_GAIN.name, val)

    def set_position_i_gain(self, val: int) -> None:
        self.bulk_write_reg(DXLTablePH.POSITION_I_GAIN.name, val)

    def set_position_d_gain(self, val: int) -> None:
        self.bulk_write_reg(DXLTablePH.POSITION_D_GAIN.name, val)

    def set_velocity_p_gain(self, val: int) -> None:
        self.bulk_write_reg(DXLTablePH.VELOCITY_P_GAIN.name, val)

    def set_velocity_i_gain(self, val: int) -> None:
        self.bulk_write_reg(DXLTablePH.VELOCITY_I_GAIN.name, val)

    def led_rgb(self, r: int, g: int, b: int) -> None:
        for motor in self.motors:
            motor.set_led_rgb(r, g, b)

    def led(self, enable: bool) -> None:
        for motor in self.motors:
            motor.set_led(enable)

    def set_goal_velocities(self, vals: list[int]) -> None:
        length = len(self.motors)
        for i in range(length):
            self.motors[i].goal_vel = vals[i]

    def set_goal_positions(self, vals: list[int]) -> None:
        length = len(self.motors)
        for i in range(length):
            self.motors[i].goal_pos = vals[i]

    def set_goal_positions_rad(self, vals: list[float]) -> None:
        length = len(self.motors)
        for i in range(length):
            self.motors[i].goal_pos_rad = vals[i]

    def set_goal_positions_deg(self, vals: list[float]) -> None:
        length = len(self.motors)
        for i in range(length):
            self.motors[i].goal_pos_deg = vals[i]

    def move_to(self, vals: list[int]) -> None:
        length = len(self.motors)
        for i in range(length):
            self.motors[i].goal_pos = vals[i]
        for i in range(length):
            self.motors[i].wait_for_init_movement()

    def move_to_deg(self, angs: list[float]) -> None:
        length = len(self.motors)
        for i in range(length):
            self.motors[i].goal_pos_deg = angs[i]
        for i in range(length):
            self.motors[i].wait_for_init_movement()

    def move_to_rad(self, angs: list[float]) -> None:
        length = len(self.motors)
        for i in range(length):
            self.motors[i].goal_pos_rad = angs[i]
        for i in range(length):
            self.motors[i].wait_for_init_movement()

    def get_goal_velocities(self) -> ndarray:
        length = len(self.motors)
        res = np.ones(length) * np.nan
        for i in range(length):
            res[i] = self.motors[i].get_goal_velocity()
        return res

    def get_positions(self) -> ndarray:
        length = len(self.motors)
        res = np.ones(length) * np.nan
        for i in range(length):
            res[i] = self.motors[i].get_position()
        return res

    def get_positions_deg(self) -> ndarray:
        length = len(self.motors)
        res = np.ones(length) * np.nan
        for i in range(length):
            res[i] = self.motors[i].pos_deg
        return res

    def get_positions_rad(self) -> ndarray:
        length = len(self.motors)
        res = np.ones(length) * np.nan
        for i in range(length):
            res[i] = self.motors[i].pos_rad
        return res

    def get_goal_positions(self) -> ndarray:
        length = len(self.motors)
        res = np.ones(length) * np.nan
        for i in range(length):
            res[i] = self.motors[i].goal_pos
        return res

    def get_goal_positions_deg(self) -> ndarray:
        length = len(self.motors)
        res = np.ones(length) * np.nan
        for i in range(length):
            res[i] = self.motors[i].goal_pos_deg
        return res

    def get_goal_positions_rad(self) -> ndarray:
        length = len(self.motors)
        res = np.ones(length) * np.nan
        for i in range(length):
            res[i] = self.motors[i].goal_pos_rad
        return res
    
    def get_currents(self) -> ndarray:
        length = len(self.motors)
        res = np.ones(length) * np.nan
        for i in range(length):
            res[i] = self.motors[i].current
        return res
    
    def get_pwm(self) -> ndarray:
        length = len(self.motors)
        res = np.ones(length) * np.nan
        for i in range(length):
            res[i] = self.motors[i].pwm
        return res
    
    def get_velocities(self) -> ndarray:
        length = len(self.motors)
        res = np.ones(length) * np.nan
        for i in range(length):
            res[i] = self.motors[i].vel
        return res


    def set_moving_threshold(self, val: int) -> None:
        self.bulk_write_reg(DXLTablePH.MOVING_THRESHOLD.name, val)

    def bulk_write_reg(self, name: str, val: int) -> None:
        for motor in self.motors:
            motor.write_reg(motor.reg_table[name], val)

    def bulk_read_reg(self, name: str) -> list:
        res = []
        for motor in self.motors:
            res.append(motor.read_reg(motor.reg_table[name]))
        return res

    def set_operating_mode(self, mode: DXLOpsMode) -> None:
        for motor in self.motors:
            motor.set_operating_mode(mode)

    def set_accel_prof(self, val: int) -> None:
        self.bulk_write_reg(DXLTablePH.PROFILE_ACCELERATION.name, val)

    def set_vel_prof(self, val: int) -> None:
        self.bulk_write_reg(DXLTablePH.PROFILE_VELOCITY.name, val)

    def set_torque_enabled(self, enable: bool) -> None:
        for motor in self.motors:
            motor.set_torque_enabled(enable)

    def close_port(self) -> None:
        for motor in self.motors:
            motor.close_port()

    @property
    def pos(self) -> ndarray:
        return self.get_positions()

    @property
    def pos_deg(self) -> ndarray:
        return self.get_positions_deg()

    @property
    def pos_rad(self) -> ndarray:
        return self.get_positions_rad()

    @property
    def goal_vel(self) -> ndarray:
        return self.get_goal_velocities()

    @goal_vel.setter
    def goal_vel(self, val: list[int]) -> None:
        self.set_goal_velocities(val)

    @property
    def goal_pos(self) -> ndarray:
        return self.get_goal_positions()

    @property
    def goal_pos_deg(self) -> ndarray:
        return self.get_goal_positions_deg()

    @property
    def goal_pos_rad(self) -> ndarray:
        return self.get_goal_positions_rad()

    @goal_pos.setter
    def goal_pos(self, val: list[int]) -> None:
        self.set_goal_positions(val)

    @goal_pos_deg.setter
    def goal_pos_deg(self, val: list[float]) -> None:
        self.set_goal_positions_deg(val)

    @goal_pos_rad.setter
    def goal_pos_rad(self, val: list[float]) -> None:
        self.set_goal_positions_rad(val)

    @property
    def torque(self) -> list:
        self.bulk_read_reg(DXLTablePH.TORQUE_ENABLE.name)

    @torque.setter
    def torque(self, enable: bool) -> None:
        self.set_torque_enabled(enable)