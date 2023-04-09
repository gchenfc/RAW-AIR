"""
@file dynamixel.py
@author Gerry Chen
@date Apr 8, 2023

Handles communication with Dynamixel servos (AX-12+) for the Georgia Tech Library's
Artist-In-Residence program, and ME Capstone team Spring 2023.

Usage:

```
with Arm(port=port, baudrate=baudrate) as arm:  # args like serial.Serial
    arm.ping(2)  # pings motor number 2
    print(arm.read_all_joint_angles_deg())  # prints the list of all 6 joint angles
    arm.command_angle(2, 90)  # sets joint 2 to 90 degrees
    arm.go_to_blocking([0, 0, 0, 0, 0], tol=tol, timeout=timeout)  # moves to home position
```

See also gerry01_test_servos.ipynb for a Jupyter notebook with more examples.
"""

import serial
import time
from typing import Union, Optional
from collections import namedtuple

Byte = int  # type -> 0-255
Data = Union[Byte, bytes, list[Byte]]


class StatusParams(bytes):
    def value(self) -> int:
        return int.from_bytes(self, byteorder='little')
    def __repr__(self) -> str:
        return str(self.value())
    def __str__(self) -> str:
        return str(self.value())

class Status(namedtuple('Status', ['id', 'error', 'data'])):
    def __repr__(self):
        return f'Status(id=0x{self.id:02x}, error={self.error:08b}, data={self.data})'


def checksum(data: bytes) -> Byte:
    return ~(sum(data) & 0xff) & 0xff


def create_packet(id: Byte, instr: Byte, data: Data) -> bytes:
    if isinstance(data, Byte):
        data = [data]
    packet = [0xFF, 0xFF, id, 2 + len(data), instr, *data]
    packet.append(checksum(packet[2:]))
    return bytes(packet)


def decode_packet(packet: bytes) -> Status:
    assert packet[0] == 0xFF and packet[1] == 0xFF, 'Wrong header'
    assert packet[-1] == checksum(packet[2:-1]), f'Wrong checksum: {packet}'
    id = packet[2]
    length = packet[3]
    assert length == len(packet) - 4, 'Wrong length'
    error = packet[4]
    data = packet[5:-1]
    return Status(id, error, StatusParams(data))


def first_packet_length(packet: bytes) -> bytes:
    assert packet[0] == 0xFF and packet[1] == 0xFF, 'Wrong header'
    if len(packet) < 4:
        return None
    length = packet[3]
    if len(packet) < length + 4:
        return None
    return length + 4


def peak_first_packet(packet: bytearray) -> bytes:
    length = first_packet_length(packet)
    if length is None:
        return None
    return packet[:length]


def pop_first_packet(packet: bytearray) -> bytes:
    length = first_packet_length(packet)
    if length is None:
        return None
    toret = packet[:length]
    del packet[:length]
    return toret


# Unit tests
# https://emanual.robotis.com/docs/en/dxl/protocol1/#checksum-instruction-packet
assert checksum(bytes([0x01, 0x05, 0x03, 0x0C, 0x64, 0xAA])) == 0xDC, 'checksum failed unit test'
exp_id_instr_data = 1, 3, bytes([0x0C, 0x64, 0xAA])
exp_packet = bytes([0xFF, 0xFF, 0x01, 0x05, 0x03, 0x0C, 0x64, 0xAA, 0xDC])
assert create_packet(*exp_id_instr_data) == exp_packet, 'create_packet failed unit test'
assert decode_packet(exp_packet) == exp_id_instr_data, 'decode_packet failed unit test'


class Dynamixel(serial.Serial):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs, timeout=0.1)
        self.buffer = bytearray()
    def write_packet(self, *args):
        toret = self.write(create_packet(*args))
        time.sleep(0.001)
        return toret
    def ping(self, id: Byte):
        return self.write_packet(id, 1, [])
    def read_data(self, id: Byte, addr: Byte, length: Byte):
        return self.write_packet(id, 2, [addr, length])
    def write_data(self, id: Byte, addr: Byte, data: Data):
        if isinstance(data, Byte):
            data = [data]
        return self.write_packet(id, 3, [addr, *data])
    def write_data_reg(self, id: Byte, addr: Byte, data: Data):
        if isinstance(data, Byte):
            data = [data]
        return self.write_packet(id, 4, [addr, *data])
    def action(self, id: Byte):
        return self.write_packet(id, 5, [])
    def factory_reset(self, id: Byte):
        return self.write_packet(id, 6, [])
    def sync_write(self, addr: Byte, id_data_pairs: list[tuple[Byte, Data]]):
        data = []
        for id, dat in id_data_pairs:
            if isinstance(dat, Byte):
                dat = [dat]
            data += [id, *dat]
        return self.write_packet(0xFE, 0x83, [addr, len(dat), *data])
    def read_all_msgs(self):
        self.buffer += self.read(self.in_waiting)
        while len(self.buffer) >= 4:
            while self.buffer[0] != 0xFF or self.buffer[1] != 0xFF:
                print('Lost data!', self.buffer[0])
                self.buffer.pop(0)  # Lost data!
                if len(self.buffer) < 4:
                    return None
            packet = pop_first_packet(self.buffer)
            if packet is None:
                return None
            yield decode_packet(packet)
        return None
    def read_next_msg(self):
        for msg in self.read_all_msgs():
            return msg
        return None

class AX12(Dynamixel):
    # Control table
    # https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/#control-table-of-ax-12a
    MODEL_NUMBER = 0
    VERSION_OF_FIRMWARE = 2
    ID = 3
    BAUD_RATE = 4
    RETURN_DELAY_TIME = 5
    CW_ANGLE_LIMIT = 6
    CCW_ANGLE_LIMIT = 8
    TEMPERATURE_LIMIT = 11
    MIN_VOLTAGE_LIMIT = 12
    MAX_VOLTAGE_LIMIT = 13
    MAX_TORQUE = 14
    STATUS_RETURN_LEVEL = 16
    ALARM_LED = 17
    ALARM_SHUTDOWN = 18
    TORQUE_ENABLE = 24
    LED = 25
    CW_COMPLIANCE_MARGIN = 26
    CCW_COMPLIANCE_MARGIN = 27
    CW_COMPLIANCE_SLOPE = 28
    CCW_COMPLIANCE_SLOPE = 29
    GOAL_POSITION = 30
    MOVING_SPEED = 32
    TORQUE_LIMIT = 34
    PRESENT_POSITION = 36
    PRESENT_SPEED = 38
    PRESENT_LOAD = 40
    PRESENT_VOLTAGE = 42
    PRESENT_TEMPERATURE = 43
    REGISTERED_INSTRUCTION = 44
    MOVING = 46
    LOCK = 47
    PUNCH = 48


def int2bytes(x: int, n: int):
    return [x >> (i * 8) & 0xff for i in range(n)]
def deg2counts(angle: float):
    return int((angle + 150) / 300 * 1023)
def counts2deg(counts: int):
    return counts / 1023 * 300 - 150


class Arm(AX12):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.write_all(AX12.MOVING_SPEED, int2bytes(25, 2))

    def write_all(self, addr: Byte, value: Data, nbytes:Optional[int]=None):
        if nbytes is not None and isinstance(value, int):
            value = int2bytes(value, nbytes)
        return self.sync_write(addr, [(i, value) for i in range(6)])
    def read_all(self, addr: Byte, len: Byte = 1):
        for id in range(6):
            self.read_data(id, addr, len)
        return list(self.read_all_msgs())
    def read_all_joint_angles_deg(self,):
        return [counts2deg(m.data.value()) for m in self.read_all(AX12.PRESENT_POSITION, 2)]
    def command_angle(self, id: Byte, angle: float):
        return self.write_data(id, AX12.GOAL_POSITION, int2bytes(deg2counts(angle), 2))
    def _command_angles_counts(self, *angles):
        return self.sync_write(AX12.GOAL_POSITION,
                               [(i, int2bytes(angle, 2)) for i, angle in enumerate(angles)])
    def command_angles_deg(self, *angles):
        assert len(angles) == 5, 'Must have 5 joint angles'
        assert min(angles) >= -150, 'Angles should range from -150 to 150'
        assert max(angles) <= 150, 'Angles should range from -150 to 150'
        angles = [angles[0], angles[1], -angles[1], *angles[2:]]
        angles = [deg2counts(angle) for angle in angles]
        return self._command_angles_counts(*angles)

    # Go to setpoint
    def reached_goal(self, goal, tol=5):
        if len(goal) == 5:
            goal = [goal[0], goal[1], -goal[1], *goal[2:]]
        elif len(goal) != 6:
            raise ValueError('goal must be length 5 or 6')
        try:
            actual = self.read_all_joint_angles_deg()
        except AssertionError as e:
            print('warning: assertion error in reached_goal: ', e)
            return False
        return all([abs(g - a) < tol for g, a in zip(goal, actual)])
    def go_to_blocking(self, goal, tol=5, timeout=None, cb=None):
        if tol is None:
            tol = 5
        self.command_angles_deg(*goal)
        tstart = time.time()
        while not self.reached_goal(goal, tol=tol):
            self.command_angles_deg(*goal)
            if cb is not None:
                try:
                    cb()
                except AssertionError as e:
                    print('warning: callback had an assertion error in go_to_blocking: ', e)
            time.sleep(0.1)
            if timeout is not None and time.time() - tstart > timeout:
                return False
        return True
