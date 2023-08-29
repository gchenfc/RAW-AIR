"""
@file dynamixel.py
@author Gerry Chen
@date Apr 8, 2023

Handles communication with Dynamixel servos (AX-12+) for the Georgia Tech Library's
Artist-In-Residence program, and ME Capstone team Spring 2023.

Usage:

```
with AX12s(port=port, baudrate=baudrate) as servos:  # args like serial.Serial
    servos.ping(2)  # pings motor number 2
    print(servos.read_all_joint_angles_deg())  # prints the list of all 6 joint angles
    servos.command_angle(2, 90)  # sets joint 2 to 90 degrees
    servos.command_angles_deg([0, 0, 0, 0, 0])  # sets all joints to 0 degrees
```

See also gerry01_test_servos.ipynb for a Jupyter notebook with more examples.
"""

import serial
import time
from typing import Union, Optional
import dataclasses

Byte = int  # type -> 0-255
Data = Union[Byte, bytes, list[Byte]]


class StatusParams(bytes):
    def value(self) -> int:
        return int.from_bytes(self, byteorder='little')
    def __repr__(self) -> str:
        return str(self.value())
    def __str__(self) -> str:
        return str(self.value())

@dataclasses.dataclass
class Status:
    id: Byte
    error: Byte
    data: StatusParams
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


def first_packet_length(packet: bytes, expected_len: int = None) -> bytes:
    assert packet[0] == 0xFF and packet[1] == 0xFF, 'Wrong header'
    if len(packet) < 4:
        return None
    length = packet[3]
    if expected_len is not None and length != expected_len:
        print('\033[93m', "warning: packet length didn't match expected length", '\033[0m')
        length = expected_len  # just try to hack it
    if len(packet) < length + 4:
        return None
    return length + 4


def peak_first_packet(packet: bytearray, expected_len: int = None) -> bytes:
    length = first_packet_length(packet, expected_len=expected_len)
    if length is None:
        return None
    return packet[:length]


def pop_first_packet(packet: bytearray, expected_len: int = None) -> bytes:
    length = first_packet_length(packet, expected_len=expected_len)
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
assert decode_packet(exp_packet) == Status(*exp_id_instr_data), 'decode_packet failed unit test'


class Dynamixel(serial.Serial):
    def __init__(self, *args, write_packet_delay=0.001, print_warnings=True, **kwargs):
        super().__init__(*args, **kwargs, timeout=0.1)
        self.buffer = bytearray()
        self.write_packet_delay = write_packet_delay
        self.VALID_IDS = set(range(6))
        self.warn = (lambda *args, **kwargs: print('\033[93m', *args, '\033[0m', **kwargs)
                    ) if print_warnings else lambda *args, **kwargs: None

    def __exit__(self, *exc):
        super().__exit__(*exc)
        if self.write_packet_delay > 0.05: # we're probably using bluetooth
            time.sleep(1)  # Give the bluetooth some time to close properly
    def write_packet(self, *args):
        toret = self.write(create_packet(*args))
        time.sleep(self.write_packet_delay)
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
    def read_all_msgs(self, exp_data_bytes=None, strict_ids=True):
        exp_len = exp_data_bytes + 2 if exp_data_bytes is not None else None
        self.buffer += self.read(self.in_waiting)
        while len(self.buffer) >= 4:
            while ((self.buffer[0] != 0xFF) or (self.buffer[1] != 0xFF) or
                   (strict_ids and (self.buffer[2] not in self.VALID_IDS))):
                # self.warn('Lost data!', self.buffer[0])
                self.buffer.pop(0)  # Lost data!
                if len(self.buffer) < 4:
                    return None
            try:
                packet = pop_first_packet(self.buffer, expected_len=exp_len)
                if packet is None:
                    return None
                yield decode_packet(packet)
            except AssertionError as e:
                self.warn('warning: assertion error in read_all_msgs: ', e)
                continue
        return None
    def read_next_msg(self, exp_data_bytes=None, strict_ids=True):
        for msg in self.read_all_msgs(exp_data_bytes=exp_data_bytes, strict_ids=strict_ids):
            return msg
        return None
    def clear_buffer(self):
        self.read(self.in_waiting)
        self.buffer.clear()

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
    x = int(x)
    return [x >> (i * 8) & 0xff for i in range(n)]
def deg2counts(id, angle: float):
    if id == 3 or id == 5:
        angle = -angle
    return int((angle + 150) / 300 * 1023)
def counts2deg(id, counts: int):
    return (counts / 1023 * 300 - 150) * (-1 if id == 3 or id == 5 else 1)


class AX12s(AX12):
    def __init__(self, *args, read_all_timeout=1.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.read_all_timeout = read_all_timeout
        self.set_speed(25)
        self.speeds = [25 for _ in range(6)]  # This gets set in set_speed, but in case you comment out set_speed

    def write_all(self, addr: Byte, value: Data, nbytes:Optional[int]=None):
        if nbytes is not None and isinstance(value, int):
            value = int2bytes(value, nbytes)
        return self.sync_write(addr, [(i, value) for i in range(6)])
    def read_all(self, addr: Byte, length: Byte = 1):
        for id in range(6):
            self.read_data(id, addr, length)
        ret = []
        tstart = time.time()
        while len(ret) < 6 and time.time() - tstart < self.read_all_timeout:
            ret += list(self.read_all_msgs(exp_data_bytes=length))
        if len(ret) < 6:  # re-query the missing values
            self.warn('warning: read_all timed out, trying to re-query for missing values')
            new_ret = [None for _ in range(6)]
            for status in ret:
                new_ret[status.id] = status
            for id in range(6):
                if new_ret[id] is None:
                    self.warn(f'\tre-querying {id=}')
                    self.read_data(id, addr, length)
            tstart = time.time()
            while (any(status is None for status in new_ret) and
                   (time.time() - tstart < self.read_all_timeout)):
                for status in self.read_all_msgs(exp_data_bytes=length):
                    new_ret[status.id] = status
            ret = [status for status in new_ret if status is not None]
            assert len(ret) == 6, f'Error reading all: {ret = }'
            self.warn('\tSuccess re-querying missing values!!!')
        return ret

    def enable_all(self):
        return self.write_all(AX12.TORQUE_ENABLE, 1)
    def disable_all(self):
        return self.write_all(AX12.TORQUE_ENABLE, 0)
    def set_speed(self, speed_counts):
        self.speeds = [speed_counts for _ in range(6)]
        return self.write_all(AX12.MOVING_SPEED, int2bytes(speed_counts, 2))
    def set_speeds(self, speeds_counts):
        assert len(speeds_counts) == 6, 'Must have 6 speeds'
        self.speeds = [v for v in speeds_counts]
        return self.sync_write(AX12.MOVING_SPEED,
                               [(i, int2bytes(abs(s), 2)) for i, s in enumerate(speeds_counts)])

    def set_compliance_margins(self, margin: int):
        assert margin < 256, 'Margin must be less than 256'
        return [
            self.write_all(AX12.CW_COMPLIANCE_MARGIN, margin, nbytes=1),
            self.write_all(AX12.CCW_COMPLIANCE_MARGIN, margin, nbytes=1)
        ]
    def set_compliance_slopes(self, slope: int):
        assert slope < 256, 'Slope must be less than 256'
        return [
            self.write_all(AX12.CW_COMPLIANCE_SLOPE, slope, nbytes=1),
            self.write_all(AX12.CCW_COMPLIANCE_SLOPE, slope, nbytes=1)
        ]

    def read_all_joint_angles_deg(self):
        return [
            counts2deg(id, m.data.value())
            for id, m in enumerate(self.read_all(AX12.PRESENT_POSITION, 2))
        ]
    def joint_angles_deg(self):
        ret = self.read_all_joint_angles_deg()
        assert len(ret) == 6, f'Error reading joint angles: {ret = }'
        return [ret[0], (ret[1] - ret[2]) / 2, ret[3], ret[4], ret[5]]
    def joint_angles_string(self):
        joint_angles = self.joint_angles_deg()
        return ' '.join([f'{angle:3.0f}' for angle in joint_angles])

    def command_angle(self, id: Byte, angle: float):
        # When writing goal position, also write speed since sometimes it gets reset to 0 due to brownout and moves really fast
        return self.write_data(id, AX12.GOAL_POSITION, int2bytes(deg2counts(id, angle), 2) + int2bytes(self.speeds[id], 2))
    def _command_angles_counts(self, *angles):
        return self.sync_write(AX12.GOAL_POSITION,
                               [(i, int2bytes(angle, 2) + int2bytes(self.speeds[i], 2)) for i, angle in enumerate(angles)])
    def command_angles_deg(self, *angles):
        assert len(angles) == 5, 'Must have 5 joint angles'
        assert min(angles) >= -150, 'Angles should range from -150 to 150'
        assert max(angles) <= 150, 'Angles should range from -150 to 150'
        angles = [angles[0], angles[1], -angles[1], *angles[2:]]
        angles = [deg2counts(id, angle) for id, angle in enumerate(angles)]
        return self._command_angles_counts(*angles)
