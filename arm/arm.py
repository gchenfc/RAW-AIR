"""
@file arm.py
@author Gerry Chen
@date Apr 8, 2023

Handles arm movement for the Georgia Tech Library's Artist-In-Residence program, and ME Capstone
team Spring 2023.

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

from dynamixel import AX12s, Byte, Data
import time
from typing import Optional
import dataclasses
from util import SE3
import kinematics

@dataclasses.dataclass
class JointPathWaypoint:
    q: list[float]  # The joint angles to go to (in degrees)
    tol: float = 5  # The tolerance for checking when the robot has reached the goal
    timeout: float = None  # The timeout: if the robot hasn't reached the goal after this time, move on anyway
    pause: float = 0  # The time to pause after reaching the goal
    speeds: Optional[list[float]] = None  # Speeds to use for each joint (in counts)

@dataclasses.dataclass
class PosePathWaypoint:
    Ts: list[SE3]  # The joint angles to go to (in degrees)
    tol: float = 5  # TODO: change this to position tolerance instead of joint angle
    timeout: float = None  # The timeout: if the robot hasn't reached the goal after this time, move on anyway
    pause: float = 0  # The time to pause after reaching the goal
    speeds: Optional[list[float]] = None  # Speeds to use for each joint (in counts)


class Arm(AX12s):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

    # Check if the robot is at the goal position
    def reached_goal(self, goal, tol=5, angles_out=None):
        if len(goal) == 5:
            goal = [goal[0], goal[1], -goal[1], *goal[2:]]
        elif len(goal) != 6:
            raise ValueError('goal must be length 5 or 6')
        try:
            actual = self.read_all_joint_angles_deg()
        except AssertionError as e:
            print('warning: assertion error in reached_goal: ', e)
            return False
        if angles_out is not None:
            angles_out[:] = actual
        return all([abs(g - a) < tol for g, a in zip(goal, actual)])

    # Go to setpoint joint configuration, in degrees
    def go_to_blocking(self, goal, tol=5, timeout=None, cb=None, speeds=None, default_speed=25):
        original_goal = list(goal)
        original_goal_6 = [goal[0], goal[1], -goal[1], *goal[2:]]
        if tol is None:
            tol = 5
        if speeds is not None:
            speeds = [speeds[0], speeds[1], speeds[1], *speeds[2:]]
            print(f'  speeds: {speeds}')
            self.set_speeds(speeds)
        else:
            self.set_speed(default_speed)
        self.command_angles_deg(*goal)
        tstart = time.time()
        actual_prev = [0, 0, 0, 0, 0, 0]
        actual = [0, 0, 0, 0, 0, 0]
        while not self.reached_goal(original_goal, tol=tol, angles_out=actual):
            # # Check if we are not moving
            # if all([abs(a - b) < 1 for a, b in zip(actual, actual_prev)]):
            #     # Check if we are close to the goal
            #     if all([abs(g - a) < 10 for g, a in zip(original_goal_6, actual)]):
            #         # If so, then figure out which motor is the problem and set the setpoint to
            #         # overshoot the goal
            #         for i, (g, a) in enumerate(zip(original_goal_6, actual)):
            #             if abs(g - a) >= tol:
            #                 i_to_goali = [0, 1, 1, 2, 3, 4]
            #                 goal[i_to_goali[i]] = g + (g - a) * 2
            #                 if i == 2:
            #                     goal[1] = -goal[1]
            #                 print(f'Warning: Motor {i} appears to be stuck.  Overriding setpoint from {g} to {goal[i_to_goali[i]]}')
            # actual_prev = actual.copy()
            # Now command the new setpoint
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

    def execute_joint_path(self, path: list[JointPathWaypoint], verbosity=0, default_speed=25):
        callback = None
        print_debug = lambda *args, **kwargs: None
        if verbosity > 0:
            print_debug = lambda *args, **kwargs: print(*args, **kwargs)
        if verbosity > 1:
            callback = lambda: print(f'          {self.joint_angles_string():80}', end='\r')

        self.write_all(Arm.TORQUE_ENABLE, 1)
        for i, wp in enumerate(path):
            print_debug(f'Going to {[int(q) for q in wp.q]} ({i+1}/{len(path)})')
            if self.go_to_blocking(wp.q,
                                   tol=wp.tol,
                                   timeout=wp.timeout,
                                   cb=callback,
                                   speeds=wp.speeds,
                                   default_speed=default_speed):
                print_debug("SUCCESS!")
            else:
                print_debug("\nTimeout...  Moving on")
            time.sleep(wp.pause)

    def execute_pose_path(self, )
