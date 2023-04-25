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
import numpy as np


@dataclasses.dataclass
class JointPathWaypoint:
    q: list[float]  # The joint angles to go to (in degrees)
    tol: float = 5  # The tolerance for checking when the robot has reached the goal
    timeout: float = None  # The timeout: if the robot hasn't reached the goal after this time, move on anyway
    pause: float = 0  # The time to pause after reaching the goal
    speeds: Optional[list[float]] = None  # Speeds to use for each joint (in counts)


@dataclasses.dataclass
class PosePathWaypoint:
    T: SE3  # The pose to go to
    tol: float = 5  # TODO: change this to position tolerance instead of joint angle
    timeout: float = None  # The timeout: if the robot hasn't reached the goal after this time, move on anyway
    pause: float = 0  # The time to pause after reaching the goal
    speeds: Optional[list[float]] = None  # Speeds to use for each joint (in counts)


class Arm(AX12s):
    CANVAS_CENTER = np.array([-0.03205, 0.147, 0.2493])

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.DO_PAINT_PATH = self.canvases2joints([[0, 0.05, 0]], use_start_q=False)

    def cur_pose(self, q=None):
        return kinematics.fk_deg(self.joint_angles_deg() if q is None else q)[0]

    def cur_point(self, q=None):
        return self.cur_pose(q=q)[:3, 3]

    def __str__(self):
        q = np.array(self.joint_angles_deg())
        p = self.cur_point(q=q)
        return f'Arm: tip_pos={str(p.round(3)):25}   q={str(q.round(0)):30}'

    # Check if the robot is at the goal position
    def reached_goal(self, goal, tol=5, angles_out=None):
        if len(goal) == 5:
            goal = [goal[0], goal[1], -goal[1], *goal[2:]]
        elif len(goal) != 6:
            raise ValueError('goal must be length 5 or 6')
        try:
            actual = self.read_all_joint_angles_deg()
        except AssertionError as e:
            self.warn('warning: assertion error in reached_goal: ', e)
            return False
        if angles_out is not None:
            angles_out[:] = actual
        return all([abs(g - a) < tol for g, a in zip(goal, actual)])

    ############################ GO TO ############################

    # Go to setpoint joint configuration, in degrees
    def go_to_blocking(self,
                       goal,
                       tol=5,
                       timeout=None,
                       cb=None,
                       verbosity=0,
                       speeds=None,
                       default_speed=25,
                       speed_use_start_q=True):
        original_goal = list(goal)
        original_goal_6 = [goal[0], goal[1], -goal[1], *goal[2:]]
        if tol is None:
            tol = 5
        if speeds is None and speed_use_start_q:
            start_q = self.joint_angles_deg()
            speeds = kinematics.joint_vels_deg(start_q, original_goal, max_vel=default_speed)
        if speeds is not None:
            speeds = [speeds[0], speeds[1], speeds[1], *speeds[2:]]
            if verbosity > 0:
                print(f'  speeds: {speeds}')
            self.set_speeds(speeds)
        else:
            self.set_speed(default_speed)
        print(goal.round(0))
        self.command_angles_deg(*goal)
        tstart = time.time()
        actual_prev = [0, 0, 0, 0, 0, 0]
        actual = [0, 0, 0, 0, 0, 0]
        if verbosity > 0:
            print('Going to', goal)
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
            if verbosity == 2:
                print('  ', self, end='\r')
            if verbosity > 2:
                print('  ', self)
            if cb is not None:
                try:
                    cb()
                except AssertionError as e:
                    self.warn('warning: callback had an assertion error in go_to_blocking: ', e)
            time.sleep(0.1)
            if timeout is not None and time.time() - tstart > timeout:
                return False
        return True

    def go_to_pose_blocking(self, goal: SE3, elbow_mode='neg', ik_params={}, **go_to_kwargs):
        """Go to a pose"""
        return self.go_to_blocking(self.pose2joint(goal, elbow_mode=elbow_mode, **ik_params),
                                   **go_to_kwargs)

    def go_to_canvas_blocking(
        self,
        goal: list[float],
        center=CANVAS_CENTER,
        angle=-np.pi / 2,
        elbow_mode='neg',
        ik_params={},
        **go_to_kwargs,
    ):
        """Go to a position relative to canvas center"""
        return self.go_to_blocking(
            self.canvas2joint(goal, center=center, angle=angle, elbow_mode=elbow_mode, **ik_params),
            **go_to_kwargs)

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

    def execute_pose_path(self,
                          path: list[PosePathWaypoint],
                          verbosity=0,
                          default_speed=25,
                          **ik_params):
        joint_path = self.poses2joints(path, default_speed=default_speed, **ik_params)
        return self.execute_joint_path(joint_path, verbosity=verbosity, default_speed=default_speed)

    def execute_canvas_path(self,
                            centerPxyz: np.ndarray,
                            center=CANVAS_CENTER,
                            vmax=50,
                            angle=-np.pi / 2,
                            elbow_mode='neg',
                            ease_dist=0.1,
                            verbosity=0,
                            **ik_params):
        joint_path = self.canvases2joints(centerPxyz, center, vmax, angle, elbow_mode, ease_dist,
                                          **ik_params)
        return self.execute_joint_path(joint_path, verbosity=verbosity, default_speed=vmax)

    ######################### Calculation Functions #############################

    def pose2joint(self, pose: SE3, elbow_mode='neg', **ik_params):
        ok, q = kinematics.ik_deg(pose, elbow_mode=elbow_mode, **ik_params)
        assert ok, f'ik failed for goal {pose}'
        return q

    def canvas2joint(self,
                     centerPxyz: list[float],
                     center=CANVAS_CENTER,
                     angle=-np.pi / 2,
                     elbow_mode='neg',
                     **ik_params) -> list[PosePathWaypoint]:
        """
        centerPxyz: 3-array goal position in the canvas frame.  x/y/z is right, out, up
        center: 3-array of the center of the canvas in the robot frame
        angle: angle of the brush (rotating around x-axis).  Default: -pi/2
        elbow_mode: 'pos' or 'neg' for positive or negative elbow or None for random
        """
        # Turn position into global pose
        wTxyz = SE3.P(centerPxyz + center) @ SE3.RotX(angle)
        return self.pose2joint(wTxyz, elbow_mode=elbow_mode, **ik_params)

    def poses2joints(self,
                     path: list[PosePathWaypoint],
                     use_start_q=True,
                     default_speed=25,
                     **ik_params):
        Ts = [wp.T for wp in path]
        # First solve all the IK so we don't stop partway through
        qs = [kinematics.ik_deg(T, **ik_params) for T in Ts]
        assert all(q[0] for q in qs), f'IK failed: {[q[0] for q in qs]}'
        qs = [q[1] for q in qs]
        # Now, handle velocities
        qdots = kinematics.qdots_deg(qs,
                                     max_qdot=default_speed,
                                     start_q=self.joint_angles_deg() if use_start_q else None)
        # Finally, return as list of JointPathWaypoint
        return [
            JointPathWaypoint(q, speeds=qdot, tol=wp.tol, timeout=wp.timeout, pause=wp.pause)
            for q, qdot, wp in zip(qs, qdots, path)
        ]

    def canvases2joints(self,
                        centerPxyz: np.ndarray,
                        center=CANVAS_CENTER,
                        vmax=50,
                        angle=-np.pi / 2,
                        ease_dist=0.1,
                        elbow_mode='neg',
                        **ik_params) -> list[PosePathWaypoint]:
        """
        centerPxyz: Nx3 array of points in the canvas frame.  x/y/z is right, out, up
        center: 3-array of the center of the canvas in the robot frame
        vmax: maximum joint velocity in counts or something, idk the unit honestly
        angle: angle of the brush (rotating around x-axis).  Default: -pi/2
        ease_dist: distance in meters to ease in/out at the beginning and end of the path
        elbow_mode: 'pos' or 'neg' for positive or negative elbow or None for random
        """
        # First, add the ease in/out
        ease_point = np.array([0, -ease_dist, 0.0])
        centerPxyz = np.vstack(
            [ease_point + centerPxyz[0], centerPxyz, ease_point + centerPxyz[-1]])
        # Now turn positions into global poses
        wTxyz = [(SE3.P(p + center) @ SE3.RotX(angle)) for p in centerPxyz]
        # Create pose waypoints
        pose_waypoints = [PosePathWaypoint(T=T, tol=5, timeout=None, pause=0) for T in wTxyz]
        # Finally, convert to joint angles
        return self.poses2joints(pose_waypoints,
                                 default_speed=vmax,
                                 elbow_mode=elbow_mode,
                                 **ik_params)

    # Predefined Trajectories
    def do_move_home(self, **go_to_kwargs):
        self.go_to_blocking([0, 0, 0, 0, 0], **go_to_kwargs)

    def do_dip(
        self,
        prep_qs_deg=[20.08, -23.90, 83.72, 3.66, 100.73],
        hover_qs_deg=[22.72, -28.00, 106.30, 3.37, 86.95],
        dip_qs_deg=[22.14, -23.90, 115.10, 3.37, 79.61],
        rub1_qs_deg=[23.02, -26.53, 114.51, 3.37, 85.77],
        rub2_qs_deg=[22.14, -12.75, 104.83, 3.66, 75.51],
        verbosity=0,
        speed=50,
    ):
        path = [
            JointPathWaypoint(q=[0, 0, 0, 0, 0], tol=5, timeout=None, pause=0),  # home
            JointPathWaypoint(q=prep_qs_deg, tol=5, timeout=None, pause=0),  # lower
            JointPathWaypoint(q=hover_qs_deg, tol=5, timeout=None, pause=0),  # hover paint
            JointPathWaypoint(q=dip_qs_deg, tol=5, timeout=None, pause=0),  # dip
            JointPathWaypoint(q=rub1_qs_deg, tol=5, timeout=None, pause=0),  # rub
            JointPathWaypoint(q=rub2_qs_deg, tol=5, timeout=None, pause=0),  # rub
            JointPathWaypoint(q=hover_qs_deg, tol=5, timeout=None, pause=0),  # lift paint
            JointPathWaypoint(q=prep_qs_deg, tol=5, timeout=None, pause=0),  # raise
            JointPathWaypoint(q=[0, 0, 0, 0, 0], tol=5, timeout=None, pause=0),  # home
        ]
        self.execute_joint_path(path, verbosity=verbosity, default_speed=speed)

    def do_prep_paint(self, center=CANVAS_CENTER, ease_dist=0.1, elbow_mode='neg', vmax=50):
        return self.go_to_canvas_blocking([0, -ease_dist, 0],
                                          center=center,
                                          elbow_mode=elbow_mode,
                                          default_speed=vmax)

    def do_start_paint(self, center=CANVAS_CENTER, angle=-np.pi / 2, elbow_mode='neg', vmax=50):
        return self.go_to_canvas_blocking([0, 0, 0],
                                          center=center,
                                          angle=angle,
                                          elbow_mode=elbow_mode,
                                          default_speed=vmax)
