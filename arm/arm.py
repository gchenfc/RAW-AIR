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

from dynamixel import AX12s, Byte, Data, JOINT_FLIPS
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
    # CANVAS_CENTER = np.array([-0.03205, 0.19, 0.2493])  # Klaus
    # CANVAS_CENTER = np.array([-0.03205, 0.207, 0.2493])  # Mobile Frame (DFL)
    # CANVAS_CENTER = np.array([-0.03205, 0.33, 0.2493])  # Library
    CANVAS_CENTER = np.array([-0.03205, 0.22, 0.2893])  # Library

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.DO_PAINT_PATH = self.canvases2joints([[0, 0.05, 0]], use_start_q=False)

    def cur_pose(self, q=None):
        return kinematics.fk_deg(self.joint_angles_deg() if q is None else q)[0]

    def cur_point(self, q=None):
        return self.cur_pose(q=q)[:3, 3]

    def cur_canvas_pose(self, q=None, center=CANVAS_CENTER):
        # THIS FUNCTION IS UNTESTED
        wTxyz = self.cur_pose(q=q)
        centerTxyz = wTxyz @ SE3.RotX(np.pi / 2) @ SE3.P(-center)
        # assert self.canvas2joint(centerTxyz, center=center, angle=-np.pi / 2) == q
        return centerTxyz

    def cur_canvas_point(self, q=None, center=CANVAS_CENTER):
        return self.cur_canvas_pose(q=q, center=center)[:3, 3]

    def __str__(self):
        q = np.array(self.joint_angles_deg())
        p = self.cur_point(q=q)
        return f'Arm: tip_pos={str(p.round(3)):25}   q={str(q.round(0)):30}'

    # Check if the robot is at the goal position
    def reached_goal(self, goal, tol=5, angles_in=None, angles_out=None):
        if len(goal) == 5:
            goal = [f * a for f, a in zip(JOINT_FLIPS, goal)]
            goal = [goal[0], goal[1], -goal[1], *goal[2:]]
        elif len(goal) != 6:
            raise ValueError('goal must be length 5 or 6')
        if angles_in is None:
            try:
                actual = self.read_all_joint_angles_deg()
            except AssertionError as e:
                self.warn('warning: assertion error in reached_goal: ', e)
                return False
        else:
            actual = angles_in
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
        return True

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
    HOME = [0, 0, 0, 0, 0]
    STORAGE = [90, 100, -83, 0, -130]
    # STORAGE_INTERMEDIATE = [90, 75, -81,  0, -109]  # Between HOME and STORAGE to dodge AprilTag
    STORAGE_INTERMEDIATE = [90, 75, -90,  0, 0]  # Between HOME and STORAGE to dodge AprilTag
    # PREP_INTERMEDIATE = [-90, 65, -38, 0, -117]
    PREP_INTERMEDIATE = [[-90, 65, -90, 0, 0], [0, 65, -38, 0, -90]]
    # PREP_TO_DIP = [[0, 65, -38, 0, -90]]
    # PREP_TO_DIP = [[-61,  72, -47,  -1, -123], [-132,  27, -56,  -1, -124]]
    # PREP_TO_DIP = [[-61,  72, -52,  -1, -118], [-132,  27, -56,  -1, -124]]
    PREP_TO_DIP = [[-61,  72, -52,  -1, -108]]#, [-132,  27, -56,  -1, -124]]
    # PREP_TO_DIP = [[0,  67, -38,  -1, -75], [-69,  84, -79,  -1, -75], [-132,  27, -56,  -1, -124]]

    def current_configuration(self):
        q = self.read_all_joint_angles_deg()
        q2 = self.joint_angles_deg(all_joint_angles_deg=q)
        if self.reached_goal(self.HOME, tol=15, angles_in=q):
            ret = 'HOME'
        elif self.reached_goal(self.STORAGE, tol=15, angles_in=q):
            ret = 'STORAGE'
        elif np.linalg.norm(self.cur_canvas_point(q=q2)) < 0.10:
            ret = 'PAINT'
        elif np.linalg.norm(self.cur_canvas_point(q=q2) - np.array([0, -0.22, 0])) < 0.10:
            ret = 'PREP_PAINT'
        else:
            ret = 'UNKNOWN'
        print("CURRENT CONFIGURATION IS ", ret)
        return ret

    def do_move_home(self, vmax=100, **go_to_kwargs):
        def prep():
            config = self.current_configuration()
            if config == 'STORAGE':
                return (#self.go_to_blocking([90, 90, -71, 0, -30],
                         #                   default_speed=vmax,
                          #                  **go_to_kwargs) and
                        self.go_to_blocking(Arm.STORAGE_INTERMEDIATE,
                                            default_speed=vmax,
                                            **go_to_kwargs))
            elif config == 'PAINT':
                return self.do_prep_paint() and prep()
            elif config == 'PREP_PAINT':
                return all(
                    self.go_to_blocking(q, default_speed=vmax, **go_to_kwargs)
                    for q in reversed(Arm.PREP_INTERMEDIATE))
            return True

        return (prep() and
                self.go_to_blocking([0, 0, 0, 0, 0], default_speed=vmax, **go_to_kwargs))

    def do_move_storage(self, vmax=100, **go_to_kwargs):
        def prep():
            config = self.current_configuration()
            if config == 'STORAGE':
                return True
            elif config != 'HOME':
                return self.do_move_home() and prep()
            elif config == 'HOME':
                return self.go_to_blocking(Arm.STORAGE_INTERMEDIATE,
                                           default_speed=vmax,
                                           **go_to_kwargs)
            return False

        return (prep() and
                self.go_to_blocking(Arm.STORAGE, default_speed=vmax, **go_to_kwargs) and
                self.disable_all())

    def do_dip(
        self,
        prep_qs_deg=[20.08, -23.90, 83.72, 3.66, 100.73],
        # hover_qs_deg=[22.72, -28.00, 106.30, 3.37, 86.95],
        # dip_qs_deg=[22.14, -23.90, 115.10, 3.37, 79.61],
        # rub1_qs_deg=[23.02, -26.53, 114.51, 3.37, 85.77],
        # rub2_qs_deg=[22.14, -12.75, 104.83, 3.66, 75.51],
        hover_qs_deg=[22.72, -28.00 + 0, 106.30 - 0, 3.37, 86.95 - 4],
        dip_qs_deg=[22.14, -23.90 + 0, 115.10 - 0, 3.37, 79.61 - 4],
        rub1_qs_deg=[23.02, -26.53 + 0, 114.51 - 0, 3.37, 85.77 - 4],
        rub2_qs_deg=[22.14, -12.75 + 0, 104.83 - 0, 3.66, 75.51 - 4],
        verbosity=0,
        vmax=100,
    ):
        if self.current_configuration() == 'PAINT':
            assert self.do_prep_paint()
        if self.current_configuration() == 'PREP_PAINT':
            path = [*[JointPathWaypoint(q=q, tol=5, timeout=None, pause=0) for q in Arm.PREP_TO_DIP],

                    JointPathWaypoint(q=[-146,  20, -49,  -1, -122], tol=5, timeout=None, pause=0),  # hover paint
                    JointPathWaypoint(q=[-146, -28, -55,  -1, -101], tol=5, timeout=None, pause=0),  # dip
                    # JointPathWaypoint(q=[-133,   8, -88,  -1, -85], tol=5, timeout=None, pause=0),  # swipe paint
                    # JointPathWaypoint(q=[-128,  24, -92,  -1, -86], tol=5, timeout=None, pause=0),  # swipe paint
                    # JointPathWaypoint(q=[-120,  32, -93,  -1, -87], tol=5, timeout=None, pause=0),  # swipe paint
                    JointPathWaypoint(q=[-146,  20, -49,  -1, -122], tol=5, timeout=None, pause=0),  # hover paint

                    *[JointPathWaypoint(q=q, tol=5, timeout=None, pause=0) for q in reversed(Arm.PREP_TO_DIP)]
                    ]
            return self.execute_joint_path(path, verbosity=verbosity, default_speed=vmax) and self.do_prep_paint()
        if self.current_configuration() != 'HOME':
            assert self.do_move_home()
        # path = [
        #     JointPathWaypoint(q=[0, 0, 0, 0, 0], tol=5, timeout=None, pause=0),  # home
        #     JointPathWaypoint(q=prep_qs_deg, tol=5, timeout=None, pause=0),  # lower
        #     JointPathWaypoint(q=hover_qs_deg, tol=5, timeout=None, pause=0),  # hover paint
        #     JointPathWaypoint(q=dip_qs_deg, tol=5, timeout=None, pause=0),  # dip
        #     JointPathWaypoint(q=rub1_qs_deg, tol=5, timeout=None, pause=0),  # rub
        #     JointPathWaypoint(q=rub2_qs_deg, tol=5, timeout=None, pause=0),  # rub
        #     JointPathWaypoint(q=hover_qs_deg, tol=5, timeout=None, pause=0),  # lift paint
        #     JointPathWaypoint(q=prep_qs_deg, tol=5, timeout=None, pause=0),  # raise
        #     JointPathWaypoint(q=[0, 0, 0, 0, 0], tol=5, timeout=None, pause=0),  # home
        # ]
        #   0   0   0   0   0  # Home
        #  86  84 -82  -0 -98  # Lower
        #  73  34 -59  -0 -124 # Hover paint
        #  67  14 -56  -0 -121 # Dip
        #  61  27 -73  -0 -114 # Rub
        #  67  32 -79  -0 -104 # Rub
        #  77  42 -88  -0 -106 # Rub
        #  68  25 -61  -0 -126 # Rub
        #  68  24 -47  -0 -136 # Swiping...
        #  73  34 -59  -0 -124 # Swipe...
        #  61  65 -69  -0 -127 # Raise
        #   0   0   0   0   0  # Home
        # path = [
        #     JointPathWaypoint(q=[0, 0, 0, 0, 0], tol=5, timeout=None, pause=0),  # home
        #     JointPathWaypoint(q=[90, 0, 0, 0, 0], tol=5, timeout=None, pause=0, speeds=[300, 0, 0, 0, 0]),  # rot
        #     JointPathWaypoint(q=[86, 84, -70, 0, -98], tol=5, timeout=None, pause=0),  # lower
        #     JointPathWaypoint(q=[73, 34, -59, 0, -124], tol=5, timeout=None, pause=0),  # hover paint
        #     JointPathWaypoint(q=[67, 14, -56, 0, -121], tol=5, timeout=None, pause=0),  # dip
        #     JointPathWaypoint(q=[61, 27, -73, 0, -114], tol=5, timeout=None, pause=0),  # rub
        #     JointPathWaypoint(q=[67, 32, -79, 0, -104], tol=5, timeout=None, pause=0),  # rub
        #     JointPathWaypoint(q=[77, 42, -88, 0, -106], tol=5, timeout=None, pause=0),  # rub
        #     JointPathWaypoint(q=[68, 25, -61, 0, -126], tol=5, timeout=None, pause=0),  # rub
        #     JointPathWaypoint(q=[68, 24, -47, 0, -136], tol=5, timeout=None, pause=0),  # swiping...
        #     # JointPathWaypoint(q=[73, 34, -59, 0, -124], tol=5, timeout=None, pause=0),  # swipe...
        #     JointPathWaypoint(q=[61, 65, -69, 0, -127], tol=5, timeout=None, pause=0),  # raise
        #     JointPathWaypoint(q=[86, 84, -70, 0, -98], tol=5, timeout=None, pause=0),  # raise
        #     JointPathWaypoint(q=[90, 0, 0, 0, 0], tol=5, timeout=None, pause=0),  # rot
        #     JointPathWaypoint(q=[0, 0, 0, 0, 0], tol=5, timeout=None, pause=0, speeds=[300, 0, 0, 0, 0]),  # home
        # ]
        path = [
            JointPathWaypoint(q=[0, 0, 0, 0, 0], tol=5, timeout=None, pause=0),  # home
            JointPathWaypoint(q=[-91, 0, 0, 0, 0], tol=5, timeout=None, pause=0, speeds=[250, 0, 0, 0, 0]),  # rot
            JointPathWaypoint(q=[-146,  26, -46,  -1, -27], tol=5, timeout=None, pause=0),  # lower
            JointPathWaypoint(q=[-146,  20, -49,  -1, -122], tol=5, timeout=None, pause=0),  # hover paint

            JointPathWaypoint(q=[-146, -28, -55,  -1, -101], tol=5, timeout=None, pause=0),  # dip

            # JointPathWaypoint(q=[61, 27, -73, 0, -114], tol=5, timeout=None, pause=0),  # rub
            # JointPathWaypoint(q=[67, 32, -79, 0, -104], tol=5, timeout=None, pause=0),  # rub
            # JointPathWaypoint(q=[77, 42, -88, 0, -106], tol=5, timeout=None, pause=0),  # rub
            # JointPathWaypoint(q=[68, 25, -61, 0, -126], tol=5, timeout=None, pause=0),  # rub

            # JointPathWaypoint(q=[68, 24, -47, 0, -136], tol=5, timeout=None, pause=0),  # swiping...
            # JointPathWaypoint(q=[73, 34, -59, 0, -124], tol=5, timeout=None, pause=0),  # swipe...

            # JointPathWaypoint(q=[-142,  23, -105,  -1, -77], tol=5, timeout=None, pause=0),  # swipe paint
            # JointPathWaypoint(q=[-142,  30, -99,  -1, -84], tol=5, timeout=None, pause=0),  # swipe paint


            # JointPathWaypoint(q=[-133,   8, -88,  -1, -85], tol=5, timeout=None, pause=0),  # swipe paint
            # JointPathWaypoint(q=[-128,  24, -92,  -1, -86], tol=5, timeout=None, pause=0),  # swipe paint
            # JointPathWaypoint(q=[-120,  32, -93,  -1, -87], tol=5, timeout=None, pause=0),  # swipe paint


            # JointPathWaypoint(q=[-120,  21, -95,  -1, -79], tol=5, timeout=None, pause=0),  # swipe paint
            # JointPathWaypoint(q=[-150,  20, -90,  -1, -90], tol=5, timeout=None, pause=0),  # swipe paint
            # JointPathWaypoint(q=[-150,  27, -87,  -1, -100], tol=5, timeout=None, pause=0),  # swipe paint

            JointPathWaypoint(q=[-146,  20, -49,  -1, -122], tol=5, timeout=None, pause=0),  # hover paint
            JointPathWaypoint(q=[-146,  26, -26,  -1, -27], tol=5, timeout=None, pause=0),  # lower
            JointPathWaypoint(q=[-91, 0, 0, 0, 0], tol=5, timeout=None, pause=0),  # rot

            JointPathWaypoint(q=[0, 0, 0, 0, 0], tol=5, timeout=None, pause=0, speeds=[250, 0, 0, 0, 0]),  # home
        ]
        self.execute_joint_path(path, verbosity=verbosity, default_speed=vmax)

    def do_prep_paint(self, center=CANVAS_CENTER, ease_dist=0.22, elbow_mode='neg', vmax=100):
        kwargs = dict(center=center, elbow_mode=elbow_mode, default_speed=vmax)
        def prep():
            config = self.current_configuration()
            if config == 'PAINT':
                return self.go_to_canvas_blocking([0, -0.05, 0.04], angle=-1.45, **kwargs)
            elif config == 'STORAGE':
                return self.do_move_home()
            elif config == 'HOME':
                return all(
                    self.go_to_blocking(q, default_speed=vmax) for q in Arm.PREP_INTERMEDIATE)
            return True

        return (prep() and self.go_to_canvas_blocking([0, -ease_dist, 0], **kwargs))

    def do_start_paint(self, center=CANVAS_CENTER, angle=-np.pi / 2, elbow_mode='neg', vmax=100):
        kwargs = dict(center=center, angle=angle, elbow_mode=elbow_mode, default_speed=vmax)

        def go_to_canvas_alt(goal, center=center, angle=-np.pi / 2, elbow_mode='neg', ik_params={}, **kwargs):
            j = self.canvas2joint(goal, center=center, angle=angle, elbow_mode=elbow_mode)
            assert abs(j[0]) < 1e-1, "FAILED, j[0] != 0, " + str(j)
            j[0] = -5
            return self.go_to_blocking(j, **kwargs)

        def prep():
            config = self.current_configuration()
            if config == 'PREP_PAINT':
                kwargs2 = {k:v for k,v in kwargs.items() if k != 'angle'}
                # return self.go_to_canvas_blocking([0, -0.05, 0.05], angle=-np.pi/2 + 0.2, tol=2, **kwargs2)
                # assert go_to_canvas_alt([0, -0.05, 0.04], angle=-np.pi/2 + 0.2, tol=5, **kwargs2)
                assert go_to_canvas_alt([0, -0.05, 0.03], angle=-np.pi/2 + 0.2, tol=5, **kwargs2)
                # self.set_compliance_slopes(8)
                # # time.sleep(0.5)
                # # self.set_compliance_slopes(1)
                # assert go_to_canvas_alt([0, -0.05, 0.03], angle=-np.pi/2, tol=2, **kwargs2)
                # self.set_compliance_slopes(32)
                return True
            if config == 'PAINT':
                return True
            else:
                return self.do_prep_paint()

        return (prep() and self.go_to_canvas_blocking([0, 0, 0], **kwargs))
