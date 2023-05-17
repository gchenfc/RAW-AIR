"""
@file arm_server.py
@brief This file contains the code for the RPC server that allows remote control of the arm.
@details To control the robot over a websocket, first run this file (in the background), then open
         a connection to this server (localhost:5910) and send a string-ified JSON object with a:
          * "method": the name of the RPC method to call
          * "params": the parameters to pass to the method, which in turn contains:
            * "args": the positional arguments to pass to the method
            * "kwargs": the keyword arguments to pass to the method
         For example, check the cable-robot repo, file arm.js
@author Gerry (w/ ChatGPT)
@date 2023-04-24
"""

import asyncio
import json
from json import JSONEncoder
import websockets

from arm import Arm, JointPathWaypoint
from dynamixel import StatusParams, Status
import kinematics
import time
import numpy as np
from util import SE3

HOST = "localhost"
PORT = 5910

# Serial settings, may get overwritten by rpc calls
default_port = '/dev/tty.usbmodem103568505'
default_baud = 1000000
default_write_packet_delay = 0.01

class MyEncoder(JSONEncoder):
    def default(self, o):
        if isinstance(o, StatusParams):
            return o.value()
        elif isinstance(o, Status):
            return o.__dict__
        elif isinstance(o, np.ndarray):
            return o.tolist()
        return JSONEncoder.default(self, o)

async def rpc_server(websocket, path):
    print('rpc_server: connected!!!!')
    with Arm(default_port, default_baud, write_packet_delay=default_write_packet_delay) as arm:
        while True:
            data = await websocket.recv()
            print(f"Received message: {data}")
            # Assume the data received is a JSON object with a "method" property
            # representing the name of the RPC method to call and a "params"
            # property representing the parameters to pass to the method.
            request = json.loads(data)
            method_name = request["method"]
            method_params = request["params"]
            # Call the RPC method and get the result or error
            try:
                result = await rpc_call(arm, method_name, *method_params.get("args", []), **method_params.get("kwargs", {}))
                response = json.dumps({"result": result}, cls=MyEncoder)
            except Exception as e:
                # If an exception is thrown, return an error object instead of a result object
                error_message = str(e)
                response = json.dumps({"error": error_message}, cls=MyEncoder)
            # Send the result or error back to the client
            await websocket.send(response)

async def rpc_call(arm: Arm, method_name, *method_params, **method_kwargs):
    # Implement your RPC method here
    # For example, if you have a method called "add" that takes two parameters,
    # you can implement it like this:
    if method_name == "add":
        offset = method_kwargs.get('offset', 0)
        result = method_params[0] + method_params[1] + offset

    # Arm functions
    elif method_name == 'arm.reached_goal':
        result = arm.reached_goal(*method_params, **method_kwargs)
    elif method_name == 'arm.go_to_blocking':
        result = arm.go_to_blocking(*method_params, **method_kwargs)
    elif method_name == 'arm.go_to_pose_blocking':
        result = arm.go_to_pose_blocking(*method_params, **method_kwargs)
    elif method_name == 'arm.go_to_canvas_blocking':
        result = arm.go_to_canvas_blocking(*method_params, **method_kwargs)
    elif method_name == 'arm.execute_joint_path':
        result = arm.execute_joint_path(*method_params, **method_kwargs)
    elif method_name == 'arm.do_move_home':
        result = arm.do_move_home(*method_params, **method_kwargs)
    elif method_name == 'arm.do_move_storage':
        result = arm.do_move_storage(*method_params, **method_kwargs)
    elif method_name == 'arm.do_dip':
        result = arm.do_dip(*method_params, **method_kwargs)
    elif method_name == 'arm.do_prep_paint':
        result = arm.do_prep_paint(*method_params, **method_kwargs)
    elif method_name == 'arm.do_start_paint':
        result = arm.do_start_paint(*method_params, **method_kwargs)
    elif method_name == 'arm.cur_pose':
        result = arm.cur_pose(*method_params, **method_kwargs)
    elif method_name == 'arm.cur_point':
        result = arm.cur_point(*method_params, **method_kwargs)
    elif method_name == 'arm.cur_canvas_pose':
        result = arm.cur_canvas_pose(*method_params, **method_kwargs)
    elif method_name == 'arm.cur_canvas_point':
        result = arm.cur_canvas_point(*method_params, **method_kwargs)

    # AX12s functions
    elif method_name == 'arm.write_all':
        result = arm.write_all(*method_params, **method_kwargs)
    elif method_name == 'arm.read_all':
        result = arm.read_all(*method_params, **method_kwargs)
    elif method_name == 'arm.enable_all':
        result = arm.enable_all(*method_params, **method_kwargs)
    elif method_name == 'arm.disable_all':
        result = arm.disable_all(*method_params, **method_kwargs)
    elif method_name == 'arm.set_speed':
        result = arm.set_speed(*method_params, **method_kwargs)
    elif method_name == 'arm.set_speeds':
        result = arm.set_speeds(*method_params, **method_kwargs)
    elif method_name == 'arm.set_compliance_margins':
        result = arm.set_compliance_margins(*method_params, **method_kwargs)
    elif method_name == 'arm.set_compliance_slopes':
        result = arm.set_compliance_slopes(*method_params, **method_kwargs)
    elif method_name == 'arm.read_all_joint_angles_deg':
        result = arm.read_all_joint_angles_deg(*method_params, **method_kwargs)
    elif method_name == 'arm.joint_angles_deg':
        result = arm.joint_angles_deg(*method_params, **method_kwargs)
    elif method_name == 'arm.joint_angles_string':
        result = arm.joint_angles_string(*method_params, **method_kwargs)
    elif method_name == 'arm.command_angle':
        result = arm.command_angle(*method_params, **method_kwargs)
    elif method_name == 'arm.command_angles_deg':
        result = arm.command_angles_deg(*method_params, **method_kwargs)

    else:
        # If the method name is not recognized, raise an exception with an error message
        raise ValueError(f"Unknown method: {method_name}")
    return result

async def main():
    async with websockets.serve(rpc_server, HOST, PORT):
        await asyncio.Future()  # run forever

asyncio.run(main())
