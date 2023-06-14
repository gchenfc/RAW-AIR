'''server.py
Main server for real-time SLN demo.
* Receives drawing inputs, sends to SLN regression (TODO(gerry))
* Receives SLN regression results, sends to cable robot controller (TODO(gerry))

run with:
    watchmedo auto-restart --pattern "*.py" --recursive --signal SIGTERM python server.py

@author Gerry Chen
'''
import asyncio
import aiofiles
import socket
import websockets
import time
import datetime
import struct
from pathlib import Path

PORTS = {
    'whiteboard_input': 5900,
    'whiteboard_fit': 5901,
    'fit_input': 5902,
    'fit_output': 5903,
    'robot_input': 5904,
    'robot_output': 5905,
    'whiteboard_passthrough': 5906
}
SAVE_FOLDER = None

COLORS = {
    '#000000': 0, # black
    '#ff0000': 1, # red
    '#ffff00': 2, # yellow
    '#008000': 3, # green
    '#0000ff': 4, # blue
    '#800080': 5  # purple
}

def get_local_ip():
    # return socket.gethostbyname(socket.gethostname())
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(("8.8.8.8", 80))
    return s.getsockname()[0]


HOST = get_local_ip()

clients = {'whiteboard': set(), 'fit': set(), 'robot_input': set(), 'whiteboard_passthrough': set()}
frame_msg = ''


def parse(msg):
    t, x, y = msg[1:].split(',')
    t, x, y = float(t), float(x), float(y)
    return msg[0], t, x, y


def create_log_file():
    if SAVE_FOLDER is None:
        return '/dev/null'
    now = datetime.datetime.now().strftime('%Y-%m-%d-%H%M%S')
    return SAVE_FOLDER / f'{now}.txt'

async def handle_whiteboard(websocket):
    # async with aiofiles.open(create_log_file(), 'w') as f:
    print('Whiteboard connection opened!')
    s = time.perf_counter()
    t = s
    try:
        clients['whiteboard'].add(websocket)
        print(frame_msg)
        await websocket.send(frame_msg)
        # Not sure why opening as 'w' causes some weird bug with websocket
        #   https://stackoverflow.com/q/70811731/9151520
        # Open with 'a' instead, and the first connection will always fail, just wait for the second
        with open(create_log_file(), 'a') as f:
            while True:
                msg = await websocket.recv()
                prev_t, t = t, time.perf_counter()
                print(f'{t - s:.2f}s', f'{1/(t-prev_t):.0f}Hz', msg)
                if msg[0] == 'C':
                    t_ipad, curr_color, prev_color = msg[1:].split(',')
                    print(curr_color, COLORS[curr_color])
                    c, *data = float(t_ipad), COLORS[curr_color], COLORS[prev_color]
                else:
                    c, *data = parse(msg)
                f.write(','.join('{}'.format(n) for n in [t, c, *data]) + '\n')
                for writer in clients['fit']:
                    writer.write(c.encode() + struct.pack('fff', *data))
                    # await writer.drain()
                for sock in clients['whiteboard_passthrough']:
                    await sock.send(msg)
    except websockets.exceptions.ConnectionClosed as e:
        print('Whiteboard connection closed!', e)
    finally:
        await websocket.close()
        clients['whiteboard'].remove(websocket)


async def whiteboard_server():
    print(f'Serving whiteboard at: {HOST}:{PORTS["whiteboard_input"]}')
    async with websockets.serve(handle_whiteboard, HOST, PORTS['whiteboard_input']):
        await asyncio.Future()
    print('Closed whiteboard server')


async def handle_fit(reader, writer):
    print('Fit connection opened!')
    try:
        clients['fit'].add(writer)
        while not reader.at_eof():
            data = await reader.read(9)
            c = data[0:1].decode()
            try:
                message = struct.unpack('ff', data[1:])
            except struct.error:
                print('Parse Error')
                continue
            addr = writer.get_extra_info('peername')
            print(f"Received {c}{message} from {addr!r}")
            for client in clients['whiteboard']:
                await client.send(f'{c}{message[0]},{message[1]}')
            for sock in clients['robot_input']:
                await sock.send(f'{c}0,{message[0]},{message[1]}')
        print('Fit connection closed due to EOF!')

    except websockets.exceptions.ConnectionClosed:
        print('Fit connection closed!')
    finally:
        clients['fit'].remove(writer)


async def fit_server():
    server = await asyncio.start_server(handle_fit, 'localhost', PORTS['fit_input'])

    addrs = ', '.join(str(sock.getsockname()) for sock in server.sockets)
    print(f'Serving fit server at {addrs}')

    async with server:
        await server.serve_forever()

    print('Closed fit server')


async def handle_robot_(websocket, client_type):
    print('Robot connection opened!')
    try:
        clients[client_type].add(websocket)
        while True:
            msg = await websocket.recv()
            print('message from robot:', msg)
            global frame_msg
            frame_msg = msg
            for sock in clients['whiteboard']:
                await sock.send(msg)
    except websockets.exceptions.ConnectionClosed:
        print('Robot connection closed!')
    finally:
        websocket.close()
        clients[client_type].remove(websocket)


async def robot_server(client_type):
    handle_robot = lambda websocket: handle_robot_(websocket, client_type)
    print(f'Serving robot at: localhost:{PORTS[client_type]}')
    async with websockets.serve(handle_robot, 'localhost', PORTS[client_type]):
        await asyncio.Future()
    print('Closed robot server')


async def main():
    await asyncio.wait([
        asyncio.create_task(whiteboard_server()),
        asyncio.create_task(fit_server()),
        asyncio.create_task(robot_server('robot_input')),
        asyncio.create_task(robot_server('whiteboard_passthrough')),
    ])


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(prog='Graffiti Whiteboard Server')
    parser.add_argument('-s',
                        '--save',
                        type=str,
                        default=None,
                        help='Folder name to save the whiteboard data to')
    args = parser.parse_args()
    if args.save is not None:
        SAVE_FOLDER = Path(args.save)

    print("STARTING UP!!!")
    asyncio.get_event_loop().run_until_complete(main())
