"""
Script for running the main processes and managing nodes, in other words starting and stopping different parts of the system
"""


import zmq
import os
import sys
import asyncio
import zmq.asyncio
import json
from config import *

# Tells a node it is running under this supervisor, which in turn is running
# under update.py. nodes/io_manager/selfupdate.py checks it before signalling the
# process group to restart everything onto freshly pulled code - run a node by
# hand and it must not take down a process group it does not own.
NODE_ENV = {**os.environ, "LIGMAX_SUPERVISED": "1"}

NODES = [
    # nodes/logging/main.py is an empty stub - not implemented yet. Leave it
    # off, since a node that exits immediately would otherwise respawn as
    # fast as start_node()'s loop allows.
    {"name": "logging", "cmd": [sys.executable, "-m", "nodes.logging.main"], "on":False},
    {"name": "io_manager", "cmd": [sys.executable, "-m", "nodes.io_manager.main"], "on":True},
    {"name": "balancing", "cmd": [sys.executable, "-m", "nodes.balancing.main"], "on":False},
    # The autonomy node. On from boot, and safe to be: it commands nothing until
    # an operator sends `autopilot_start`. Running it all the time is what has
    # it warmed up - lidar tracked, plan restored, Jetson connected - by the
    # moment somebody does. It also owns TCP 3401, so io_manager leaves that
    # port alone (nodes/io_manager/edge_link.py, LIGMAX_EDGE_OWNER).
    {"name": "self_driving", "cmd": [sys.executable, "-m", "nodes.self_driving.main"], "on":True}
]

RESTART_BACKOFF_S = 5.0  # a node that exits immediately must not respawn in a tight loop

RUNNING_NODES = {}
async def start_node(node, logging_sock):
    if node["name"] in RUNNING_NODES:
        await logging_sock.send_string(json.dumps({"name": "main.py", "level": "info", "message": f"Node {node['name']} is already running"}))
        return

    while node["on"]:
        proc = await asyncio.create_subprocess_exec(*node["cmd"], env=NODE_ENV)
        RUNNING_NODES[node["name"]] = proc
        await logging_sock.send_string(json.dumps({"name": "main.py", "level": "info", "message": f"Started node {node['name']}"}))
        await proc.wait()
        await logging_sock.send_string(json.dumps({"name": "main.py", "level": "info", "message": f"Node {node['name']} exited with code {proc.returncode}"}))
        if node["on"]:
            await asyncio.sleep(RESTART_BACKOFF_S)


async def stop_node(node, logging_sock):
    if node["name"] in RUNNING_NODES:
        proc = RUNNING_NODES[node["name"]]
        proc.terminate()
        await proc.wait()
        del RUNNING_NODES[node["name"]]
        await logging_sock.send_string(json.dumps({"name": "main.py", "level": "info", "message": f"Stopped node {node['name']}"}))
    

async def reciver_loop(sub_sock, logging_sock):
    while True:
        try:
            msg = await sub_sock.recv_string()
            json_msg = json.loads(msg)
            for item in NODES:
                if item["name"] == json_msg.get("name"):
                    item["on"] = json_msg.get("on", item["on"])
                    if item["on"]:
                        asyncio.create_task(start_node(item, logging_sock))
                    else:
                        asyncio.create_task(stop_node(item, logging_sock))

        except Exception as e:
            log_msg = json.dumps({"name": "main.py", "level": "error", "Exception": str(e), "message": f"Error in reciever_loop on message {msg}"})
            await logging_sock.send_string(log_msg)



async def main():
    ctx = zmq.asyncio.Context()
    sub_sock = ctx.socket(zmq.SUB)
    sub_sock.connect("tcp://127.0.0.1:5556") 
    sub_sock.setsockopt_string(zmq.SUBSCRIBE, "")

    logging_sock = ctx.socket(zmq.PUB)
    logging_sock.bind(f"tcp://127.0.0.1:{LOGGING_PORT}")

    for item in NODES:
        if item["on"]:
            asyncio.create_task(start_node(item, logging_sock))

    await reciver_loop(sub_sock, logging_sock)

if __name__ == "__main__":
    asyncio.run(main())