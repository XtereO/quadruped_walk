import time
import sys
import signal
import numpy as np
import mujoco
import mujoco.viewer

import cvxpy as cp
import control as ct

import zmq
import struct

signal.signal(signal.SIGPIPE, signal.SIG_DFL)
np.set_printoptions(precision=5)

## ZMQ
ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://127.0.0.1:5555")
sock_u = ctx.socket(zmq.SUB)
sock_u.connect("tcp://127.0.0.1:5556")
sock_u.setsockopt(zmq.SUBSCRIBE, b"")


MODEL = 'segway.xml'

m = mujoco.MjModel.from_xml_path(MODEL)
d = mujoco.MjData(m)


with mujoco.viewer.launch_passive(m, d, show_left_ui=True, show_right_ui=False) as viewer:
    
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
    viewer.cam.fixedcamid = m.camera("side_cam").id
    
    psi = 0.0
    prev_psi = 0
    prev_time = time.time()
    u_left = 0.0
    u_right = 0.0
    
    print("Running...")
    start = time.time()
    while viewer.is_running() and time.time() - start < 300:
        current_time = time.time()
        step_start = current_time

        t = current_time - start
        dt = current_time - prev_time; prev_time = current_time

        # system state
        theta_right = d.joint("right-wheel-joint").qpos[0]
        dtheta_right = d.joint("right-wheel-joint").qvel[0]
        dpsi = d.sensor("imu_gyro").data[0]
        psi += dpsi * dt

        x = np.array([theta_right, psi, dtheta_right, dpsi])


        msg = struct.pack(
                "dffff",
                t, theta_right, dtheta_right, psi, dpsi
        )
        sock.send(msg, zmq.NOBLOCK)

        t_u = -1
        try:
            msg = sock_u.recv(zmq.NOBLOCK)
            t_u, u_left, u_right = struct.unpack("dff", msg)
        except zmq.Again:
            u_left = 0.0
            u_right = 0.0
        except:
            u_left = 0.0
            u_right = 0.0
        
        d.ctrl[0] = u_left
        d.ctrl[1] = u_right

        mujoco.mj_step(m, d)

        viewer.sync()
        
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
