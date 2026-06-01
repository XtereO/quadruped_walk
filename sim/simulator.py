import time
import sys
import signal
import platform
import numpy as np
import mujoco
import mujoco.viewer

import cvxpy as cp
import control as ct

import zmq
import struct

## ZMQ Setup
ctx = zmq.Context()
sock = ctx.socket(zmq.PUB)
sock.bind("tcp://127.0.0.1:5555")
sock_u = ctx.socket(zmq.SUB)
sock_u.connect("tcp://127.0.0.1:5556")
sock_u.setsockopt(zmq.SUBSCRIBE, b"")

if platform.system() == 'Windows':
    sock.setsockopt(zmq.LINGER, 0)
    sock_u.setsockopt(zmq.LINGER, 0)
else:
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)

MODEL = 'quadruped.xml'

m = mujoco.MjModel.from_xml_path(MODEL)
d = mujoco.MjData(m)

# Get joint and actuator IDs
joint_names = [
    'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
    'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
    'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'
]

actuator_names = [
    'FR_hip_act', 'FR_thigh_act', 'FR_calf_act',
    'FL_hip_act', 'FL_thigh_act', 'FL_calf_act',
    'RR_hip_act', 'RR_thigh_act', 'RR_calf_act',
    'RL_hip_act', 'RL_thigh_act', 'RL_calf_act'
]

joint_ids = {name: m.joint(name).id for name in joint_names}
actuator_ids = {name: m.actuator(name).id for name in actuator_names}

prev_time = time.time()
ctrl_inputs = np.zeros(12)

print("Quadruped Simulation Starting...")
print(f"Model: {MODEL}")
print(f"Number of joints: {m.njnt}")
print(f"Number of actuators: {m.nu}")
print(f"Number of bodies: {m.nbody}")

with mujoco.viewer.launch_passive(m, d, show_left_ui=False, show_right_ui=False) as viewer:
    
    viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    viewer.cam.distance = 2.0
    viewer.cam.azimuth = 45.0
    viewer.cam.elevation = -20.0
    
    print("Running simulation for 300 seconds...")
    start = time.time()
    
    while viewer.is_running() and time.time() - start < 300:
        current_time = time.time()
        step_start = current_time
        
        t = current_time - start
        dt = current_time - prev_time
        prev_time = current_time
        
        # Get current state
        trunk_id = m.body('trunk').id
        body_pos = d.xpos[trunk_id].copy()
        body_quat = d.xquat[trunk_id].copy()
        
        def quat_to_euler(w, x, y, z):
            sinr_cosp = 2.0 * (w * x + y * z)
            cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
            roll = np.arctan2(sinr_cosp, cosr_cosp)
            
            sinp = 2.0 * (w * y - z * x)
            if abs(sinp) >= 1:
                pitch = np.copysign(np.pi / 2, sinp)
            else:
                pitch = np.arcsin(sinp)
            
            siny_cosp = 2.0 * (w * z + x * y)
            cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
            yaw = np.arctan2(siny_cosp, cosy_cosp)
            
            return roll, pitch, yaw
        
        body_roll, body_pitch, body_yaw = quat_to_euler(
            body_quat[0], body_quat[1], body_quat[2], body_quat[3]
        )
        
        # Get joint positions and velocities
        joint_positions = {}
        joint_velocities = {}
        for name in joint_names:
            joint_id = joint_ids[name]
            joint_positions[name] = d.qpos[m.jnt_qposadr[joint_id]]
            joint_velocities[name] = d.qvel[m.jnt_dofadr[joint_id]]
        
        # Get body velocities
        body_linear_vel = d.cvel[trunk_id][:3]
        body_angular_vel = d.cvel[trunk_id][3:6]
        
        # Pack and send state via ZMQ
        # Format: time, body_x, body_y, body_z, body_roll, body_pitch, body_yaw,
        #         body_vx, body_vy, body_vz, body_roll_rate, body_pitch_rate, body_yaw_rate,
        #         12 joint positions, 12 joint velocities
        # Total: 1 + 12 + 12 + 12 = 37 doubles
        state_msg = struct.pack(
            "d" + "d"*36,  # 1 time + 36 state values
            t,
            body_pos[0], body_pos[1], body_pos[2],
            body_roll, body_pitch, body_yaw,
            body_linear_vel[0], body_linear_vel[1], body_linear_vel[2],
            body_angular_vel[0], body_angular_vel[1], body_angular_vel[2],
            joint_positions['FR_hip_joint'], joint_positions['FR_thigh_joint'], joint_positions['FR_calf_joint'],
            joint_positions['FL_hip_joint'], joint_positions['FL_thigh_joint'], joint_positions['FL_calf_joint'],
            joint_positions['RR_hip_joint'], joint_positions['RR_thigh_joint'], joint_positions['RR_calf_joint'],
            joint_positions['RL_hip_joint'], joint_positions['RL_thigh_joint'], joint_positions['RL_calf_joint'],
            joint_velocities['FR_hip_joint'], joint_velocities['FR_thigh_joint'], joint_velocities['FR_calf_joint'],
            joint_velocities['FL_hip_joint'], joint_velocities['FL_thigh_joint'], joint_velocities['FL_calf_joint'],
            joint_velocities['RR_hip_joint'], joint_velocities['RR_thigh_joint'], joint_velocities['RR_calf_joint'],
            joint_velocities['RL_hip_joint'], joint_velocities['RL_thigh_joint'], joint_velocities['RL_calf_joint']
        )
        
        sock.send(state_msg, zmq.NOBLOCK)
        
        # Receive control inputs
        try:
            msg = sock_u.recv(zmq.NOBLOCK)
            unpacked = struct.unpack("d" + "f"*12, msg)
            t_u = unpacked[0]
            ctrl_inputs = np.array(unpacked[1:13])
        except zmq.Again:
            pass
        except Exception as e:
            print(f"Error receiving control: {e}")
            ctrl_inputs = np.zeros(12)
        
        # Apply control inputs
        for i, name in enumerate(actuator_names):
            d.ctrl[actuator_ids[name]] = ctrl_inputs[i]
        
        # Step simulation
        mujoco.mj_step(m, d)
        
        # Sync viewer
        viewer.sync()
        
        # Timing control
        time_until_next_step = m.opt.timestep - (time.time() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

print("Simulation finished.")