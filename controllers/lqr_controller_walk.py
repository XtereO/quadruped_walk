import time
import platform
import math
import signal
import numpy as np
import zmq
import struct

## ZMQ Setup
ctx = zmq.Context()
sock_state = ctx.socket(zmq.SUB)
sock_state.connect("tcp://127.0.0.1:5555")
sock_state.setsockopt(zmq.SUBSCRIBE, b"")

sock_u = ctx.socket(zmq.PUB)
sock_u.bind("tcp://127.0.0.1:5556")

if platform.system() == 'Windows':
    sock_state.setsockopt(zmq.LINGER, 0)
    sock_u.setsockopt(zmq.LINGER, 0)
else:
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)

class QuadrupedController:
    def __init__(self):
        # LOWER gains for softer control
        self.kp_thigh = 40.0   # Reduced from 120
        self.kp_calf = 35.0    # Reduced from 100
        self.kp_hip = 40.0     # Reduced from 80
        self.kd = 8.0          # Reduced from 15
        
        # SMALLER amplitudes for gentle walking
        self.walk_freq = 1.2   # Hz
        self.thigh_amplitude = 0.35  # Reduced from 0.45
        self.calf_amplitude = 0.40   # Reduced from 0.50
        
        # Joint offsets
        self.thigh_offset = 0.45
        self.calf_offset = -0.65
        
        # Target velocity - small to start
        self.target_velocity = 0.03  # m/s
        
        # Gait pattern (trot)
        self.phase_offset = {
            'FR': 0.0,
            'FL': math.pi,
            'RR': math.pi,
            'RL': 0.0
        }
        
        # Soft stabilization gains
        self.roll_gain = 15.0
        self.pitch_gain = 15.0
        self.yaw_gain = 10.0
        
        self.max_torque = 18.0  # Reduced from 33.5
        
    def run(self):
        print("=" * 60)
        print("SOFT QUADRUPED WALKING - FORWARD DIRECTION")
        print(f"Walk frequency: {self.walk_freq} Hz")
        print(f"Thigh amplitude: {self.thigh_amplitude} rad")
        print(f"Target velocity: {self.target_velocity} m/s FORWARD")
        print("=" * 60)
        
        t = 0.0
        
        joint_names = [
            'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
            'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
            'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
            'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint'
        ]
        
        joint_positions = {name: 0.0 for name in joint_names}
        joint_velocities = {name: 0.0 for name in joint_names}
        
        step_count = 0
        
        while True:
            try:
                msg = sock_state.recv(zmq.NOBLOCK)
                unpacked = struct.unpack("d" + "d"*36, msg)
                t = unpacked[0]
                
                # Body states
                body_x = unpacked[1]
                body_vx = unpacked[7]
                body_roll = unpacked[4]
                body_pitch = unpacked[5]
                body_yaw = unpacked[6]
                body_roll_rate = unpacked[10]
                body_pitch_rate = unpacked[11]
                body_yaw_rate = unpacked[12]
                
                # Joint states
                for i, name in enumerate(joint_names):
                    joint_positions[name] = unpacked[13 + i]
                    joint_velocities[name] = unpacked[25 + i]
                
                # Simple velocity control
                vel_error = self.target_velocity - body_vx
                speed_factor = max(0.3, min(1.0, 0.7 + vel_error * 1.5))
                
                # Soft stabilization
                roll_adj = -self.roll_gain * body_roll - 5.0 * body_roll_rate
                pitch_adj = -self.pitch_gain * body_pitch - 5.0 * body_pitch_rate
                yaw_adj = -self.yaw_gain * body_yaw - 3.0 * body_yaw_rate
                
                # FORWARD gait - positive sine = thigh forward = robot forward
                # FR and RL together
                phase_FR = 2 * math.pi * self.walk_freq * t
                phase_FL = phase_FR + math.pi
                phase_RR = phase_FR + math.pi
                phase_RL = phase_FR
                
                # Calculate targets with FORWARD direction
                # Thigh: positive angle = forward = robot moves forward
                fr_thigh = self.thigh_offset + self.thigh_amplitude * math.sin(phase_FR) * speed_factor
                fr_calf = self.calf_offset - self.calf_amplitude * math.sin(phase_FR) * speed_factor
                fr_hip = 0.05 * math.sin(phase_FR) + 0.05 * yaw_adj + 0.05 * roll_adj
                
                fl_thigh = self.thigh_offset + self.thigh_amplitude * math.sin(phase_FL) * speed_factor
                fl_calf = self.calf_offset - self.calf_amplitude * math.sin(phase_FL) * speed_factor
                fl_hip = 0.05 * math.sin(phase_FL) - 0.05 * yaw_adj + 0.05 * roll_adj
                
                rr_thigh = self.thigh_offset + self.thigh_amplitude * math.sin(phase_RR) * speed_factor
                rr_calf = self.calf_offset - self.calf_amplitude * math.sin(phase_RR) * speed_factor
                rr_hip = -0.05 * math.sin(phase_RR) + 0.05 * yaw_adj - 0.05 * roll_adj
                
                rl_thigh = self.thigh_offset + self.thigh_amplitude * math.sin(phase_RL) * speed_factor
                rl_calf = self.calf_offset - self.calf_amplitude * math.sin(phase_RL) * speed_factor
                rl_hip = -0.05 * math.sin(phase_RL) - 0.05 * yaw_adj - 0.05 * roll_adj
                
                # Apply soft PD control
                cmd_list = []
                
                # FR
                cmd_list.append(self.kp_hip * (fr_hip - joint_positions['FR_hip_joint']) - self.kd * joint_velocities['FR_hip_joint'])
                cmd_list.append(self.kp_thigh * (fr_thigh - joint_positions['FR_thigh_joint']) - self.kd * joint_velocities['FR_thigh_joint'])
                cmd_list.append(self.kp_calf * (fr_calf - joint_positions['FR_calf_joint']) - self.kd * joint_velocities['FR_calf_joint'])
                
                # FL
                cmd_list.append(self.kp_hip * (fl_hip - joint_positions['FL_hip_joint']) - self.kd * joint_velocities['FL_hip_joint'])
                cmd_list.append(self.kp_thigh * (fl_thigh - joint_positions['FL_thigh_joint']) - self.kd * joint_velocities['FL_thigh_joint'])
                cmd_list.append(self.kp_calf * (fl_calf - joint_positions['FL_calf_joint']) - self.kd * joint_velocities['FL_calf_joint'])
                
                # RR
                cmd_list.append(self.kp_hip * (rr_hip - joint_positions['RR_hip_joint']) - self.kd * joint_velocities['RR_hip_joint'])
                cmd_list.append(self.kp_thigh * (rr_thigh - joint_positions['RR_thigh_joint']) - self.kd * joint_velocities['RR_thigh_joint'])
                cmd_list.append(self.kp_calf * (rr_calf - joint_positions['RR_calf_joint']) - self.kd * joint_velocities['RR_calf_joint'])
                
                # RL
                cmd_list.append(self.kp_hip * (rl_hip - joint_positions['RL_hip_joint']) - self.kd * joint_velocities['RL_hip_joint'])
                cmd_list.append(self.kp_thigh * (rl_thigh - joint_positions['RL_thigh_joint']) - self.kd * joint_velocities['RL_thigh_joint'])
                cmd_list.append(self.kp_calf * (rl_calf - joint_positions['RL_calf_joint']) - self.kd * joint_velocities['RL_calf_joint'])
                
                # Soft limits
                for i in range(len(cmd_list)):
                    cmd_list[i] = max(-self.max_torque, min(self.max_torque, cmd_list[i]))
                
                msg_u = struct.pack("d" + "f"*12, t, *cmd_list)
                sock_u.send(msg_u, zmq.NOBLOCK)
                
                step_count += 1
                if step_count % 25 == 0:
                    dir_str = "FORWARD" if body_vx > 0 else "BACKWARD"
                    print(f"T:{t:5.2f}s | X:{body_x:7.2f}m | Vx:{body_vx:5.2f}m/s {dir_str} | "
                          f"FR_thigh:{fr_thigh:5.2f}rad | FR_calf:{fr_calf:5.2f}rad")
                
            except zmq.Again:
                continue
            except Exception as e:
                print(f"Error: {e}")
                continue

if __name__ == "__main__":
    controller = QuadrupedController()
    controller.run()