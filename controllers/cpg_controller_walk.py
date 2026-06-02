import time
import math
import signal
import platform
import zmq
import struct
import numpy as np
import control as ct

# ZMQ Setup with Windows fix
ctx = zmq.Context()

# SUB socket (receives state from simulator)
sock_state = ctx.socket(zmq.SUB)
sock_state.connect("tcp://127.0.0.1:5555")
sock_state.setsockopt(zmq.SUBSCRIBE, b"")
sock_state.setsockopt(zmq.RCVTIMEO, 100)  # 100ms timeout

# PUB socket (sends control to simulator)
sock_u = ctx.socket(zmq.PUB)
sock_u.bind("tcp://127.0.0.1:5556")

# Windows-specific socket settings
if platform.system() == 'Windows':
    sock_state.setsockopt(zmq.LINGER, 0)
    sock_u.setsockopt(zmq.LINGER, 0)
else:
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)

class CPGQuadruped2D:
    def __init__(self):
        # Gait parameters - softer, more natural movement
        self.gait_frequency = 1.1  # Hz (slower for smoother motion)
        self.stride_amplitude = 0.32  # Radians (moderate step size)
        self.step_height = 0.22  # Radians (moderate foot lift)
        
        # Leg names
        self.leg_names = ['FR', 'FL', 'RR', 'RL']
        
        # Phase offsets for trot gait
        self.phase_offsets = {
            'FR': 0.0,
            'FL': math.pi,
            'RR': math.pi,
            'RL': 0.0
        }
        
        # Home/sitting positions
        self.home_positions = {
            'FR': {'hip': 0.0, 'thigh': 0.3, 'calf': -0.5},
            'FL': {'hip': 0.0, 'thigh': 0.3, 'calf': -0.5},
            'RR': {'hip': 0.0, 'thigh': 0.3, 'calf': -0.5},
            'RL': {'hip': 0.0, 'thigh': 0.3, 'calf': -0.5}
        }
        
        # Joint limits
        self.joint_limits = {
            'hip': (-0.08, 0.08),
            'thigh': (-0.2, 0.8),
            'calf': (-1.0, 0.2)
        }
        
        # Soft motion parameters
        self.acceleration_limit = 1.0  # rad/s^2 (limits joint acceleration)
        self.velocity_limit = 2.0     # rad/s (limits joint velocity)
        
        # Previous commands for smoothing
        self.prev_thigh_cmd = {leg: 0.3 for leg in self.leg_names}
        self.prev_calf_cmd = {leg: -0.5 for leg in self.leg_names}
        self.prev_hip_cmd = {leg: 0.0 for leg in self.leg_names}
        
        # PID for body height (softer gains)
        self.height_error_integral = 0
        self.last_height_error = 0
        self.height_kp = 1.8   # Reduced for softer response
        self.height_ki = 0.15  # Reduced
        self.height_kd = 0.5   # Reduced
        
        # Desired states
        self.desired_height = 0.22
        self.desired_velocity = 0.1
        
        # Control smoothing
        self.prev_controls = np.zeros(12)
        self.smoothing = 0.6  # Higher = smoother, slower response
        
        # Timing
        self.last_t = 0
        self.last_print_time = 0
        self.last_dt = 0.005


        # PID for pitch stabilization (softer gains)
        self.pitch_error_integral = 0
        self.last_pitch_error = 0
        self.pitch_kp = 1.0    # Reduced for softer response
        self.pitch_ki = 0.3    # Reduced
        self.pitch_kd = 0.5    # Reduced

        # LQR gains (tuned for quadruped)
        self.K_pitch = np.array([-1.5, -0.5])  # [Kp, Kd] equivalent but optimal
        # Add integrator for steady-state error
        self.pitch_integral = 0
        self.pitch_integral_gain = 0.15
        # Simplified pitch dynamics (pendulum model)
        # I * theta_ddot + m*g*h * theta = torque
        I = 0.06  # Moment of inertia (kg*m²) - from your XML
        m = 9.2  # Mass (kg)
        g = 9.81
        h = 0.1   # Height of COM (m)

        # State space: [pitch, pitch_rate]
        A = np.array([[0, 1],
              [m*g*h/I, 0]])
        B = np.array([[0],
              [1/I]])

        # LQR weights
        Q = np.diag([5.0, 0.5])  # Penalize angle more than rate
        R = np.array([[10]])      # Control effort penalty

        # Solve for optimal gains
        K, S, E = ct.lqr(A, B, Q, R)
        self.K_pitch = -K[0]  # Returns [angle_gain, rate_gain]
        print(self.K_pitch)
        
    def parse_state(self, msg):
        """Parse simulator state message"""
        try:
            data = struct.unpack("d" * 37, msg)
            
            state = {
                't': data[0],
                'body_x': data[1],
                'body_y': data[2],
                'body_z': data[3],
                'body_roll': data[4],
                'body_pitch': data[5],
                'body_yaw': data[6],
                'body_vx': data[7],
                'body_vy': data[8],
                'body_vz': data[9],
                'body_roll_rate': data[10],
                'body_pitch_rate': data[11],
                'body_yaw_rate': data[12],
            }
            
            # Joint positions
            state['joints'] = {}
            joint_names = ['FR_hip', 'FR_thigh', 'FR_calf',
                          'FL_hip', 'FL_thigh', 'FL_calf',
                          'RR_hip', 'RR_thigh', 'RR_calf',
                          'RL_hip', 'RL_thigh', 'RL_calf']
            
            # is it right mapping to legs?
            for i, name in enumerate(joint_names):
                state['joints'][name] = data[13 + i]
            
            return state
            
        except Exception as e:
            return None
    
    def smooth_command(self, new_cmd, prev_cmd, dt, joint_type='thigh'):
        """Apply acceleration and velocity limiting for soft motion"""
        # Calculate rate of change
        delta = new_cmd - prev_cmd
        
        # Limit velocity (max change per second)
        max_velocity = self.velocity_limit
        max_delta = max_velocity * dt
        delta = np.clip(delta, -max_delta, max_delta)
        
        # Apply acceleration limit
        if hasattr(self, 'prev_delta'):
            accel = (delta - self.prev_delta) / dt if dt > 0 else 0
            if abs(accel) > self.acceleration_limit:
                # Scale down delta to respect acceleration limit
                accel_sign = np.sign(accel)
                delta = self.prev_delta + accel_sign * self.acceleration_limit * dt
        
        self.prev_delta = delta
        smoothed_cmd = prev_cmd + delta
        
        return smoothed_cmd
    
    def height_control(self, current_height, dt):
        """Soft PID controller for body height"""
        error = self.desired_height - current_height
        
        # P term
        p = self.height_kp * error
        
        # I term (with anti-windup)
        self.height_error_integral += error * dt
        self.height_error_integral = np.clip(self.height_error_integral, -0.08, 0.08)
        i = self.height_ki * self.height_error_integral
        
        # D term
        d = self.height_kd * (error - self.last_height_error) / dt if dt > 0 else 0
        self.last_height_error = error
        
        correction = p + i + d
        return np.clip(correction, -0.12, 0.12)
    
    def pitch_control(self, pitch, pitch_rate, dt):
        """LQR controller for body pitch - more stable than PID"""
        # State vector
        x = np.array([pitch, pitch_rate])
    
        # LQR control law: u = -K * x
        correction = -np.dot(self.K_pitch, x)
    
        # Add integral term to eliminate steady-state error
        self.pitch_integral += pitch * dt
        self.pitch_integral = np.clip(self.pitch_integral, -0.1, 0.1)
        correction += self.pitch_integral_gain * self.pitch_integral
    
        # Limit output
        return np.clip(correction, -0.3, 0.3)
        """Soft PID controller for body pitch"""
        error = -pitch
        
        # P term
        p = self.pitch_kp * error
        
        # I term
        self.pitch_error_integral += error * dt
        self.pitch_error_integral = np.clip(self.pitch_error_integral, -0.08, 0.08)
        i = self.pitch_ki * self.pitch_error_integral
        
        # D term
        d = self.pitch_kd * (error - self.last_pitch_error) / dt if dt > 0 else 0
        self.last_pitch_error = error
        
        correction = p + i + d
        return np.clip(correction, -0.2, 0.2)
    

    def compute_gait(self, leg, t, height_correction, pitch_correction, velocity_factor, state):
        """Compute joint angles with smooth transitions"""
        # Get phase for this leg
        phase = 2 * math.pi * self.gait_frequency * t + self.phase_offsets[leg]
        
        # Use smooth sine wave for all motions (no abrupt changes)
        # Thigh angle: smooth reciprocating motion
        side_factor = 0
        if leg in ['FR', 'RR']:  # Right legs
            side_factor = 1.0 - state['body_yaw'] * 0.5
        else:  # Left legs
            side_factor = 1.0 + state['body_yaw'] * 0.5
    
        thigh_offset = self.stride_amplitude * velocity_factor * side_factor
        #thigh_offset = self.stride_amplitude * velocity_factor
        
        # Smoother motion using sine squared for softer foot placement
        smooth_factor = math.sin(phase)**2  # Creates softer transitions
        
        if leg in ['FR', 'FL']:
            # Front legs
            thigh_cmd = self.home_positions[leg]['thigh'] - thigh_offset * math.sin(phase)
            # Add small forward bias
            thigh_cmd += 0.04
        else:
            # Rear legs
            thigh_cmd = self.home_positions[leg]['thigh'] - thigh_offset * math.sin(phase + math.pi)
            thigh_cmd += 0.02

        '''
        if abs(state['body_yaw']) > 0.05:  # If turned
            if state['body_yaw'] > 0:  # Turned right
                thigh_cmd += 0.02 if leg in ['FR', 'RR'] else -0.02  # Right legs push more
            else:  # Turned left
                thigh_cmd += -0.02 if leg in ['FR', 'RR'] else 0.02
        '''
        # Calf angle: smooth lifting motion
        calf_offset = self.step_height
        calf_cmd = self.home_positions[leg]['calf'] + calf_offset * math.cos(phase) * 0.7
        
        # Add body stabilization corrections (softer)
        if leg in ['FR', 'FL']:
            thigh_cmd += pitch_correction * 0.25
        else:
            thigh_cmd -= pitch_correction * 0.25
        
        # Add height correction
        thigh_cmd += height_correction * 0.1 #0.2
        calf_cmd += height_correction * 0.5 #0.1
        
        # Hip joint
        hip_cmd = pitch_correction * 0.05
        yaw_correction = -state['body_yaw_rate'] * 0.1 - state['body_yaw'] * 0.05
        hip_cmd += yaw_correction
        
        return hip_cmd, thigh_cmd, calf_cmd
    
    def compute_control(self, state):
        """Main control loop with soft motion"""
        if state is None:
            return np.array([0, 0.3, -0.5] * 4)
        
        t = state['t']
        
        # Calculate dt with upper limit
        if self.last_t > 0:
            dt = min(t - self.last_t, 0.01)
            dt = max(dt, 0.001)
        else:
            dt = 0.005
        self.last_dt = dt
        self.last_t = t
        
        # Compute corrections (softer)
        height_correction = self.height_control(state['body_z'], dt) # 0
        pitch_correction = self.pitch_control(state['body_pitch'], state['body_pitch_rate'], dt)
        
        # Velocity factor (gentle adjustment)
        velocity_error = self.desired_velocity - state['body_vx']
        velocity_factor = 1.0 + np.clip(velocity_error * 0.3, -0.2, 0.3)
        
        # Build control commands for all legs
        controls = []
        for i, leg in enumerate(self.leg_names):
            # Get raw gait command
            hip_raw, thigh_raw, calf_raw = self.compute_gait(
                leg, t, height_correction, pitch_correction, velocity_factor, state
            )
            
            # Apply smoothing to each joint
            hip_smooth = self.smooth_command(hip_raw, self.prev_hip_cmd[leg], dt, 'hip')
            thigh_smooth = self.smooth_command(thigh_raw, self.prev_thigh_cmd[leg], dt, 'thigh')
            calf_smooth = self.smooth_command(calf_raw, self.prev_calf_cmd[leg], dt, 'calf')
            
            # Store for next iteration
            self.prev_hip_cmd[leg] = hip_smooth
            self.prev_thigh_cmd[leg] = thigh_smooth
            self.prev_calf_cmd[leg] = calf_smooth
            
            # Apply joint limits
            hip_smooth = np.clip(hip_smooth, self.joint_limits['hip'][0], self.joint_limits['hip'][1])
            thigh_smooth = np.clip(thigh_smooth, self.joint_limits['thigh'][0], self.joint_limits['thigh'][1])
            calf_smooth = np.clip(calf_smooth, self.joint_limits['calf'][0], self.joint_limits['calf'][1])
            
            controls.extend([hip_smooth, thigh_smooth, calf_smooth])
        
        # Final smoothing filter
        controls = np.array(controls)
        smoothed = self.smoothing * controls + (1 - self.smoothing) * self.prev_controls
        self.prev_controls = smoothed
        
        return smoothed
    
    def print_status(self, state, controls):
        """Print status information"""
        current_time = time.time()
        if current_time - self.last_print_time > 1.5:  # Print less frequently
            self.last_print_time = current_time
            
            direction = "FORWARD →" if state['body_vx'] > 0.05 else ("BACKWARD ←" if state['body_vx'] < -0.05 else "STANDING ●")
            
            print(f"\n[{state['t']:5.2f}s] {direction}")
            print(f"  Pos X: {state['body_x']:6.3f}m | Vel: {state['body_vx']:6.3f}m/s | "
                  f"H: {state['body_z']:6.3f}m | Pitch: {math.degrees(state['body_pitch']):5.1f}°")
            
            # Show thigh commands (smoothness indicator)
            print(f"  Thigh: FR={controls[1]:6.3f} | FL={controls[4]:6.3f} | "
                  f"RR={controls[7]:6.3f} | RL={controls[10]:6.3f}")
            
            # Show actual positions
            if 'joints' in state:
                print(f"  Actual: FR={state['joints']['FR_thigh']:6.3f} | "
                      f"FL={state['joints']['FL_thigh']:6.3f} | "
                      f"RR={state['joints']['RR_thigh']:6.3f} | "
                      f"RL={state['joints']['RL_thigh']:6.3f}")
            
            # Motion quality indicator
            cmd_rate = np.mean(np.abs(np.diff(controls[1::3]))) if len(controls) > 1 else 0
            smoothness = "Smooth" if cmd_rate < 0.05 else "Jerky" if cmd_rate > 0.15 else "Moderate"
            print(f"  Motion: {smoothness} (Δcmd={cmd_rate:.3f}) | "
                  f"Freq={self.gait_frequency:.1f}Hz | Stride={self.stride_amplitude:.2f}")

def main():
    controller = CPGQuadruped2D()
    
    print("="*70)
    print("SOFT CPG QUADRUPED CONTROLLER - SMOOTH FORWARD WALKING")
    print("="*70)
    print(f"\nMotion Parameters (Soft settings):")
    print(f"  • Frequency: {controller.gait_frequency} Hz (slower = smoother)")
    print(f"  • Stride amplitude: {controller.stride_amplitude} rad")
    print(f"  • Step height: {controller.step_height} rad")
    print(f"  • Acceleration limit: {controller.acceleration_limit} rad/s²")
    print(f"  • Velocity limit: {controller.velocity_limit} rad/s")
    print(f"  • Control smoothing: {controller.smoothing}")
    print(f"\nDesired velocity: {controller.desired_velocity} m/s FORWARD")
    print("\nPress Ctrl+C to stop")
    print("="*70)
    
    try:
        while True:
            try:
                # Receive state from simulator
                msg = sock_state.recv(zmq.NOBLOCK)
                state = controller.parse_state(msg)
                
                if state and state['t'] > 0:
                    # Compute control
                    controls = controller.compute_control(state)
                    
                    # Send control command
                    control_msg = struct.pack("d" + "f"*12, state['t'], *controls)
                    sock_u.send(control_msg, zmq.NOBLOCK)
                    
                    # Print status
                    controller.print_status(state, controls)
                    
            except zmq.Again:
                time.sleep(0.001)
                continue
            except Exception as e:
                print(f"Error: {e}")
                time.sleep(0.01)
                
    except KeyboardInterrupt:
        print("\n\nStopping controller...")
        sock_state.close()
        sock_u.close()
        ctx.term()
        print("Controller stopped.")

if __name__ == "__main__":
    main()