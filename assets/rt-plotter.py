import signal
import platform
import zmq
import struct
import pyqtgraph as pg
from PyQt6 import QtWidgets, QtCore
from collections import deque

WINDOW = 10000          # Number of points to show
UPDATE_MS = 20          # Update interval in milliseconds
ADDR = "tcp://127.0.0.1:5555"
# State format: time (double) + 36 doubles = 37 doubles total
# Body: x,y,z, roll,pitch,yaw, vx,vy,vz, roll_rate,pitch_rate,yaw_rate
# Joints: 12 positions + 12 velocities
FMT = "d" + "d"*36     # 37 doubles

# Control format: time + 12 floats (joint commands)
FMT_U = "d" + "f"*12

ctx = zmq.Context()
sock = ctx.socket(zmq.SUB)
sock.connect(ADDR)
sock.setsockopt(zmq.SUBSCRIBE, b"")
sock.setsockopt(zmq.RCVTIMEO, 10)

sock_u = ctx.socket(zmq.SUB)
sock_u.connect("tcp://127.0.0.1:5556")
sock_u.setsockopt(zmq.SUBSCRIBE, b"")
sock_u.setsockopt(zmq.RCVTIMEO, 10)

if platform.system() == 'Windows':
    sock.setsockopt(zmq.LINGER, 0)
    sock_u.setsockopt(zmq.LINGER, 0)
else:
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)

app = QtWidgets.QApplication([])
win = pg.GraphicsLayoutWidget(title="Quadruped State Visualizer")
win.resize(1400, 900)
win.show()

# Data buffers
t_buf = deque(maxlen=WINDOW)
pitch_buf = deque(maxlen=WINDOW)      # Body pitch (rad)
pitch_rate_buf = deque(maxlen=WINDOW) # Pitch rate (rad/s)
yaw_buf = deque(maxlen=WINDOW)        # Body yaw (rad) - for drift
yaw_rate_buf = deque(maxlen=WINDOW)   # Yaw rate
y_buf = deque(maxlen=WINDOW)          # Lateral position Y (m)
x_buf = deque(maxlen=WINDOW)          # Forward position X (m)
vx_buf = deque(maxlen=WINDOW)         # Forward velocity (m/s)
height_buf = deque(maxlen=WINDOW)     # Body height Z (m)
roll_buf = deque(maxlen=WINDOW)       # Body roll (rad)

# Control inputs (joint commands)
fr_thigh_buf = deque(maxlen=WINDOW)   # Front Right thigh command
fl_thigh_buf = deque(maxlen=WINDOW)   # Front Left thigh command
rr_thigh_buf = deque(maxlen=WINDOW)   # Rear Right thigh command
rl_thigh_buf = deque(maxlen=WINDOW)   # Rear Left thigh command
fr_calf_buf = deque(maxlen=WINDOW)    # Front Right calf command

# Plot 1: Pitch (angle and rate)
p1 = win.addPlot(title="Body Pitch (Balance)")
p1.addLegend()
p1.showGrid(x=True, y=True)
c_pitch = p1.plot(pen=pg.mkPen('y', width=2), name="pitch (rad)")
c_pitch_rate = p1.plot(pen=pg.mkPen('c', width=1), name="pitch rate (rad/s)")
p1.setLabel('left', 'Angle / Rate')
p1.setLabel('bottom', 'Time (s)')

win.nextRow()

# Plot 2: Yaw (drift) and lateral position
p2 = win.addPlot(title="Drift / Yaw")
p2.addLegend()
p2.showGrid(x=True, y=True)
p2.setXLink(p1)
c_yaw = p2.plot(pen=pg.mkPen('m', width=2), name="yaw (rad)")
c_yaw_rate = p2.plot(pen=pg.mkPen('g', width=1), name="yaw rate (rad/s)")
p2.setLabel('left', 'Yaw (rad)')
p2.setLabel('bottom', 'Time (s)')

win.nextRow()

# Plot 3: Forward velocity and position
p3 = win.addPlot(title="Forward Motion")
p3.addLegend()
p3.showGrid(x=True, y=True)
p3.setXLink(p1)
c_vx = p3.plot(pen=pg.mkPen('b', width=2), name="velocity X (m/s)")
c_x = p3.plot(pen=pg.mkPen('r', width=1), name="position X (m)")
p3.setLabel('left', 'Velocity / Position')
p3.setLabel('bottom', 'Time (s)')

win.nextRow()

# Plot 4: Height and Roll
p4 = win.addPlot(title="Height & Roll")
p4.addLegend()
p4.showGrid(x=True, y=True)
p4.setXLink(p1)
c_height = p4.plot(pen=pg.mkPen((139, 69, 19), width=2), name="height (m)")
c_roll = p4.plot(pen=pg.mkPen((255, 165, 0), width=1), name="roll (rad)")
p4.setLabel('left', 'Height (m) / Roll (rad)')
p4.setLabel('bottom', 'Time (s)')

win.nextRow()

# Plot 5: Thigh commands (all 4 legs)
p5 = win.addPlot(title="Thigh Joint Commands")
p5.addLegend()
p5.showGrid(x=True, y=True)
p5.setXLink(p1)
c_fr_thigh = p5.plot(pen=pg.mkPen('r', width=2), name="FR thigh")
c_fl_thigh = p5.plot(pen=pg.mkPen('g', width=2), name="FL thigh")
c_rr_thigh = p5.plot(pen=pg.mkPen('b', width=2), name="RR thigh")
c_rl_thigh = p5.plot(pen=pg.mkPen('m', width=2), name="RL thigh")
p5.setLabel('left', 'Command (rad)')
p5.setLabel('bottom', 'Time (s)')

win.nextRow()

# Plot 6: Calf commands and lateral position Y
p6 = win.addPlot(title="Calf Command & Lateral Position")
p6.addLegend()
p6.showGrid(x=True, y=True)
p6.setXLink(p1)
c_fr_calf = p6.plot(pen=pg.mkPen('y', width=2), name="FR calf")
c_y = p6.plot(pen=pg.mkPen('w', width=1), name="Y position (m)")
p6.setLabel('left', 'Command (rad) / Y (m)')
p6.setLabel('bottom', 'Time (s)')

def parse_state(msg):
    """Parse 37 doubles from simulator"""
    try:
        data = struct.unpack(FMT, msg)
        t = data[0]
        
        # Body states (indices 1-12)
        body_x = data[1]
        body_y = data[2]
        body_z = data[3]
        body_roll = data[4]
        body_pitch = data[5]
        body_yaw = data[6]
        body_vx = data[7]
        body_vy = data[8]
        body_vz = data[9]
        body_roll_rate = data[10]
        body_pitch_rate = data[11]
        body_yaw_rate = data[12]
        
        # Joint positions (indices 13-24)
        fr_thigh_pos = data[14]   # FR_thigh
        fl_thigh_pos = data[17]   # FL_thigh
        rr_thigh_pos = data[20]   # RR_thigh
        rl_thigh_pos = data[23]   # RL_thigh
        fr_calf_pos = data[15]    # FR_calf
        
        return {
            't': t,
            'pitch': body_pitch,
            'pitch_rate': body_pitch_rate,
            'yaw': body_yaw,
            'yaw_rate': body_yaw_rate,
            'y': body_y,
            'x': body_x,
            'vx': body_vx,
            'height': body_z,
            'roll': body_roll,
            'fr_thigh': fr_thigh_pos,
            'fl_thigh': fl_thigh_pos,
            'rr_thigh': rr_thigh_pos,
            'rl_thigh': rl_thigh_pos,
            'fr_calf': fr_calf_pos,
        }
    except Exception as e:
        return None

def parse_control(msg):
    """Parse control message: time + 12 floats"""
    try:
        data = struct.unpack(FMT_U, msg)
        t = data[0]
        # Joint order: FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf,
        #             RR_hip, RR_thigh, RR_calf, RL_hip, RL_thigh, RL_calf
        fr_thigh = data[2]   # FR_thigh is index 2
        fl_thigh = data[5]   # FL_thigh is index 5
        rr_thigh = data[8]   # RR_thigh is index 8
        rl_thigh = data[11]  # RL_thigh is index 11
        fr_calf = data[3]    # FR_calf is index 3
        return {
            't': t,
            'fr_thigh': fr_thigh,
            'fl_thigh': fl_thigh,
            'rr_thigh': rr_thigh,
            'rl_thigh': rl_thigh,
            'fr_calf': fr_calf,
        }
    except Exception as e:
        return None

def update():
    updated = False
    
    # Receive state from simulator
    while True:
        try:
            msg = sock.recv(zmq.NOBLOCK)
            state = parse_state(msg)
            if state:
                t_buf.append(state['t'])
                pitch_buf.append(state['pitch'])
                pitch_rate_buf.append(state['pitch_rate'])
                yaw_buf.append(state['yaw'])
                yaw_rate_buf.append(state['yaw_rate'])
                y_buf.append(state['y'])
                x_buf.append(state['x'])
                vx_buf.append(state['vx'])
                height_buf.append(state['height'])
                roll_buf.append(state['roll'])
                updated = True
        except zmq.Again:
            break
    
    # Receive control commands
    while True:
        try:
            msg_u = sock_u.recv(zmq.NOBLOCK)
            ctrl = parse_control(msg_u)
            if ctrl:
                fr_thigh_buf.append(ctrl['fr_thigh'])
                fl_thigh_buf.append(ctrl['fl_thigh'])
                rr_thigh_buf.append(ctrl['rr_thigh'])
                rl_thigh_buf.append(ctrl['rl_thigh'])
                fr_calf_buf.append(ctrl['fr_calf'])
                updated = True
        except zmq.Again:
            break
    
    if not updated:
        return
    
    # Update all plots
    if len(t_buf) > 0:
        # Plot 1: Pitch
        c_pitch.setData(list(t_buf), list(pitch_buf))
        c_pitch_rate.setData(list(t_buf), list(pitch_rate_buf))
        
        # Plot 2: Yaw
        c_yaw.setData(list(t_buf), list(yaw_buf))
        c_yaw_rate.setData(list(t_buf), list(yaw_rate_buf))
        
        # Plot 3: Velocity and position
        c_vx.setData(list(t_buf), list(vx_buf))
        c_x.setData(list(t_buf), list(x_buf))
        
        # Plot 4: Height and roll
        c_height.setData(list(t_buf), list(height_buf))
        c_roll.setData(list(t_buf), list(roll_buf))
        
        # Plot 5: Thigh commands
        if len(fr_thigh_buf) > 0:
            c_fr_thigh.setData(list(t_buf), list(fr_thigh_buf))
            c_fl_thigh.setData(list(t_buf), list(fl_thigh_buf))
            c_rr_thigh.setData(list(t_buf), list(rr_thigh_buf))
            c_rl_thigh.setData(list(t_buf), list(rl_thigh_buf))
        
        # Plot 6: Calf commands and Y position
        if len(fr_calf_buf) > 0:
            c_fr_calf.setData(list(t_buf), list(fr_calf_buf))
        c_y.setData(list(t_buf), list(y_buf))

# Timer setup
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(UPDATE_MS)

# Set window title
win.setWindowTitle("Quadruped Robot Monitor - Pitch, Yaw, Velocity, Commands")

app.exec()