import signal
import platform
import zmq
import struct
import pyqtgraph as pg
from PyQt6 import QtWidgets, QtCore
from collections import deque

WINDOW = 10000          # Number of points to show
UPDATE_MS = 20
# State format: time (double) + 36 doubles = 37 doubles total
# Body: x,y,z, roll,pitch,yaw, vx,vy,vz, roll_rate,pitch_rate,yaw_rate
# Joints: 12 positions + 12 velocities
FMT = "d" + "d"*36     # 37 doubles

# Control format: time + 12 floats (joint commands)
FMT_U = "d" + "f"*12

ctx = zmq.Context("tcp://127.0.0.1:5555")
sock = ctx.socket(zmq.SUB)
sock.connect()
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
pitch_rate_buf = deque(maxlen=WINDOW)  # Pitch rate (rad/s)
yaw_buf = deque(maxlen=WINDOW)        # Body yaw (rad) - for drift
yaw_rate_buf = deque(maxlen=WINDOW)   # Yaw rate
y_buf = deque(maxlen=WINDOW)          # Lateral position Y (m)
x_buf = deque(maxlen=WINDOW)          # Forward position X (m)
vx_buf = deque(maxlen=WINDOW)         # Forward velocity (m/s)
height_buf = deque(maxlen=WINDOW)     # Body height Z (m)
roll_buf = deque(maxlen=WINDOW)       # Body roll (rad)

# Control inputs (joint commands)
fr_thigh_buf = deque(maxlen=WINDOW)   # Front Right
fl_thigh_buf = deque(maxlen=WINDOW)   # Front Left
rr_thigh_buf = deque(maxlen=WINDOW)   # Rear Right
rl_thigh_buf = deque(maxlen=WINDOW)   # Rear Left
fr_calf_buf = deque(maxlen=WINDOW)    # Front Right (calf)


def create_plot(title, charts, labels, winNextRow=True):
    p = win.addPlot(title=title)
    p.addLegend()
    p.showGrid(x=True, y=True)
    cv = []
    for c in charts:
        cv.append(p.plot(pen=pg.mkPen(c[0], width=2), name=c[1]))
    for l in labels:
        p.setLabel(l[0], l[1])

    if (winNextRow):
        win.nextRow()

    return p, cv
t_label = ('bottom', 'Time (s)')
create_time_plot = lambda title, charts, labels, winNextRow=True: create_plot(title, charts, [*labels, t_label], winNextRow)

p1, [c_pitch, c_pitch_rate] = create_time_plot('Body Pitch (Balance)', [(
    'y', 'pitch (rad)'), ('c', 'pitch rate (rad/s)')], [('left', 'Angle / Rate')])
p2, [c_yaw, c_yaw_rate] = create_time_plot(
    'Drift / Yaw', [('m', 'yaw (rad)'), ('g', 'yaw rate (rad/s)')], [('left', 'Yaw (rad)')])
p3, [c_vx, c_x] = create_time_plot('Forward Motion', [(
    'b', 'velocity X (m/s)'), ('r', 'position X (m)')], [('left', 'Velocity / Position')])
p4, [c_height, c_roll] = create_time_plot('Height & Roll', [((139, 69, 19), 'height (m)'), ((
    255, 165, 0), 'roll (rad)')], [('left', 'Height (m) / Roll (rad)')])
p5, [c_fr_thigh, c_fl_thigh, c_rr_thigh, c_rl_thigh] = create_time_plot('Thigh Joint Commands', [(
    'r', 'FR thigh'), ('g', 'FL thigh'), ('b', 'RR thigh'), ('m', 'RL thigh')], [('left', 'Command (rad)')])
p6, [c_fr_calf, c_y] = create_time_plot('Calf Command & Y Position', [(
    'y', 'FR calf'), ('w', 'Y position (m)')], [('left', 'Command (rad) / Y (m)')], False)


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
        fr_thigh_pos = data[14]   
        fl_thigh_pos = data[17]   
        rr_thigh_pos = data[20]   
        rl_thigh_pos = data[23]   
        fr_calf_pos = data[15]    

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
        fr_thigh = data[2]   
        fl_thigh = data[5]   
        rr_thigh = data[8]   
        rl_thigh = data[11]  
        fr_calf = data[3]    
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
        c_pitch.setData(list(t_buf), list(pitch_buf))
        c_pitch_rate.setData(list(t_buf), list(pitch_rate_buf))

        c_yaw.setData(list(t_buf), list(yaw_buf))
        c_yaw_rate.setData(list(t_buf), list(yaw_rate_buf))

        c_vx.setData(list(t_buf), list(vx_buf))
        c_x.setData(list(t_buf), list(x_buf))

        c_height.setData(list(t_buf), list(height_buf))
        c_roll.setData(list(t_buf), list(roll_buf))

        if len(fr_thigh_buf) > 0:
            c_fr_thigh.setData(list(t_buf), list(fr_thigh_buf))
            c_fl_thigh.setData(list(t_buf), list(fl_thigh_buf))
            c_rr_thigh.setData(list(t_buf), list(rr_thigh_buf))
            c_rl_thigh.setData(list(t_buf), list(rl_thigh_buf))

        if len(fr_calf_buf) > 0:
            c_fr_calf.setData(list(t_buf), list(fr_calf_buf))
        c_y.setData(list(t_buf), list(y_buf))


timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(UPDATE_MS)

win.setWindowTitle("Quadruped Robot Monitor - Pitch, Yaw, Velocity, Commands")

app.exec()
