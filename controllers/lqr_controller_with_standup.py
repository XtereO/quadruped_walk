import time
import platform
import math
import signal
import zmq
import struct

## ZMQ
ctx = zmq.Context()
sock_state = ctx.socket(zmq.SUB)
sock_state.connect("tcp://127.0.0.1:5555")
sock_state.setsockopt(zmq.SUBSCRIBE, b"")

sock_u = ctx.socket(zmq.PUB)
sock_u.bind("tcp://127.0.0.1:5556")

if platform.system() == 'Windows':
    # Windows: Set linger to 0 to avoid hanging
    sock_state.setsockopt(zmq.LINGER, 0)
    sock_u.setsockopt(zmq.LINGER, 0)
else:
    # Unix: Handle SIGPIPE
    signal.signal(signal.SIGPIPE, signal.SIG_DFL)

def main():
    K = [-2.23607, -25.29926,  -1.49292,  -5.85518]

    t, theta, dtheta, psi, dpsi = 0.0, 0.0, 0.0, 0.0, 0.0
    u = 0.0

    while True:

        try:
            msg = sock_state.recv(zmq.NOBLOCK)
            t, theta, dtheta, psi, dpsi = struct.unpack("dffff", msg)
        except:
            continue

        if t > 0.0:
            u = -(K[0] * theta + \
                K[1] * psi + \
                K[2] * dtheta + \
                K[3] * dpsi)

            if (u > 8.0): u = 8.0
            if (u < -8.0): u = -8.0

            if (abs(psi) >= 1.7):
                sign = int(math.copysign(1, psi))
                u = sign * 8.0

        msg = struct.pack("dff", t, u, u)
        sock_u.send(msg, zmq.NOBLOCK)


if __name__=="__main__":
    main()
