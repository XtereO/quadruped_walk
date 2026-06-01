import control as ct
import matplotlib.pyplot as plt
import scipy
import numpy as np
np.set_printoptions(precision=5)

g = 9.81

"""
Model parameters (for mujoco segway.xml)
""" 

# wheel
m_wheel = 1.8
r_wheel = 0.17 / 2.0
J_wheel = 0.5 * m_wheel * r_wheel**2

# body
m_body = 1.0
com_body = 0.426
J_body = m_body * com_body**2 / 12.0

"""
Linear model in form E qdd + F qd + G q = H u
"""

E = np.array([
    [m_body * com_body * r_wheel,   m_body * com_body**2 + J_body],
    [m_body * r_wheel**2 + 2.0 * m_wheel * r_wheel**2 + 2.0 * J_wheel, m_body * com_body * r_wheel]
]);

F = np.array([
    [0, 0],
    [0, 0]
])

G = np.array([
    [0, -m_body * com_body * g],
    [0, 0]
])

H = np.array([
    [-2],
    [2]
])


"""
Linear model in from xd = Ax + Bu
"""
invE = np.linalg.inv(E)

EG = invE @ G
EF = invE @ F
EH = invE @ H
EG = -1.0 * EG
EF = -1.0 * EF

# x = [theta, psi, dtheta, dpsi]
A = np.array([   
    [0, 0, 1, 0],
    [0, 0, 0, 1],
    [0, EG[0,1], EF[0,0], EF[0,1]],
    [0, EG[1,1], EF[1,0], EF[1,1]],
])

B = np.array([
    [0 ],
    [0],
    [EH[0,0]],
    [EH[1,0]],
])

C = np.eye(4)
D = np.zeros((4,1))

print(A)
print(B)

c_matrix = np.hstack((B,A@B,A@A@B, A@A@A@B))
o_matrix = np.vstack((C,C@A,C@A@A, C@A@A@A))

print(np.linalg.matrix_rank(c_matrix))
print(np.linalg.matrix_rank(o_matrix))

"""
Discretize linear ss model
"""
sys = ct.ss(A, B, C, D)


# print(np.linalg.eigvals(A))
dt = 0.005
sys_d = ct.c2d(sys, dt)

Ad = sys_d.A
Bd = sys_d.B
print(np.array2string(Ad, separator=", "))
print(np.array2string(Bd, separator=", "))
print(np.linalg.eigvals(Ad))

"""
State feeadback design
"""

"MPC (without constraints)"
Q = np.diag([50, 100, 1, 1])
R = np.array([[10]])

N = 200
n_x = Ad.shape[0]
n_u = Bd.shape[1]

barQ = scipy.linalg.block_diag(*[Q]*N)
barR = scipy.linalg.block_diag(*[R]*N)

Phi = np.vstack([np.linalg.matrix_power(Ad, i+1) for i in range(N)])

Psi = np.zeros((N*n_x, N*n_u))
for i in range(N):
    for j in range(N):
        if i >= j:
            power = i - j
            if power == 0:
                Psi[i*n_x:(i+1)*n_x, j*n_u:(j+1)*n_u] = Bd
            else:
                Psi[i*n_x:(i+1)*n_x, j*n_u:(j+1)*n_u] = np.linalg.matrix_power(Ad, power) @ Bd

H = Psi.T @ barQ @ Psi + barR
F = Psi.T @ barQ @ Phi
Kmpc_full = np.linalg.solve(H, F)

Kmpc = Kmpc_full[0:n_u, :]
print("Kmpc", np.array2string(Kmpc.flatten(), separator=", "))
from scipy.linalg import solve_discrete_are
P = solve_discrete_are(Ad, Bd, Q, R)
print("P", np.array2string(P.flatten(), separator=", "))


"LQR"
# Klqr, S, E = ct.lqr(A, B, Q, R)
Klqr, _, _ = ct.dlqr(Ad, Bd, Q, R)
print("Klqr", np.array2string(Klqr.flatten(), separator=", "))


"Modal control"
time = 1.4; w0 = 7.8 / time
roots = [-w0] * 4
coeffs = np.poly(roots)

I = np.eye(4)
A2 = A @ A
A3 = A2 @ A
A4 = A3 @ A
akp = A4 + coeffs[1] * A3 + coeffs[2] * A2 + coeffs[3] * A + coeffs[4] * I

Kmodal = np.array([np.array([0, 0, 0, 1]) @ np.linalg.inv(c_matrix) @ akp])
print("Kmodal", np.array2string(Kmodal.flatten(), separator=', '))

# ## butterborth
# coeffs = [1, 2.61*w0, 3.41*w0**2, 2.61*w0**3, w0**4]
# poles = np.roots(coeffs)
# K1_b = np.array([ct.place_acker(A, B, poles)])

# ## newton
# coeffs = [1, 4*w0, 6*w0**2, 4*w0**3, w0**4]
# poles = np.roots(coeffs)
# K1_n = np.array([ct.place_acker(A, B, poles)])


"tests"

sys = ct.ss(A, B, C, D)

t = np.linspace(0, 5, 500)
x0 = np.array([0.7, 0, 0, 0])

sys_cl_modal = ct.ss(A - B @ Kmodal, B, C, D)
t_out_m, y_out_m = ct.initial_response(sys_cl_modal, t, x0)

sys_cl_lqr = ct.ss(A - B @ Klqr, B, C, D)
t_out_lqr, y_out_lqr = ct.initial_response(sys_cl_lqr, t, x0)

sys_cl_mpc = ct.ss(A - B @ Kmpc, B, C, D)
t_out_mpc, y_out_mpc = ct.initial_response(sys_cl_mpc, t, x0)



"""
Plot modal control vs lqr
"""

fig, axes = plt.subplots(5, 1, figsize=(10, 12))
for ax in axes: ax.grid(True)

axes[0].plot(t_out_m, y_out_m[0], t_out_lqr, y_out_lqr[0], t_out_mpc, y_out_mpc[0])
axes[0].set_ylabel('psi')
axes[0].legend(["modal", "lqr", "mpc"])

axes[1].plot(t_out_m, y_out_m[2], t_out_lqr, y_out_lqr[2], t_out_mpc, y_out_mpc[2])
axes[1].set_ylabel('dpsi')

axes[2].plot(t_out_m, y_out_m[3], t_out_lqr, y_out_lqr[3], t_out_mpc, y_out_mpc[3])
axes[2].set_ylabel('theta')

axes[3].plot(t_out_m, y_out_m[1], t_out_lqr, y_out_lqr[1], t_out_mpc, y_out_mpc[1])
axes[3].set_ylabel('dtheta')

u_m = np.zeros(len(t_out_m))
u_lqr = np.zeros(len(t_out_m))
u_mpc = np.zeros(len(t_out_m))
for i in range(len(t_out_m)):
    x = y_out_m[:, i].flatten() 
    u_m[i] = -Kmodal.flatten() @ x

    x = y_out_lqr[:, i].flatten() 
    u_lqr[i] = -Klqr.flatten() @ x

    x = y_out_lqr[:, i].flatten() 
    u_mpc[i] = -Kmpc.flatten() @ x


axes[4].plot(t_out_m, u_m, t_out_lqr, u_lqr, t_out_mpc, u_mpc)
axes[4].set_ylabel('u')

plt.tight_layout()
plt.show()