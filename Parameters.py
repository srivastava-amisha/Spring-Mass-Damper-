# Single link arm Parameter File
import numpy as np
import control as cnt
import sys
sys.path.append('..')  # add parent directory
import Param as P

#  tuning parameters
tr = 2
zeta = 0.7
integrator_pole = -5
wn_obs = 1.1          # natural frequency for observer
zeta_obs = 0.7      # damping ratio for observer
dist_obsv_pole = -5.5  # pole for disturbance observer


Ts = P.Ts  # sample rate of the controller
F_max = P.F_max  # limit on control signal
m = P.m
k = P.k
b = P.b


# State Space Equations
A = np.array([[0.0, 1.0],
               [-P.k/P.m, -P.b/P.m]])

B = np.array([[0.0],
               [1/P.m]])

C = np.array([[1.0, 0.0]])

# control design
# form augmented system
A1 = np.array([[0.0, 1.0, 0.0],
               [-P.k/P.m, -P.b/P.m, 0.0],
               [-1.0, 0.0, 0.0]])

B1 = np.array([[0.0],
               [1/P.m],
               [0.0]])

# gain calculation
wn = 2.2/tr  # natural frequency

#des_char_poly = np.convolve(
#    [1, 2*zeta*wn, wn**2],
#    [1, integrator_pole])
#des_poles = np.roots(des_char_poly)

des_char_poly = np.array([1, 2*zeta*wn_obs, wn_obs**2])
des_poles_poly = np.roots(des_char_poly)
des_poles = np.concatenate((des_poles_poly,
                            integrator_pole*np.ones(1)))

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A1, B1)) != 3:
    print("The system is not controllable")
else:
    K1 = cnt.acker(A1, B1, des_poles)
    K = np.array([K1.item(0), K1.item(1)])
    ki = K1.item(2)

# observer design
# Augmented Matrices
A2 = np.concatenate((
        np.concatenate((A, B), axis=1),
        np.zeros((1, 3))),
        axis=0)
B2 = np.concatenate((B, np.zeros((1, 1))), axis=0)
C2 = np.concatenate((C, np.zeros((1, 1))), axis=1)


des_char_est = np.array([1., 2.*zeta*wn_obs, wn_obs**2.])
des_poles_est = np.roots(des_char_est)
des_obsv_poles = np.concatenate((des_poles_est,
                                 dist_obsv_pole*np.ones(1)))

# Compute the gains if the system is controllable
if np.linalg.matrix_rank(cnt.ctrb(A2.T, C2.T)) != 3:
    print("The system is not observerable")
else:
    L2 = cnt.acker(A2.T, C2.T, des_obsv_poles).T
    L = np.array([[L2.item(0)], [L2.item(1)]])
    Ld = L2.item(0)

print('K: ', K)
print('ki: ', ki)
print('L^T: ', L.T)
print('Ld: ', Ld)

