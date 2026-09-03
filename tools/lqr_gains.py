#!/usr/bin/env python3
"""
Compute starting balance gains for the wheeled-bipedal robot from a
wheeled-inverted-pendulum (WIP) model, using LQR.

Outputs gains in the SKETCH'S units so they drop straight into
  effort = kp*pitch_deg + kd*pitchRate_deg + kv*wheelVel_revs
i.e. kp<->Kp, kd<->Kd, kv<->Kv.

Everything here is an ESTIMATE meant to give a sane STARTING POINT and,
more importantly, the correct RATIO between the three gains. The overall
magnitude depends on the motor's force-per-effort (very uncertain with PWM
voltage control), so expect to scale all three by one factor on the robot.
"""
import numpy as np
from scipy.linalg import solve_continuous_are

# ---------------- Physical parameters (best estimates) ----------------
g   = 9.81            # gravity [m/s^2]
Mt  = 4.5             # total mass [kg]  (measured)
l   = 0.20            # CoM height above wheel axle [m]  (measured)
r   = 0.05            # wheel radius [m]  (100 mm dia)

m   = 4.0             # body (pendulum) mass [kg]  (most of the mass)
# Effective translational "cart" mass = wheels + reflected wheel & motor inertia.
# Reflected motor rotor inertia through the 50:1 gearbox dominates:
Jrotor = 1.0e-6       # single motor rotor inertia [kg m^2] (rough for 37D class)
N      = 50.0         # gear ratio
m_w    = 0.4          # combined wheel mass [kg]
I_w    = 0.5*m_w*r**2 # wheels as disks
M = m_w + I_w/r**2 + 2*(Jrotor*N**2)/r**2   # effective cart mass [kg]

I_b = (1.0/12.0)*m*(0.40**2)   # body pitch inertia about CoM [kg m^2] (~rod, 0.4 m)

print(f"Effective cart mass M = {M:.3f} kg,  body I = {I_b:.4f} kg m^2")

# ---------------- Linearized WIP state space -------------------------
# state x = [theta (rad), theta_dot (rad/s), v (m/s)],  input u = ground force F [N]
p = I_b*(M+m) + M*m*l**2
A = np.array([
    [0.0,               1.0, 0.0],
    [m*g*l*(M+m)/p,     0.0, 0.0],
    [-m*m*g*l*l/p,      0.0, 0.0],
])
B = np.array([[0.0],
              [-m*l/p],
              [(I_b + m*l*l)/p]])

# ---------------- LQR weights ----------------------------------------
# Penalize tilt hardest, then tilt-rate, then velocity (velocity kept soft so
# the robot can move to balance but still resists runaway).
Q = np.diag([60.0, 1.0, 3.0])   # [theta, theta_dot, v]
R = np.array([[1.0]])

S = solve_continuous_are(A, B, Q, R)
K = np.linalg.inv(R) @ B.T @ S          # u = -K x, K = [k_theta, k_thetadot, k_v] in SI
k_th, k_thd, k_v = K.flatten()
print(f"\nLQR K (SI, force units): k_theta={k_th:.2f} N/rad  "
      f"k_thetadot={k_thd:.2f} N/(rad/s)  k_v={k_v:.2f} N/(m/s)")

# ---------------- Convert to sketch units ----------------------------
# effort = F / F_scale ; F_scale ~ peak ground force per unit effort.
# Rough: both wheels ~47 N near stall. At balancing speeds real force is lower,
# so treat F_scale as uncertain -> the overall gain scale is a tuning knob.
F_scale = 47.0
DEG = np.pi/180.0          # deg -> rad
REVS = 2*np.pi*r           # rev/s -> m/s  (= 0.314)

kp = k_th  * DEG  / F_scale     # per degree of pitch
kd = k_thd * DEG  / F_scale     # per deg/s of pitch rate
kv = k_v   * REVS / F_scale     # per rev/s of wheel speed

print(f"\nGains in SKETCH units (effort per unit):")
print(f"  kp = {kp:.4f}   (effort per degree)")
print(f"  kd = {kd:.4f}   (effort per deg/s)")
print(f"  kv = {kv:.4f}   (effort per rev/s)")
print(f"\nRATIO kp : kd : kv = 1 : {kd/kp:.3f} : {kv/kp:.3f}")
print("Use the RATIO; then scale all three by one factor until it balances.")
