import numpy as np
import matplotlib.pyplot as plt

# Parameters
h = 0.2
g = 9.81
dt = 0.01
T = 5.0

# PD gains
kpx, kdx = 2.0, 1.0
kpy, kdy = 2.0, 1.0
kp_phi, kd_phi = 1.0, 0.5

# Desired trajectory
x_des = lambda t: 2 * np.cos(0.5 * t)
y_des = lambda t: 2 * np.sin(0.5 * t)
phi_des = lambda t: 0.0

xdot_des = lambda t: -1 * np.sin(0.5 * t)
ydot_des = lambda t: 1 * np.cos(0.5 * t)
phidot_des = lambda t: 0.0

xddot_des = lambda t: -0.5 * np.cos(0.5 * t)
yddot_des = lambda t: -0.5 * np.sin(0.5 * t)
phiddot_des = lambda t: 0.0

# Initial states
x, y, psi, phi, v, omega = x_des(0), y_des(0), np.pi / 2, 0, 0.1, 0.0

x_hist, y_hist, phi_hist, psi_hist, v_hist, omega_hist = [], [], [], [], [], []

# Simulation loop
for t in np.arange(0, T, dt):
    # Desired values
    x_d, y_d, phi_d = x_des(t), y_des(t), phi_des(t)
    xdot_d, ydot_d, phidot_d = xdot_des(t), ydot_des(t), phidot_des(t)
    xddot_d, yddot_d, phiddot_d = xddot_des(t), yddot_des(t), phiddot_des(t)

    # Errors
    ex, ey, ephi = x - x_d, y - y_d, phi - phi_d
    evx, evy = v * np.cos(psi) - xdot_d, v * np.sin(psi) - ydot_d
    ephidot = (h / g) * v * omega - phidot_d

    # Virtual inputs (PD control with desired accelerations)
    nu1 = xddot_d - kpx * ex - kdx * evx
    nu2 = yddot_d - kpy * ey - kdy * evy
    nu3 = phiddot_d - kp_phi * ephi - kd_phi * ephidot

    # Decoupling matrix
    A = np.array([[np.cos(psi), -v * np.sin(psi)],
                  [np.sin(psi), v * np.cos(psi)]])

    # Solve for (vdot, omegadot)
    u, residuals, rank, s = np.linalg.lstsq(A, np.array([nu1, nu2]), rcond=None)
    vdot, omegadot = u
    vdot, omegadot = u

    # Update states
    v += vdot * dt
    omega += omegadot * dt
    x += v * np.cos(psi) * dt
    y += v * np.sin(psi) * dt
    psi += omega * dt
    phi += (h / g) * v * omega * dt

    # Store values
    x_hist.append(x)
    y_hist.append(y)
    phi_hist.append(phi)
    psi_hist.append(psi)
    v_hist.append(v)
    omega_hist.append(omega)

# Plot results
plt.figure(figsize=(12, 8))

plt.subplot(2, 2, 1)
plt.plot(x_hist, y_hist, label='Trajectory')
plt.plot([x_des(t) for t in np.arange(0, T, dt)], [y_des(t) for t in np.arange(0, T, dt)], '--', label='Desired')
plt.xlabel('x')
plt.ylabel('y')
plt.legend()
plt.title('Trajectory Tracking')

plt.subplot(2, 2, 2)
plt.plot(np.arange(0, T, dt), phi_hist, label='Roll Angle (phi)')
plt.xlabel('Time (s)')
plt.ylabel('Roll Angle (rad)')
plt.title('Roll Angle Evolution')
plt.grid()

plt.subplot(2, 2, 3)
plt.plot(np.arange(0, T, dt), v_hist, label='Velocity (v)')
plt.xlabel('Time (s)')
plt.ylabel('Velocity (m/s)')
plt.title('Velocity Profile')
plt.grid()

plt.subplot(2, 2, 4)
plt.plot(np.arange(0, T, dt), omega_hist, label='Angular Velocity (omega)')
plt.xlabel('Time (s)')
plt.ylabel('Angular Velocity (rad/s)')
plt.title('Angular Velocity Profile')
plt.grid()

plt.tight_layout()
plt.show()