import numpy as np
import matplotlib.pyplot as plt

def cubic_coeff(qi, qf, vi, vf, ti, tf):
    """Solve cubic polynomial coefficients for one segment."""
    T = tf - ti
    a0 = qi
    a1 = vi
    a2 = (3*(qf-qi) - (2*vi + vf)*T) / (T**2)
    a3 = (2*(qi-qf) + (vi + vf)*T) / (T**3)
    return a0, a1, a2, a3

def blend_trajectory(times, positions, dt=0.01):
    n = len(times)
    # Estimate velocities at waypoints (finite difference)
    v = np.zeros(n)
    for i in range(1, n-1):
        v[i] = (positions[i+1] - positions[i-1]) / (times[i+1] - times[i-1])
    v[0] = 0.0       # start at rest
    v[-1] = 0.0      # end at rest
    
    # Build trajectory
    t_all, q_all, qd_all, qdd_all = [], [], [], []
    for i in range(n-1):
        a0,a1,a2,a3 = cubic_coeff(positions[i], positions[i+1],
                                  v[i], v[i+1], times[i], times[i+1])
        t_segment = np.arange(times[i], times[i+1], dt)
        tau = t_segment - times[i]
        q = a0 + a1*tau + a2*tau**2 + a3*tau**3
        qd = a1 + 2*a2*tau + 3*a3*tau**2
        qdd = 2*a2 + 6*a3*tau
        t_all.extend(t_segment)
        q_all.extend(q)
        qd_all.extend(qd)
        qdd_all.extend(qdd)
    return np.array(t_all), np.array(q_all), np.array(qd_all), np.array(qdd_all)

# Example with 3 waypoints
times = [0, 2, 5]
positions = [0, np.pi/4, np.pi/2]

t, q, qd, qdd = blend_trajectory(times, positions)

# Plot
plt.figure()
plt.plot(t, q, label="Position (rad)")
plt.plot(t, qd, label="Velocity (rad/s)")
plt.plot(t, qdd, label="Acceleration (rad/s²)")
plt.scatter(times, positions, color="red", label="Waypoints")
plt.legend()
plt.xlabel("Time (s)")
plt.title("Cubic Polynomial Blending (No Stop at Waypoints)")
plt.grid()
plt.show()
