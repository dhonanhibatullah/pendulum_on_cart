from modules.poc_simulator import *
from modules.poc_graphic import *
import numpy as np
import matplotlib.pyplot as plt
import control as ct



# Initial value and parameters
state_init = np.array([
    [0.0],
    [0.0],
    [0.0],
    [0.0]
])
timestep = 0.016667



# Declare the simulator and graphics
poc_simulator = PocSimulator(
    pole_length = 1.0,
    pole_mass   = 1.0,
    pole_inert  = 0.5,
    cart_mass   = 2.0,
    state_init  = state_init,
    timestep    = timestep
)
poc_graphics = PocGraphic()



# Parameters
k1  = poc_simulator.k1
k2  = poc_simulator.k2
mp  = poc_simulator.cart_mass
l   = poc_simulator.pole_length
g   = 9.8



# Linear controller
A = [
    [0, 1, 0, 0],
    [0, 0, (k1*k2*g)/(1.0 - k1*k2), 0],
    [0, 0, 0, 1],
    [0, 0, (k2*g)/(1.0 - k1*k2), 0]
]
B = [
    [0],
    [-k1/(mp*l*(1.0 - k1*k2))],
    [0],
    [-k1*k2/(mp*l*(1.0 - k1*k2))]
]
C = np.eye(4)
D = np.zeros((4, 1))
Q = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
R = np.array([[0.001]])
linear_sys  = ct.ss(A, B, C, D)
K, _, _     = ct.lqr(linear_sys, Q, R)




# Control law
KP  = 1.0
KI  = 1.0
eta = 8.0
k   = 0.0
centerize = False


def calcControlLaw(x:np.ndarray, time:float) -> float:
    global centerize

    # Get the states
    x1 = x[0].item()
    x2 = x[1].item()
    x3 = x[2].item()
    x4 = x[3].item()


    # Calculate control law
    theta, theta_dot, theta_ddot = calcTrajectory(time)
    # if (time < eta or not np.fabs(x1) < 0.1) and not centerize:
    if (time < eta or not np.fabs(x4) < 0.1) and not centerize:
        v   = theta_ddot + KP*(theta - x3) + KI*(theta_dot - x4)
        c1  = (mp*l)/(k1*np.cos(x3))
        c2  = ((1.0 - k1*k2*(np.cos(x3)**2.0))/k2)*v
        c3  = 0.5*k1*np.sin(2.0*x3)*(x4**2.0)
        c4  = g*np.sin(x3)
        u   = c1*(c2 + c3 + c4)
    else:
        centerize = True
        u = -(K@np.array([[x1], [x2], [x3 - np.pi], [x4]]))[0].item()
    return u


def calcTrajectory(time:float) -> float:
    if time > eta: 
        return np.pi, 0.0, 0.0
    else:
        theta       = (-2.0*(time**3.0)/(eta**3.0) + 3.0*(time**2.0)/(eta**2.0))*np.pi*np.cos((2.0*np.pi*k*time)/eta)
        theta_dot   = (-6.0*(time**2.0)/(eta**3.0) + 6.0*time/(eta**2.0))*np.pi*np.cos((2.0*np.pi*k*time)/eta) + (2.0*(time**3.0)/(eta**3.0) - 3.0*(time**2.0)/(eta**2.0))*(2.0*(np.pi**2.0)*k/eta)*np.sin((2.0*np.pi*k*time)/eta)
        theta_ddot  = (8.0*(np.pi**3.0)*(k**2.0)*(time**3.0)/(eta**5.0) - 12.0*(np.pi**3.0)*(k**2.0)*(time**2.0)/(eta**4.0) - 12.0*np.pi*time/(eta**3.0) + 6.0*np.pi/(eta**2.0))*np.cos((2.0*np.pi*k*time)/eta) + (12.0*(time**2.0)/(eta**3.0) - 12.0*time/(eta**2.0))*(2.0*(np.pi**2.0)*k/eta)*np.sin((2.0*np.pi*k*time)/eta)
        return theta, theta_dot, theta_ddot



# Simulation run!
x1_his      = []
x2_his      = []
x3_his      = []
x4_his      = []
u_his       = []
time_his    = []
for i in range(3000):
    
    # Get current state
    time    = poc_simulator.getSimulationTime()
    state   = poc_simulator.getCurrentState()
    x1_his.append(state[0].item())
    x2_his.append(state[1].item())
    x3_his.append(state[2].item())
    x4_his.append(state[3].item())
    time_his.append(time)

    # Render animation
    poc_graphics.stepRender(
        cart_x      = state[0].item(),
        pole_theta  = state[2].item()
    )

    # Update simulation
    u = calcControlLaw(state, time)
    poc_simulator.stepSimulation(u)
    u_his.append(u)



# Plot the result 
fig, ax = plt.subplots(3, 2)

ax[0, 0].plot(time_his, [0.0 for t in time_his], label='target')
ax[0, 0].plot(time_his, x1_his, label='state')
ax[0, 0].set_title('$x_1 (x)$ plot')
ax[0, 0].set_xlabel('time (s)')
ax[0, 0].set_ylabel('$x_1$ (m)')
ax[0, 0].legend()
ax[0, 0].grid()

ax[1, 0].plot(time_his, [0.0 for t in time_his], label='target')
ax[1, 0].plot(time_his, x2_his, label='state')
ax[1, 0].set_title('$x_2 (\dot{x})$ plot')
ax[1, 0].set_xlabel('time (s)')
ax[1, 0].set_ylabel('$x_2$ (m/s)')
ax[1, 0].legend()
ax[1, 0].grid()

ax[0, 1].plot(time_his, [np.pi for t in time_his], label='target')
ax[0, 1].plot(time_his, x3_his, label='state')
ax[0, 1].set_title('$x_3 (\\theta)$ plot')
ax[0, 1].set_xlabel('time (s)')
ax[0, 1].set_ylabel('$x_3$ (rad)')
ax[0, 1].legend()
ax[0, 1].grid()

ax[1, 1].plot(time_his, [0.0 for t in time_his], label='target')
ax[1, 1].plot(time_his, x4_his, label='state')
ax[1, 1].set_title('$x_4 (\dot{\\theta})$ plot')
ax[1, 1].set_xlabel('time (s)')
ax[1, 1].set_ylabel('$x_4$ (rad/s)')
ax[1, 1].legend()
ax[1, 1].grid()

ax[2, 0].plot(time_his, u_his)
ax[2, 0].set_title('$u$ plot')
ax[2, 0].set_xlabel('time (s)')
ax[2, 0].set_ylabel('$u$ (N)')
ax[2, 0].grid()

fig.tight_layout()
plt.show()