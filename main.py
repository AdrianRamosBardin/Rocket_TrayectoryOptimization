#Import Optimization Library
import casadi
from casadi import *

#Import basic Python Libraries
import numpy as np
from matplotlib import pyplot as plt

#CONSTANTS
g = 9.8
m = 100000 # kg

min_thrust = 880 * 1000 # N
max_thrust = 1 * 2210 * 1000 #kN

length = 50 # m 
width = 10

I = (1/12) * m * length**2

#Define Physical Constrains of the System

deg_to_rad = 0.0174533
max_gimble = 20  * deg_to_rad
min_gimble = -max_gimble



#Define Non-Linear State equation
def x_dot(x, u):
    theta = x[4]
    
    thrust = u[0]
    thrust_angle = u[1]
    
    # Horizontal force
    F_x = max_thrust * thrust * sin(thrust_angle + theta)
    x_dot = x[1]
    x_dotdot = (F_x) / m
        
    # Vertical force
    F_y = max_thrust * thrust * cos(thrust_angle + theta)
    y_dot = x[3]
    y_dotdot = (F_y) / m - g
    
    # Torque
    T = -length/2 * max_thrust * thrust * sin(thrust_angle)
    theta_dot = x[5]
    theta_dotdot = T / I
    
    return [x_dot, x_dotdot, y_dot, y_dotdot, theta_dot, theta_dotdot]




# Make an optimization problem
opti = casadi.Opti()

# Set the number of steps and the timestep (dt)
steps = 400
t_step = 0.04 

# Generate the array of state and control vectors
x = opti.variable(steps, 6) # 6 Dof System
u = opti.variable(steps, 2) # 2 DoF Control 

#Checking for controllability in non linear systems is not a trivial task, so we will omit it
#   -> In linear Systems you can check is the system is controllable and in wich of the deegres of the system is ans is not controllable
#   -> Another cool thing you can do is use SVD for cheching controlability, with this you can check how controllable a DoF is with the current actuators

#INITIAL CONDITIONS FOR OPTIMIZATION
x[0, :] = [0, 0, 1000, -80, -np.pi/2, 0]
x[steps-1, :] = [0, 0, 0, 0, 0, 0]


# COST FUNCTION: Squared Error (Simple, more complex functions could be used for convex optimizaton problems)
opti.minimize(sumsqr(u[:, 0]) +  sumsqr(u[:, 1]) + 2 * sumsqr(x[:, 5]))

# Set dynamics constraints
for i in range(0, steps-1):
    opti.subject_to( x[i+1, 0] - x[i, 0] == x_dot(x[i, :], u[i, :])[0] * t_step )
    opti.subject_to( x[i+1, 1] - x[i, 1] == x_dot(x[i, :], u[i, :])[1] * t_step )
    
    opti.subject_to( x[i+1, 2] - x[i, 2] == x_dot(x[i, :], u[i, :])[2] * t_step )
    opti.subject_to( x[i+1, 3] - x[i, 3] == x_dot(x[i, :], u[i, :])[3] * t_step )
    
    opti.subject_to( x[i+1, 4] - x[i, 4] == x_dot(x[i, :], u[i, :])[4] * t_step )
    opti.subject_to( x[i+1, 5] - x[i, 5] == x_dot(x[i, :], u[i, :])[5] * t_step )

# Set bounds constraints
for i in range(0, steps):
    opti.subject_to( opti.bounded(0.4, u[i, 0], 1))
    opti.subject_to( opti.bounded(min_gimble, u[i, 1], max_gimble))

# Select solver
opti.solver('ipopt')

#Solving...
sol = opti.solve()

duration = sol.value(t_step) * steps
T = np.linspace(0, duration, num = steps) #Time vector for ploting

# Plot state
plt.subplot(211)
plt.plot(T, sol.value(x)[:, 0], label = "x")
plt.plot(T, sol.value(x)[:, 1], label = "x_dot")
plt.plot(T, sol.value(x)[:, 2], label = "y")
plt.plot(T, sol.value(x)[:, 3], label = "y_dot")
plt.plot(T, sol.value(x)[:, 4], label = "theta")
plt.plot(T, sol.value(x)[:, 5], label = "theta_dot")
plt.legend()
#plt.xlabel("Time [s]")
#plt.title("State Variables")

# Plot control input
plt.subplot(212)
plt.plot(T, sol.value(u)[:, 0], label = "thrust %")
plt.plot(T, sol.value(u)[:, 1], label = "angle")
plt.legend()
plt.xlabel("Time [s]")
#plt.title("Control Inputs")
plt.show()

duration = sol.value(t_step) * steps

print("The optimization lasted: " + str(duration) + " [s]")

