#!/usr/bin/env python3
import sympy as sp
import pandas as pd
import numpy as np
from sympy import cse, lambdify

# Define state variables
p1, p2, p3 = sp.symbols('p1 p2 p3')   # Position
v1, v2, v3 = sp.symbols('v1 v2 v3')   # Velocity
q1, q2, q3, q4 = sp.symbols('q1 q2 q3 q4')  # Quaternion components
w1, w2, w3 = sp.symbols('w1 w2 w3')   # Angular velocity

# Define input variables (actuator inputs)
u1, u2, u3, u4, u5, u6 = sp.symbols('u1 u2 u3 u4 u5 u6')

# Define time step
dt = sp.symbols('dt')

# State vector
x = sp.Matrix([p1, p2, p3, v1, v2, v3, q1, q2, q3, q4, w1, w2, w3])

# Input vector
u = sp.Matrix([u1, u2, u3, u4, u5, u6])

# Define the discrete-time state update equations (replace with actual equations)
p_I_next = sp.Matrix([p1 + v1 * dt, p2 + v2 * dt, p3 + v3 * dt])
v_I_next = sp.Matrix([v1, v2, v3])  # Replace with actual v_I_dot equation and integrate
q_next = sp.Matrix([q1, q2, q3, q4])  # Replace with actual quaternion update equation and normalize
w_B_next = sp.Matrix([w1, w2, w3])  # Replace with actual w_B_dot equation and integrate

# Concatenate the next state equations
f = sp.Matrix.vstack(p_I_next, v_I_next, q_next, w_B_next)

# Compute the Jacobians
A_d = f.jacobian(x)
B_d = f.jacobian(u)

# Display the matrices
sp.pprint(A_d)
sp.pprint(B_d)
