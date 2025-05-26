"""

New Dynamics Model

@spencer-boebel

5-23-25

"""

import numpy as np
import sympy as sp

T = sp.Symbol('T')
l = sp.Symbol('l')
theta = sp.Symbol('theta') # wold angle
phi = sp.Symbol('phi') # world swing
alpha = sp.Symbol('alpha') # gimbal angle
beta = sp.Symbol('beta') # gimbal swing
psi = sp.Symbol('psi') # roll swing factor

I = sp.Matrix([[sp.Symbol(f'I{i}{j}') for j in range(3)] for i in range(3)])

psi_beta_0 = alpha*sp.sin(beta+psi)
psi_beta_1 = alpha*sp.cos(beta+psi)

nu_phi_0 = theta*sp.sin(phi)
nu_phi_1 = theta*sp.cos(phi)

x_ddot_0 = -T*sp.sin(psi_beta_0 + nu_phi_1)*sp.cos(psi_beta_1 + nu_phi_0)
x_ddot_1 = -T*sp.cos(psi_beta_0 + nu_phi_1)*sp.sin(psi_beta_1 + nu_phi_0)
x_ddot_2 = -T*sp.cos(psi_beta_0 + nu_phi_1)*sp.cos(psi_beta_1 + nu_phi_0)

t_xstar_1 = T*l*sp.cos(psi_beta_1)*sp.sin(psi_beta_0)
t_xstar_0 = T*l*sp.sin(psi_beta_1)*sp.cos(psi_beta_0)
t_xstar_2 = 0

t_bf = sp.Matrix([t_xstar_0, t_xstar_1, t_xstar_2])

alpha_bf = I.inv() * t_bf

ddot_theta_0 = alpha_bf[0]*sp.cos(phi) - alpha_bf[1]*sp.sin(phi)*sp.cos(theta)
ddot_theta_1 = alpha_bf[0]*sp.sin(phi) - alpha_bf[1]*sp.cos(phi)*sp.cos(theta)
ddot_theta_2 = alpha_bf[1]*sp.sin(theta)


