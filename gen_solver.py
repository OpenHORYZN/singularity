import opengen as og
import casadi.casadi as cs
import math

# Nonconvex MPC for trajectory following

(nu, nx, N, ts) = (6, 3, 30, 0.1)

# q = Gain stepwise position deviation cost
# r = Gain acceleration cost
# qN = Gain terminal position deviation cost
(q, r, qN) = (10, 1, 10)

u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', nu + 3 + nx*N)
# Z0 is current vel, current acc, current pos, and the path for N timesteps

(v0x, v0y, v0z, a0x, a0y, a0z, p0x, p0y, p0z) = (z0[0], z0[1], z0[2], z0[3], z0[4], z0[5], z0[6], z0[7], z0[8])

cost = 0

uvel = u[0:3*N]
uacc = u[3*N:nu*N]

# lv = last velocity, la = last acceleration, cp = current position (direct shooting)
cp = [p0x, p0y, p0z]
lv = [v0x, v0y, v0z]
la = [a0x, a0y, a0z]

for t in range(0, 3*N, 3):
    p0i = t+9
    (xref, yref, zref) = (z0[p0i], z0[p0i+1], z0[p0i+2])

    cost += q*((cp[0]-xref)**2 + (cp[1]-yref)**2 + (cp[2]-zref)**2)
    cv = uvel[t:t+3]
    ca = uacc[t:t+3]
    cost += 0.1*(ca[0] ** 2 + ca[1] ** 2 + ca[2] ** 2)
    cp[0] += ts * cv[0] 
    cp[1] += ts * cv[1]
    cp[2] += ts * cv[2]
    lv = cv
    la = ca


u0vel = uvel[0:3]
u0acc = uacc[0:3]

f1 = cs.vertcat(u0vel[0] - v0x, u0vel[1] - v0y, u0vel[2] - v0z, u0acc[0] - a0x, u0acc[1] - a0y, u0acc[2] - a0z)

to_zero = og.constraints.Zero()

vmin = [-2.0] * (nu*N)
vmax = [2.0] * (nu*N)
amin = [-1.0] * (3*N)
amax = [1.0] * (3*N)
vbounds = og.constraints.Rectangle(vmin, vmax)
abounds = og.constraints.Rectangle(amin, amax)
bounds = og.constraints.CartesianProduct([3*N, nu*N], [vbounds, abounds])

problem = og.builder.Problem(u, z0, cost) \
    .with_constraints(vbounds) \
    .with_aug_lagrangian_constraints(f1, to_zero)

build_config = og.config.BuildConfiguration()\
    .with_build_directory("solver")\
    .with_build_mode("release")
    
meta = og.config.OptimizerMeta()\
    .with_optimizer_name("navigation")
    
solver_config = og.config.SolverConfiguration()\
    .with_tolerance(1e-5)
    
builder = og.builder.OpEnOptimizerBuilder(problem,
                                          meta,
                                          build_config,
                                          solver_config)
builder.build()