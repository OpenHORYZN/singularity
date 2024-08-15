import opengen as og
import casadi.casadi as cs

# Nonconvex MPC for trajectory following

(nu, nx, N, ts) = (3, 3, 10, 1)

# q = Gain stepwise position deviation cost
# r = Gain acceleration cost
# qN = Gain terminal position deviation cost
(q, r, qN) = (0, 1, 1)

u = cs.SX.sym('u', nu*N)
z0 = cs.SX.sym('z0', nx + nx + nx)

(x, y, z, xref, yref, zref, v0x, v0y, v0z) = (z0[0], z0[1], z0[2], z0[3], z0[4], z0[5], z0[6], z0[7], z0[8])

cost = 0
# LCI = last control input
lci = [v0x, v0y, v0z]

for t in range(0, nu*N, nu):
    cost += q*((x-xref)**2 + (y-yref)**2 + (z-zref)**2)
    u_t = u[t:t+3]
    dx = u_t[0]-lci[0]
    dy = u_t[1]-lci[1]
    dz = u_t[2]-lci[2]
    cost += r * (dx**2 + dy**2 + dz**2)
    x += ts * u_t[0] 
    y += ts * u_t[1]
    z += ts * u_t[2]
    lci[0] = u_t[0]
    lci[1] = u_t[1]
    lci[2] = u_t[2]


u0 = u[0:3]
cost += qN*((x-xref)**2 + (y-yref)**2 + (z-zref)**2)

f1 = cs.vertcat(u0[0] - v0x, u0[1] - v0y, u0[2] - v0z)

to_zero = og.constraints.Zero()
max_accel = og.constraints.BallInf(None, 0.4)
# set_c = og.constraints.CartesianProduct([2, nu*N + 2], [to_zero, max_accel])

umin = [-2.0] * (nu*N)
umax = [2.0] * (nu*N)
bounds = og.constraints.Rectangle(umin, umax)

problem = og.builder.Problem(u, z0, cost) \
    .with_constraints(bounds) \
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