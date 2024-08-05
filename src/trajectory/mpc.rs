use nalgebra::{Vector2, Vector3};
use optimization_engine::{constraints::*, panoc::*, *};

pub fn compute() {
    let waypoints = [
        Vector2::new(1.0, 1.0),
        Vector2::new(6.0, 5.0),
        Vector2::new(3.0, 2.0),
        Vector2::new(1.0, 1.0),
    ];
    let (nu, nx, N, L, ts) = (2, 3, 200, 0.5, 0.1);
    let (xref, yref, thetaref) = (1.0, 1.0, 0.0);
    let (q, qtheta, r, qN, qthetaN) = (10.0, 0.1, 1.0, 200.0, 2.0);
    let (x0, y0, theta0) = (0.0, 0.0, 0.0);
    let t_start = 0;
    let t_end = 100;
    let sampling_rate = 1;
    let horizon = (t_end - t_start) / sampling_rate;
    let tolerance = 1e-6;
    let lbfgs_memory = 10;
    let max_iters = 100;
    let mut u = vec![0.0; nu * N];

    // the cache is created only ONCE
    let mut panoc_cache = PANOCCache::new(nu * N, tolerance, lbfgs_memory);

    let cost = |u: &[f64], c: &mut f64| -> Result<(), SolverError> {
        *c = 0.0;
        let (mut x, mut y, mut theta): (f64, f64, f64) = (x0, y0, theta0);
        for t in (0..nu * N).step_by(2) {
            *c +=
                q * ((x - xref).powi(2) + (y - yref).powi(2)) + qtheta * (theta - thetaref).powi(2);
            let u_t = &u[t..t + 2];
            let theta_dot = (1.0 / L) * (u_t[1] * theta.cos() - u_t[0] * theta.sin());
            *c += r * Vector2::new(u_t[0], u_t[1]).norm_squared();
            x += ts * (u_t[0] + L * theta.sin() * theta_dot);
            y += ts * (u_t[1] - L * theta.cos() * theta_dot);
            theta += ts * theta_dot;
        }

        *c += qN * ((x - xref).powi(2) + (y - yref).powi(2)) + qthetaN * (theta - thetaref).powi(2);

        Ok(())
    };

    let grad = |u: &[f64], grad: &mut [f64]| -> Result<(), SolverError> {
        grad.iter_mut().for_each(|g| *g = 0.0); // Initialize the gradient vector to zero

        let (mut x, mut y, mut theta): (f64, f64, f64) = (x0, y0, theta0);

        for t in (0..nu * N).step_by(2) {
            let u_t = &u[t..t + 2]; // Control input slice

            // Compute state cost gradients
            let dx = 2.0 * q * (x - xref);
            let dy = 2.0 * q * (y - yref);

            // Compute dynamics derivatives
            let theta_dot = (1.0 / L) * (u_t[1] * theta.cos() - u_t[0] * theta.sin());
            let dtheta_dot_du0 = -(1.0 / L) * theta.sin();
            let dtheta_dot_du1 = (1.0 / L) * theta.cos();

            let dx_du0 = ts * (1.0 + L * theta.sin() * dtheta_dot_du0);
            let dx_du1 = ts * L * theta.sin() * dtheta_dot_du1;
            let dy_du0 = ts * (-L * theta.cos() * dtheta_dot_du0);
            let dy_du1 = ts * (1.0 - L * theta.cos() * dtheta_dot_du1);

            // Accumulate gradient for control effort cost
            grad[t] += r * 2.0 * u_t[0];
            grad[t + 1] += r * 2.0 * u_t[1];

            // Accumulate gradient for state cost
            grad[t] += dx * dx_du0 + dy * dy_du0;
            grad[t + 1] += dx * dx_du1 + dy * dy_du1;

            // Update state
            x += ts * (u_t[0] + L * theta.sin() * theta_dot);
            y += ts * (u_t[1] - L * theta.cos() * theta_dot);
            theta += ts * theta_dot;
        }

        // Terminal cost gradient
        let dx = 2.0 * qN * (x - xref);
        let dy = 2.0 * qN * (y - yref);

        for t in (0..nu * N).step_by(2) {
            // Compute dynamics derivatives
            let dtheta_dot_du0 = -(1.0 / L) * theta.sin();
            let dtheta_dot_du1 = (1.0 / L) * theta.cos();

            let dx_du0 = ts * (1.0 + L * theta.sin() * dtheta_dot_du0);
            let dx_du1 = ts * L * theta.sin() * dtheta_dot_du1;
            let dy_du0 = ts * (-L * theta.cos() * dtheta_dot_du0);
            let dy_du1 = ts * (1.0 - L * theta.cos() * dtheta_dot_du1);

            // Accumulate gradient for terminal cost
            grad[t] += dx * dx_du0 + dy * dy_du0;
            grad[t + 1] += dx * dx_du1 + dy * dy_du1;
        }

        Ok(())
    };

    let umin = vec![-3.0; nu * N];
    let umax = vec![3.0; nu * N];

    let bounds = constraints::Rectangle::new(Some(&umin), Some(&umax));

    // the problem definition is updated at every iteration
    let problem = Problem::new(&bounds, grad, cost);

    // updated instance of the solver
    let mut panoc = PANOCOptimizer::new(problem, &mut panoc_cache).with_max_iter(10000);

    let status = panoc.solve(&mut u).unwrap();

    println!("{status:?}");
}
