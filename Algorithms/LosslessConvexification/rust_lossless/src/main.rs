use clarabel::algebra::*;
use clarabel::solver::*;
mod lossless;

fn main() {
    // Problem dimensions
    let n = 2; // number of variables
    let m = 2; // dimension of SOC constraint

    // Objective: (1/2)xᵀ P x + qᵀ x
    let P = CscMatrix::identity(n);
    let q = vec![0.0, 0.0];

    // Constraint: A x + s = b
    // Here, A = I and b = 0 ⇒ s = x ∈ SOC
    let A = CscMatrix::identity(m);
    let b = vec![0.0, 0.0];

    // Define the cone(s)
    // Note: SupportedConeT::SecondOrderConeT takes a single usize (dimension)
    let cones = [SupportedConeT::SecondOrderConeT(m)];

    // Default solver settings
    let settings = DefaultSettings::default();

    // Build and solve problem
    let mut solver = DefaultSolver::new(&P, &q, &A, &b, &cones, settings);
    solver.solve();

    // Retrieve and print solution
    let x = &solver.solution.x;
    println!("Solution x = {:?}", x);
    println!("Solver status: {:?}", solver.info.status);
}