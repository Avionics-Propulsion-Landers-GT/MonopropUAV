#include "../CustomLinear/Matrix.h"
#include "../CustomLinear/Vector.h"
#include <vector>

inline double sign(double x) {
    return (x > 0) - (x < 0);
}

/**
 * @brief Computes the continuous-time B matrix of the nonlinear system linearized around a state/input in the body frame.
 *
 * @param m        Mass of the vehicle (scalar)
 * @param f        Air density (scalar)
 * @param cDrag    Coefficient of drag (scalar)
 * @param areaVar  Reference cross-sectional area (scalar)
 * @param full_state  State vector (12x1): [r; v; angles; angular_rates]
 * @param full_input  Input vector (7x1): [T; a; b; a_dot; b_dot; a_ddot; b_ddot]
 * @param rc       Vector from COM to COP (3x1)
 * @param rt       Vector from COM to thrust application point (3x1)
 * @param inertia      Inertia tensor of full vehicle (3x3)
 * @param inertia_a    Inertia tensor of TVC element a (3x3)
 * @param inertia_b    Inertia tensor of TVC element b (3x3)
 * @return Matrix  B matrix (12x7)
 */
Matrix calculateBBF(
    double m,
    double f,
    double cDrag,
    double areaVar,
    const Vector& full_state,
    const Vector& full_input,
    const Vector& rc,
    const Vector& rt,
    const Matrix& inertia,
    const Matrix& inertia_a,
    const Matrix& inertia_b,
    const Vector& angular_states
) {
    // Extract inertia values
    double Ixx = inertia(0,0);
    double Ixy = inertia(0,1);
    double Ixz = inertia(0,2);
    double Iyy = inertia(1,1);
    double Iyz = inertia(1,2);
    double Izz = inertia(2,2);

    double Ixx_a = inertia_a(0,0);
    double Ixy_a = inertia_a(0,1);
    double Ixz_a = inertia_a(0,2);

    double Ixx_b = inertia_b(0,0);
    double Ixy_b = inertia_b(0,1);
    double Ixz_b = inertia_b(0,2);

    double Iyy_b = inertia_b(1,1);
    double Iyz_b = inertia_b(1,2);

    // Extract inputs
    double T = -full_input[0];
    double a = full_input[1];
    double b = full_input[2];
    double a_dot = angular_states[0];
    double b_dot = angular_states[1];
    double a_ddot = angular_states[2];
    double b_ddot = angular_states[3];

    // Extract states
    double r_x = full_state[0];
    double r_y = full_state[1];
    double r_z = full_state[2];

    double v_x = full_state[3];
    double v_y = full_state[4];
    double v_z = full_state[5];

    double psiVar = full_state[6];
    double thetaVar = full_state[7];
    double phiVar = full_state[8];

    double psiVar_dot = full_state[9];
    double thetaVar_dot = full_state[10];
    double phiVar_dot = full_state[11];

    double rt_x = rt[0];
    double rt_y = rt[1];
    double rt_z = rt[2];

    double t2 = cos(a);
    double t3 = cos(b);
    double t4 = m;
    double t5 = sin(a);
    double t6 = sin(b);
    double t7 = pow(Ixy, 2);
    double t8 = pow(Ixz, 2);
    double t9 = pow(Iyz, 2);
    double t10 = Ixy * Ixz;
    double t11 = Ixx * Iyy;
    double t12 = Ixx * Iyz;
    double t13 = Ixy * Iyz;
    double t14 = Ixz * Iyy;
    double t15 = Ixz * Iyz;
    double t16 = Ixx * Izz;
    double t17 = Ixy * Izz;
    double t18 = Iyy * Izz;
    double t19 = Izz * t11;
    double t20 = rt_x * t2;
    double t21 = rt_z * t2;
    double t22 = rt_x * t5;
    double t23 = rt_z * t5;
    double t24 = rt_x * t6;
    double t25 = rt_z * t6;
    double t26 = 1.0 / t4; 
    double t27 = Iyz * t10 * 2.0;
    double t28 = Ixx * t9;
    double t29 = Iyy * t8;
    double t30 = Izz * t7;
    double t31 = T * rt_x * t3;
    double t32 = T * rt_z * t3;
    double t33 = -t11;
    double t34 = -t12;
    double t35 = -t14;
    double t36 = -t16;
    double t37 = -t17;
    double t38 = -t18;
    double t41 = rt_y * t2 * t3;
    double t42 = rt_y * t3 * t5;
    double t44 = T * rt_y * t2 * t6;
    double t45 = T * rt_y * t5 * t6;
    double t39 = -t27;
    double t40 = -t19;
    double t43 = -t22;
    double t46 = -t42;
    double t47 = t10 + t34;
    double t48 = t13 + t35;
    double t49 = t15 + t37;
    double t50 = t20 + t23;
    double t51 = -t44;
    double t52 = t7 + t33;
    double t53 = t8 + t36;
    double t54 = t9 + t38;
    double t55 = t25 + t41;
    double t57 = t31 + t45;
    double t56 = t21 + t43;
    double t58 = t24 + t46;
    double t59 = t32 + t51;
    double t60 = t28 + t29 + t30 + t39 + t40;
    double t61 = 1.0 / t60;
    
    std::vector<double> mt1 = {
        0.0, 0.0, 0.0, t3 * t5 * t26, t6 * t26, -t2 * t3 * t26,
        0.0, 0.0, 0.0,
        t48 * t58 * t61 + t54 * t55 * t61 + t3 * t49 * t50 * t61,
        -t49 * t55 * t61 + t47 * t58 * t61 - t3 * t50 * t53 * t61,   
        -t48 * t55 * t61 - t52 * t58 * t61 + t3 * t47 * t50 * t61,
        0.0, 0.0, 0.0, T * t2 * t3 * t26, 0.0, T * t3 * t5 * t26,
        0.0, 0.0, 0.0,
        -T * t41 * t48 * t61 + T * t46 * t54 * t61 + T * t3 * t49 * t56 * t61,
        -T * t41 * t47 * t61 + T * t42 * t49 * t61 - T * t3 * t53 * t56 * t61,
        T * t42 * t48 * t61 + T * t41 * t52 * t61 + T * t3 * t47 * t56 * t61,
        0.0, 0.0, 0.0, -T * t5 * t6 * t26, T * t3 * t26, T * t2 * t6 * t26,
        0.0, 0.0, 0.0,
        t48 * t57 * t61 + t54 * t59 * t61 - T * t6 * t49 * t50 * t61,
        t47 * t57 * t61 - t49 * t59 * t61 + T * t6 * t50 * t53 * t61
    };
    
    std::vector<double> mt2 = {
        -t48 * t59 * t61 - t52 * t57 * t61 - T * t6 * t47 * t50 * t61,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 
        -Ixx_a * t54 * t61 + Ixy_a * t49 * t61 + Ixz_a * t48 * t61,
        Ixx_a * t49 * t61 - Ixy_a * t53 * t61 + Ixz_a * t47 * t61,
        Ixx_a * t48 * t61 + Ixy_a * t47 * t61 - Ixz_a * t52 * t61,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        -Ixy_b * t54 * t61 + Iyy_b * t49 * t61 + Iyz_b * t48 * t61,
        Ixy_b * t49 * t61 - Iyy_b * t53 * t61 + Iyz_b * t47 * t61,
        Ixy_b * t48 * t61 + Iyy_b * t47 * t61 - Iyz_b * t52 * t61
    };
    
    /* ------------ Final matrix based off previous calculateB ------------ */

    std::vector<double> B_flat;
    B_flat.reserve(12 * 3);

    B_flat.insert(B_flat.end(), mt1.begin(), mt1.end());
    B_flat.insert(B_flat.end(), mt2.begin(), mt2.end());
   

    Matrix B(12, 3, 0.0);
    for (unsigned int i = 0; i < 3; ++i) {
        for (unsigned int j = 0; j < 12; ++j) {
            B(j, i) = B_flat[i * 12 + j];
        }
    }

    return B;
}