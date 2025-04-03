#include "../Matrix.h"
#include "../Vector.h"
#include <vector>
#include <iostream>
#include <cmath>

inline double sign(double x) {
    return (x > 0) - (x < 0);
}

/**
 * @brief Computes the continuous-time B matrix of the nonlinear system linearized around a state/input.
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
Matrix calculateB(
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
    const Matrix& inertia_b
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
    double a_dot = full_input[3];
    double b_dot = full_input[4];
    double a_ddot = full_input[5];
    double b_ddot = full_input[6];

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
    double t4 = m;  // assuming m is real; if complex use std::conj(m)
    double t5 = cos(phiVar);
    double t6 = cos(psiVar);
    double t7 = sin(a);
    double t8 = sin(b);
    double t9 = cos(thetaVar);
    double t10 = sin(phiVar);
    double t11 = sin(psiVar);
    double t12 = sin(thetaVar);
    double t13 = Ixy * Ixy;
    double t14 = Ixz * Ixz;
    double t15 = Iyz * Iyz;
    double t16 = Ixy * Ixz;
    double t17 = Ixx * Iyy;
    double t18 = Ixx * Iyz;
    double t19 = Ixy * Iyz;
    double t20 = Ixz * Iyy;
    double t21 = Ixz * Iyz;
    double t22 = Ixx * Izz;
    double t23 = Ixy * Izz;
    double t24 = Iyy * Izz;
    double t25 = Izz * t17;
    double t26 = Ixz_a * t12;
    double t27 = Iyz_b * t12;
    double t28 = rt_x * t2;
    double t29 = rt_z * t2;
    double t30 = rt_x * t7;
    double t31 = rt_z * t7;
    double t32 = rt_x * t8;
    double t33 = rt_z * t8;
    double t34 = t5 * t6;
    double t35 = t5 * t11;
    double t36 = t6 * t10;
    double t37 = t10 * t11;
    double t38 = 1.0 / t4;
    double t39 = Iyz * t16 * 2.0;
    double t40 = Ixx * t15;
    double t41 = Iyy * t14;
    double t42 = Izz * t13;
    double t43 = T * rt_x * t3;
    double t44 = T * rt_z * t3;
    double t45 = -t17;
    double t46 = -t18;
    double t47 = -t20;
    double t48 = -t22;
    double t49 = -t23;
    double t50 = -t24;
    double t53 = Ixx_a * t5 * t9;
    double t54 = Ixy_b * t5 * t9;
    double t55 = Ixz_a * t6 * t9;
    double t56 = Iyz_b * t6 * t9;
    double t57 = rt_y * t2 * t3;
    double t58 = Ixy_a * t9 * t10;
    double t59 = Iyy_b * t9 * t10;
    double t60 = Ixz_a * t9 * t11;
    double t61 = Iyz_b * t9 * t11;
    double t62 = rt_y * t3 * t7;
    double t67 = T * rt_y * t2 * t8;
    double t68 = T * rt_y * t7 * t8;
    double t51 = -t39;
    double t52 = -t25;
    double t63 = -t26;
    double t64 = -t27;
    double t65 = -t30;
    double t66 = t12 * t37;
    double t69 = t12 * t34;
    double t70 = t12 * t35;
    double t71 = t12 * t36;
    double t72 = -t62;
    double t73 = t16 + t46;
    double t74 = t19 + t47;
    double t75 = t21 + t49;
    double t76 = T * t12 * t57;
    double t77 = t28 + t31;
    double t78 = -t67;
    double t79 = t13 + t45;
    double t80 = t14 + t48;
    double t81 = t15 + t50;
    double t84 = T * t6 * t9 * t57;
    double t85 = t33 + t57;
    double t86 = T * t5 * t9 * t62;
    double t87 = T * t9 * t11 * t57;
    double t89 = t43 + t68;
    double t82 = -t70;
    double t83 = -t71;
    double t88 = t29 + t65;
    double t90 = t32 + t72;
    double t91 = t34 + t66;
    double t92 = t37 + t69;
    double t93 = t44 + t78;
    double t100 = t12 * t89;
    double t101 = t5 * t9 * t85;
    double t102 = t3 * t9 * t10 * t77;
    double t108 = T * t8 * t9 * t10 * t77;
    double t109 = t6 * t9 * t89;
    double t110 = t9 * t11 * t89;
    double t118 = t53 + t58 + t63;
    double t119 = t54 + t59 + t64;
    double t134 = t40 + t41 + t42 + t51 + t52;
    double t94 = Ixx_a * t92;
    double t95 = Ixy_a * t91;
    double t96 = Ixy_b * t92;
    double t97 = Iyy_b * t91;
    double t98 = t35 + t83;
    double t99 = t36 + t82;
    double t103 = t12 * t90;
    double t111 = -t102;
    double t112 = t6 * t9 * t90;
    double t113 = t9 * t11 * t90;
    double t120 = T * t3 * t9 * t10 * t88;
    double t121 = t5 * t9 * t93;
    double t125 = T * t72 * t92;
    double t126 = t3 * t77 * t91;
    double t127 = t85 * t92;
    double t128 = T * t8 * t77 * t91;
    double t131 = T * t3 * t88 * t91;
    double t135 = t92 * t93;
    double t138 = 1.0 / t134;
    double t104 = Ixx_a * t99;
    double t105 = Ixy_a * t98;
    double t106 = Ixy_b * t99;
    double t107 = Iyy_b * t98;
    double t123 = -t112;
    double t124 = T * t62 * t99;
    double t129 = t3 * t77 * t98;
    double t130 = t85 * t99;
    double t132 = T * t8 * t77 * t98;
    double t133 = -t128;
    double t136 = T * t3 * t88 * t98;
    double t137 = -t131;
    double t139 = t93 * t99;
    double t140 = -t135;
    double t141 = t76 + t86 + t120;
    double t146 = t101 + t103 + t111;
    double t147 = t100 + t108 + t121;
    double t114 = -t104;
    double t115 = -t105;
    double t116 = -t106;
    double t117 = -t107;
    double t148 = t84 + t125 + t136;
    double t149 = t87 + t124 + t137;
    double t150 = t113 + t126 + t130;
    double t151 = t123 + t127 + t129;
    double t152 = t110 + t133 + t139;
    double t153 = t109 + t132 + t140;
    double t142 = t60 + t95 + t114;
    double t143 = t61 + t97 + t116;
    double t144 = t55 + t94 + t115;
    double t145 = t56 + t96 + t117;
    
    // --- Placeholder ---
    // Use all intermediate terms defined from symbolic output to fill mt1, mt2, mt3 like before
    std::vector<double> mt1 = {
        0.0, 0.0, 0.0,
        -t38 * (t2 * t3 * t12 + t8 * t9 * t10 + t3 * t5 * t7 * t9),
        t38 * (-t8 * t91 + t3 * t7 * t99 + t2 * t3 * t9 * t11),
        t38 * (t8 * t98 - t3 * t7 * t92 + t2 * t3 * t6 * t9),
        0.0, 0.0, 0.0,
        -t74 * t138 * t151 + t75 * t138 * t150 + t81 * t138 * t146,
        -t75 * t138 * t146 - t73 * t138 * t151 - t80 * t138 * t150,
        -t74 * t138 * t146 + t73 * t138 * t150 + t79 * t138 * t151,
        0.0, 0.0, 0.0,
        t38 * (T * t3 * t7 * t12 - T * t2 * t3 * t5 * t9),
        t38 * (T * t2 * t3 * t99 - T * t3 * t7 * t9 * t11),
        -t38 * (T * t2 * t3 * t92 + T * t3 * t6 * t7 * t9),
        0.0, 0.0, 0.0,
        -t74 * t138 * t148 - t81 * t138 * t141 - t75 * t138 * t149,
        t75 * t138 * t141 - t73 * t138 * t148 + t80 * t138 * t149
    };    
    
    std::vector<double> mt2 = {
        t74 * t138 * t141 - t73 * t138 * t149 + t79 * t138 * t148,
        0.0, 0.0, 0.0,
        t38 * (T * t2 * t8 * t12 - T * t3 * t9 * t10 + T * t5 * t7 * t8 * t9),
        -t38 * (T * t3 * t91 + T * t7 * t8 * t99 + T * t2 * t8 * t9 * t11),
        t38 * (T * t3 * t98 + T * t7 * t8 * t92 - T * t2 * t6 * t8 * t9),
        0.0, 0.0, 0.0,
        t74 * t138 * t153 + t75 * t138 * t152 + t81 * t138 * t147,
        -t75 * t138 * t147 + t73 * t138 * t153 - t80 * t138 * t152,
        -t74 * t138 * t147 + t73 * t138 * t152 - t79 * t138 * t153,
        0.0, 0.0, 0.0,
        // Padding zeros to align length
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0, 0, 0, 0, 0, 0,
        -t81 * t118 * t138 + t75 * t138 * t142 + t74 * t138 * t144,
         t75 * t118 * t138 + t73 * t138 * t144 - t80 * t138 * t142
    };

    std::vector<double> mt3 = {
        t74 * t118 * t138 + t73 * t138 * t142 - t79 * t138 * t144,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        -t81 * t119 * t138 + t75 * t138 * t143 + t74 * t138 * t145,
         t75 * t119 * t138 + t73 * t138 * t145 - t80 * t138 * t143,
         t74 * t119 * t138 + t73 * t138 * t143 - t79 * t138 * t145
    };

    std::vector<double> B_flat;
    B_flat.reserve(12 * 7);

    B_flat.insert(B_flat.end(), mt1.begin(), mt1.end());
    B_flat.insert(B_flat.end(), mt2.begin(), mt2.end());
    B_flat.insert(B_flat.end(), mt3.begin(), mt3.end());

    Matrix B(12, 7, 0.0);
    for (unsigned int i = 0; i < 7; ++i) {
        for (unsigned int j = 0; j < 12; ++j) {
            B(j, i) = B_flat[i * 12 + j];
        }
    }

    return B;

}