#include "../Matrix.h"
#include "../Vector.h"
#include <math.h>

template <size_t N>
void insert_flat(Matrix& dest, unsigned int& idx, const double (&arr)[N]) {
    for (size_t i = 0; i < N; ++i) {
        dest(idx++, 0) = arr[i];
    }
}

inline double sign(double x) {
    return (x > 0) - (x < 0);
}

/**
 * @brief Computes the continuous-time A and B matrices of the nonlinear system linearized around a state/input.
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
 * @param inertia_s    Inertia tensor of static part (3x3)
 * @param inertia_a    Inertia tensor of TVC element a (3x3)
 * @param inertia_b    Inertia tensor of TVC element b (3x3)
 * @return Matrix A matrix
 */
Matrix calculateA(
    double m,
    double f,
    double cDrag,
    double areaVar,
    const Vector& full_state,   // size 12
    const Vector& full_input,   // size 7
    const Vector& rc,           // size 3
    const Vector& rt,           // size 3
    const Matrix& inertia,      // size 3×3
    const Matrix& inertia_s,
    const Matrix& inertia_a,
    const Matrix& inertia_b
) {



    // Extract from full inertia tensor
    double Ixx   = inertia(0, 0);
    double Ixy   = inertia(0, 1);
    double Ixz   = inertia(0, 2);
    double Iyy   = inertia(1, 1);
    double Iyz   = inertia(1, 2);
    double Izz   = inertia(2, 2);

    // From static component
    double Ixx_s = inertia_s(0, 0);
    double Ixy_s = inertia_s(0, 1);
    double Ixz_s = inertia_s(0, 2);
    double Iyy_s = inertia_s(1, 1);
    double Iyz_s = inertia_s(1, 2);
    double Izz_s = inertia_s(2, 2);

    // From upper TVC inertia
    double Ixx_a = inertia_a(0, 0);
    double Ixy_a = inertia_a(0, 1);
    double Ixz_a = inertia_a(0, 2);
    double Iyy_a = inertia_a(1, 1);
    double Iyz_a = inertia_a(1, 2);
    double Izz_a = inertia_a(2, 2);

    // From lower TVC inertia
    double Ixx_b = inertia_b(0, 0);
    double Ixy_b = inertia_b(0, 1);
    double Ixz_b = inertia_b(0, 2);
    double Iyy_b = inertia_b(1, 1);
    double Iyz_b = inertia_b(1, 2);
    double Izz_b = inertia_b(2, 2);
    
    double T          = full_input[0];
    double a          = full_input[1];
    double b          = full_input[2];
    double a_dot      = full_input[3];
    double b_dot      = full_input[4];
    double a_dot_dot  = full_input[5];
    double b_dot_dot  = full_input[6];

    // Position
    double r_x = full_state[0];
    double r_y = full_state[1];
    double r_z = full_state[2];

    // Velocity
    double v_x = full_state[3];
    double v_y = full_state[4];
    double v_z = full_state[5];

    // Euler Angles (ZYX)
    double psiVar    = full_state[6];  // rotation about x
    double thetaVar  = full_state[7];  // rotation about y
    double phiVar    = full_state[8];  // rotation about z

    // Angular Velocities
    double psiVar_dot   = full_state[9];
    double thetaVar_dot = full_state[10];
    double phiVar_dot   = full_state[11];

    double rc_x = rc[0];
    double rc_y = rc[1];
    double rc_z = rc[2];
    double rt_x = rt[0];
    double rt_y = rt[1];
    double rt_z = rt[2];

    // cos & sin
    double t2  = cos(a);
    double t3  = cos(b);
    double t4  = m;  // Assuming m is a scalar
    double t5  = cos(phiVar);
    double t6  = cos(psiVar);
    double t7  = sin(a);
    double t8  = sin(b);
    double t9  = cos(thetaVar);
    double t10 = sin(phiVar);
    double t11 = sin(psiVar);
    double t12 = sin(thetaVar);

    // Powers (squared values)
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
    double t25 = t9 * t9;
    double t26 = Izz * t17;
    double t27 = Ixz_a * t9;
    double t28 = Ixz_b * t9;
    double t29 = Ixz_s * t9;
    double t30 = Iyz_a * t9;
    double t31 = Iyz_b * t9;
    double t32 = Iyz_s * t9;
    double t33 = Izz_a * t9;
    double t34 = Izz_b * t9;
    double t35 = Izz_s * t9;
    double t36 = Ixz_a * t12;
    double t37 = Ixz_b * t12;
    double t38 = Ixz_s * t12;
    double t39 = Iyz_a * t12;
    double t40 = Iyz_b * t12;
    double t41 = Iyz_s * t12;
    double t42 = Izz_a * t12;
    double t43 = Izz_b * t12;
    double t44 = Izz_s * t12;
    double t45 = rt_x * t2;
    double t46 = rt_z * t7;
    double t47 = t9 * v_x;
    double t48 = rc_x * t12;
    double t49 = rc_y * t12;
    double t50 = t12 * v_x;
    double t51 = t5 * t6;
    double t52 = t5 * t11;
    double t53 = t6 * t10;
    double t56 = t10 * t11;
    double t57 = 1.0 / t4;
    double t60 = 2.0 * Iyz * t16;
    double t61 = Ixx * t15;
    double t62 = Iyy * t14;
    double t63 = Izz * t13;
    double t66 = -t17;
    double t67 = -t18;
    double t68 = -t20;
    double t69 = -t22;
    double t70 = -t23;
    double t71 = -t24;
    double t72 = T * rt_x * t8;
    double t73 = T * rt_z * t8;
    double t76 = Ixx_a * t5 * t9;
    double t77 = Ixx_b * t5 * t9;
    double t78 = Ixx_s * t5 * t9;
    double t79 = Ixy_a * t5 * t9;
    double t80 = Ixy_b * t5 * t9;
    double t81 = Ixy_s * t5 * t9;
    double t85 = Iyy_a * t5 * t9;
    double t86 = Iyy_b * t5 * t9;
    double t87 = Iyy_s * t5 * t9;
    double t100 = Ixx_a * t5 * t12;
    double t101 = Ixx_a * t9 * t10;
    double t102 = Ixx_b * t5 * t12;
    double t103 = Ixx_b * t9 * t10;
    double t104 = Ixx_s * t5 * t12;
    double t105 = Ixx_s * t9 * t10;
    double t106 = Ixy_a * t5 * t12;
    double t107 = Ixy_a * t9 * t10;
    double t108 = Ixy_b * t5 * t12;
    double t109 = Ixy_b * t9 * t10;
    double t110 = Ixy_s * t5 * t12;
    double t111 = Ixy_s * t9 * t10;
    double t118 = Iyy_a * t9 * t10;
    double t119 = Iyy_b * t9 * t10;
    double t120 = Iyy_s * t9 * t10;
    double t142 = a_dot * t5 * t9;
    double t143 = Ixy_a * t10 * t12;
    double t144 = Ixy_b * t10 * t12;
    double t145 = Ixy_s * t10 * t12;
    double t146 = Iyy_a * t10 * t12;
    double t147 = Iyy_b * t10 * t12;
    double t148 = Iyy_s * t10 * t12;
    double t163 = b_dot * t9 * t10;
    double t164 = rc_y * t5 * t9;
    double t165 = rc_z * t5 * t9;
    double t168 = rc_x * t6 * t9;
    double t169 = rc_y * t6 * t9;
    double t171 = t6 * t9 * v_y;
    double t172 = t6 * t9 * v_z;
    double t177 = rc_x * t9 * t10;
    double t178 = rc_z * t9 * t10;
    double t183 = rc_x * t9 * t11;
    double t184 = rc_y * t9 * t11;
    double t196 = t9 * t11 * v_y;
    double t197 = t6 * t12 * v_z;
    double t198 = t9 * t11 * v_z;
    double t204 = t11 * t12 * v_y;
    double t227 = T * rt_y * t2 * t3;
    double t232 = T * rt_y * t3 * t7;
    double t54 = rc_x * t47;
    double t55 = rc_y * t47;
    double t58 = t48 * v_x;
    double t59 = t49 * v_x;
    double t64 = a_dot_dot * t27;
    double t65 = b_dot_dot * t31;
    double t74 = -t60;
    double t75 = -t26;
    double t82 = t5 * t27;
    double t83 = t5 * t28;
    double t84 = t5 * t29;
    double t88 = t5 * t30;
    double t89 = t5 * t31;
    double t90 = t5 * t32;
    double t91 = t6 * t27;
    double t92 = t6 * t28;
    double t93 = t6 * t29;
    double t94 = t6 * t30;
    double t95 = t6 * t31;
    double t96 = t6 * t32;
    double t97 = t6 * t33;
    double t98 = t6 * t34;
    double t99 = t6 * t35;
    double t112 = t5 * t36;
    double t113 = t10 * t27;
    double t114 = t5 * t37;
    double t115 = t10 * t28;
    double t116 = t5 * t38;
    double t117 = t10 * t29;
    double t121 = t10 * t30;
    double t122 = t10 * t31;
    double t123 = t10 * t32;
    double t124 = t6 * t36;
    double t125 = t11 * t27;
    double t126 = t6 * t37;
    double t127 = t11 * t28;
    double t128 = t6 * t38;
    double t129 = t11 * t29;
    double t130 = t6 * t39;
    double t131 = t11 * t30;
    double t132 = t6 * t40;
    double t133 = t11 * t31;
    double t134 = t6 * t41;
    double t135 = t11 * t32;
    double t136 = t6 * t42;
    double t137 = t11 * t33;
    double t138 = t6 * t43;
    double t139 = t11 * t34;
    double t140 = t6 * t44;
    double t141 = t11 * t35;
    double t149 = t10 * t39;
    double t150 = t10 * t40;
    double t151 = t10 * t41;
    double t152 = t11 * t36;
    double t153 = t11 * t37;
    double t154 = t11 * t38;
    double t155 = t11 * t39;
    double t156 = t11 * t40;
    double t157 = t11 * t41;
    double t158 = t11 * t42;
    double t159 = t11 * t43;
    double t160 = t11 * t44;
    double t161 = rc_x * t51;
    double t162 = rc_z * t51;
    double t166 = t51 * v_y;
    double t167 = t51 * v_z;
    double t170 = t5 * t47;
    double t173 = rc_x * t52;
    double t174 = rc_y * t53;
    double t175 = rc_z * t52;
    double t176 = rc_z * t53;
    double t179 = t52 * v_y;
    double t180 = t53 * v_y;
    double t181 = t52 * v_z;
    double t182 = t53 * v_z;
    double t185 = -t36;
    double t186 = -t37;
    double t187 = -t38;
    double t188 = -t39;
    double t189 = -t40;
    double t190 = -t41;
    double t191 = -t42;
    double t192 = -t43;
    double t193 = -t44;
    double t194 = t5 * t50;
    double t195 = t10 * t47;
    double t199 = rc_y * t56;
    double t200 = rc_z * t56;
    double t201 = t56 * v_y;
    double t202 = t56 * v_z;
    double t203 = t10 * t50;
    double t205 = -t50;
    double t211 = t183 * v_y;
    double t212 = t184 * v_y;
    double t213 = t6 * t48 * v_z;
    double t214 = t183 * v_z;
    double t215 = t6 * t49 * v_z;
    double t216 = t184 * v_z;
    double t217 = t12 * t56;
    double t225 = t11 * t48 * v_y;
    double t226 = t11 * t49 * v_y;
    double t228 = a_dot_dot * t79;
    double t230 = b_dot_dot * t86;
    double t233 = a_dot_dot * t100;
    double t234 = a_dot_dot * t101;
    double t237 = b_dot_dot * t108;
    double t238 = b_dot_dot * t109;
    double t241 = a_dot_dot * t143;
    double t243 = b_dot_dot * t147;
    double t245 = t12 * t51;
    double t254 = t168 * v_y;
    double t255 = t169 * v_y;
    double t256 = t168 * v_z;
    double t257 = t169 * v_z;
    double t258 = t12 * t52;
    double t259 = t12 * t53;
    double t270 = Ixx_a * t9 * t51;
    double t271 = Ixx_b * t9 * t51;
    double t272 = Ixx_s * t9 * t51;
    double t273 = Ixy_a * t9 * t51;
    double t274 = Ixy_b * t9 * t51;
    double t275 = Ixy_s * t9 * t51;
    double t276 = t27 * t51;
    double t277 = t28 * t51;
    double t278 = t29 * t51;
    double t279 = Ixx_a * t9 * t52;
    double t280 = Ixx_b * t9 * t52;
    double t281 = Ixx_s * t9 * t52;
    double t282 = Ixy_a * t9 * t52;
    double t283 = Ixy_a * t9 * t53;
    double t284 = Ixy_b * t9 * t52;
    double t285 = Ixy_b * t9 * t53;
    double t286 = Ixy_s * t9 * t52;
    double t287 = Ixy_s * t9 * t53;
    double t288 = t27 * t52;
    double t289 = t28 * t52;
    double t290 = t29 * t52;
    double t291 = Iyy_a * t9 * t53;
    double t292 = Iyy_b * t9 * t53;
    double t293 = Iyy_s * t9 * t53;
    double t294 = t30 * t53;
    double t295 = t31 * t53;
    double t296 = t32 * t53;
    double t297 = psiVar_dot + t142;
    double t298 = Ixy_a * t9 * t56;
    double t299 = Ixy_b * t9 * t56;
    double t300 = Ixy_s * t9 * t56;
    double t301 = Iyy_a * t9 * t56;
    double t302 = Iyy_b * t9 * t56;
    double t303 = Iyy_s * t9 * t56;
    double t304 = t30 * t56;
    double t305 = t31 * t56;
    double t306 = t32 * t56;
    double t307 = psiVar_dot + t163;
    double t309 = t49 * t51;
    double t311 = -t101;
    double t312 = -t103;
    double t313 = -t105;
    double t314 = -t107;
    double t315 = -t109;
    double t316 = -t111;
    double t333 = t48 * t53;
    double t334 = t49 * t52;
    double t351 = -t168;
    double t352 = t48 * t56;
    double t356 = -t177;
    double t359 = -t184;
    double t361 = -t198;
    double t370 = t16 + t67;
    double t371 = t19 + t68;
    double t372 = t21 + t70;
    double t379 = t45 + t46;
    double t386 = -t232;
    double t421 = t13 + t66;
    double t422 = t14 + t69;
    double t423 = t15 + t71;
    double t439 = t48 + t165;
    double t442 = t49 + t178;
    double t545 = t47 + t197 + t204;
    double t550 = t27 + t100 + t143;
    double t551 = t28 + t102 + t144;
    double t552 = t29 + t104 + t145;
    double t553 = t30 + t106 + t146;
    double t554 = t31 + t108 + t147;
    double t555 = t32 + t110 + t148;
    double t206 = t10 * t54;
    double t207 = t5 * t59;
    double t208 = t10 * t55;
    double t209 = rc_z * t194;
    double t210 = rc_z * t195;
    double t218 = t199 * v_y;
    double t219 = t200 * v_y;
    double t220 = rc_x * t202;
    double t221 = t199 * v_z;
    double t222 = t200 * v_z;
    double t223 = t10 * t58;
    double t224 = rc_z * t203;
    double t229 = t6 * t64;
    double t231 = t6 * t65;
    double t235 = a_dot_dot * t124;
    double t236 = t11 * t64;
    double t239 = b_dot_dot * t132;
    double t240 = t11 * t65;
    double t242 = a_dot_dot * t152;
    double t244 = b_dot_dot * t156;
    double t246 = t161 * v_y;
    double t247 = rc_y * t166;
    double t248 = t162 * v_y;
    double t249 = t161 * v_z;
    double t250 = t162 * v_z;
    double t251 = t5 * t54;
    double t252 = t5 * t55;
    double t253 = rc_z * t170;
    double t260 = t173 * v_y;
    double t261 = rc_x * t180;
    double t262 = t174 * v_y;
    double t263 = t175 * v_y;
    double t264 = t176 * v_y;
    double t265 = t173 * v_z;
    double t266 = rc_y * t181;
    double t267 = t174 * v_z;
    double t268 = t175 * v_z;
    double t269 = t176 * v_z;
    double t308 = t9 * t167;
    double t310 = t12 * t162;
    double t317 = -t113;
    double t318 = -t115;
    double t319 = -t117;
    double t320 = -t124;
    double t321 = -t126;
    double t322 = -t128;
    double t323 = -t130;
    double t324 = -t132;
    double t325 = -t134;
    double t326 = -t136;
    double t327 = -t138;
    double t328 = -t140;
    double t329 = t12 * t166;
    double t330 = t9 * t179;
    double t331 = t12 * t167;
    double t332 = t9 * t182;
    double t335 = t12 * t175;
    double t336 = t12 * t176;
    double t337 = -t152;
    double t338 = -t153;
    double t339 = -t154;
    double t340 = -t155;
    double t341 = -t156;
    double t342 = -t157;
    double t343 = -t158;
    double t344 = -t159;
    double t345 = -t160;
    double t346 = t12 * t179;
    double t347 = t12 * t180;
    double t348 = t9 * t201;
    double t349 = t12 * t181;
    double t350 = t12 * t182;
    double t353 = t12 * t200;
    double t354 = t12 * t201;
    double t355 = t12 * t202;
    double t357 = -t180;
    double t358 = -t181;
    double t360 = -t194;
    double t362 = -t203;
    double t364 = -t211;
    double t365 = -t212;
    double t366 = -t216;
    double t373 = a_dot_dot * t270;
    double t374 = b_dot_dot * t274;
    double t375 = a_dot_dot * t279;
    double t376 = a_dot_dot * t283;
    double t377 = b_dot_dot * t284;
    double t378 = b_dot_dot * t292;
    double t380 = a_dot_dot * t298;
    double t381 = b_dot_dot * t302;
    double t382 = -t228;
    double t384 = -t230;
    double t391 = t49 * t166;
    double t395 = t48 * t167;
    double t397 = t49 * t167;
    double t400 = t48 * t179;
    double t403 = t49 * t179;
    double t407 = t48 * t182;
    double t409 = t49 * t182;
    double t413 = -t254;
    double t414 = -t256;
    double t415 = -t257;
    double t416 = t48 * t201;
    double t417 = t49 * t201;
    double t419 = t48 * t202;
    double t424 = -t258;
    double t425 = -t259;
    double t431 = -t333;
    double t432 = -t334;
    double t451 = T * t3 * t379;
    double t452 = t51 + t217;
    double t453 = t56 + t245;
    double t454 = t79 + t311;
    double t455 = t80 + t312;
    double t456 = t81 + t313;
    double t457 = t85 + t314;
    double t458 = t86 + t315;
    double t459 = t87 + t316;
    double t499 = t164 + t356;
    double t500 = t171 + t361;
    double t556 = t33 + t112 + t149;
    double t557 = t34 + t114 + t150;
    double t558 = t35 + t116 + t151;
    double t590 = t172 + t196 + t205;
    double t595 = t76 + t107 + t185;
    double t596 = t77 + t109 + t186;
    double t597 = t78 + t111 + t187;
    double t598 = t79 + t118 + t188;
    double t599 = t80 + t119 + t189;
    double t600 = t81 + t120 + t190;
    double t601 = t82 + t121 + t191;
    double t602 = t83 + t122 + t192;
    double t603 = t84 + t123 + t193;
    double t635 = t5 * t9 * t550;
    double t636 = t5 * t9 * t551;
    double t637 = t5 * t9 * t552;
    double t641 = t9 * t10 * t553;
    double t642 = t9 * t10 * t554;
    double t643 = t9 * t10 * t555;
    double t748 = t61 + t62 + t63 + t74 + t75;
    double t363 = -t209;
    double t367 = -t221;
    double t368 = -t223;
    double t369 = -t224;
    double t387 = rc_y * t308;
    double t388 = t9 * t250;
    double t389 = -t242;
    double t390 = -t244;
    double t392 = rc_y * t330;
    double t393 = t12 * t248;
    double t394 = t9 * t263;
    double t396 = rc_x * t332;
    double t398 = t12 * t250;
    double t399 = t9 * t269;
    double t402 = rc_x * t348;
    double t404 = t12 * t263;
    double t405 = t12 * t264;
    double t406 = t9 * t219;
    double t410 = t12 * t268;
    double t411 = t12 * t269;
    double t412 = -t252;
    double t418 = t12 * t219;
    double t420 = t12 * t222;
    double t426 = -t261;
    double t427 = -t264;
    double t428 = -t265;
    double t429 = -t266;
    double t430 = -t268;
    double t433 = -t335;
    double t434 = -t336;
    double t435 = -t347;
    double t436 = -t349;
    double t437 = -t373;
    double t438 = -t374;
    double t440 = -t376;
    double t441 = -t378;
    double t445 = -t397;
    double t446 = t48 * t357;
    double t447 = -t403;
    double t449 = t49 * t358;
    double t460 = t88 + t317;
    double t461 = t89 + t318;
    double t462 = t90 + t319;

    double t463 = Ixx_a * t452;
    double t464 = Ixx_a * t453;
    double t465 = Ixx_b * t452;
    double t466 = Ixx_b * t453;
    double t467 = Ixx_s * t452;
    double t468 = Ixx_s * t453;
    double t469 = Ixy_a * t452;
    double t470 = Ixy_a * t453;
    double t471 = Ixy_b * t452;
    double t472 = Ixy_b * t453;
    double t473 = Ixy_s * t452;
    double t474 = Ixy_s * t453;
    double t475 = Ixz_a * t452;
    double t476 = Ixz_a * t453;
    double t477 = Ixz_b * t452;
    double t478 = Ixz_b * t453;
    double t479 = Ixz_s * t452;
    double t480 = Ixz_s * t453;
    double t481 = Iyy_a * t452;
    double t482 = Iyy_a * t453;
    double t483 = Iyy_b * t452;
    double t484 = Iyy_b * t453;
    double t485 = Iyy_s * t452;
    double t486 = Iyy_s * t453;
    double t487 = Iyz_a * t452;
    double t488 = Iyz_a * t453;
    double t489 = Iyz_b * t452;
    double t490 = Iyz_b * t453;
    double t491 = Iyz_s * t452;
    double t492 = Iyz_s * t453;
    double t493 = a_dot * t453;
    double t494 = b_dot * t452;
    double t495 = t452 * v_y;
    double t496 = t453 * v_y;
    double t497 = t452 * v_z;
    double t498 = t453 * v_z;
    double t501 = t52 + t425;
    double t502 = t53 + t424;

    double t604 = t12 * t556;
    double t605 = t12 * t557;
    double t606 = t12 * t558;
    double t607 = t5 * t9 * t454;
    double t608 = t5 * t9 * t455;
    double t609 = t5 * t9 * t456;
    double t613 = t9 * t10 * t457;
    double t614 = t9 * t10 * t458;
    double t615 = t9 * t10 * t459;

    double t624 = fabs(t590);         // fabs() is for floating-point absolute value
    double t625 = copysign(1.0, t590);
    double t628 = t590 * t590;

    double t638 = t6 * t9 * t556;
    double t639 = t6 * t9 * t557;
    double t640 = t6 * t9 * t558;
    double t644 = t9 * t11 * t556;
    double t645 = t9 * t11 * t557;
    double t646 = t9 * t11 * t558;
    double t647 = t9 * t601;
    double t648 = t9 * t602;
    double t649 = t9 * t603;
    double t650 = t12 * t601;
    double t651 = t12 * t602;
    double t652 = t12 * t603;
    double t654 = 2.0 * t12 * t590;

    double t655 = t200 + t310 + t351;
    double t658 = t162 + t353 + t359;

    double t660 = t5 * t9 * t595;
    double t661 = t5 * t9 * t596;
    double t662 = t5 * t9 * t597;
    double t663 = t5 * t9 * t598;
    double t664 = t5 * t9 * t599;
    double t665 = t5 * t9 * t600;

    double t669 = t5 * t12 * t595;
    double t670 = t9 * t10 * t595;
    double t671 = t5 * t12 * t596;
    double t672 = t9 * t10 * t596;
    double t673 = t5 * t12 * t597;
    double t674 = t9 * t10 * t597;
    double t675 = t9 * t10 * t598;
    double t676 = t9 * t10 * t599;
    double t677 = t9 * t10 * t600;

    double t684 = t10 * t12 * t598;
    double t685 = t10 * t12 * t599;
    double t686 = t10 * t12 * t600;

    double t696 = t270 + t283 + t320;
    double t697 = t271 + t285 + t321;
    double t698 = t272 + t287 + t322;
    double t699 = t273 + t291 + t323;
    double t700 = t274 + t292 + t324;
    double t701 = t275 + t293 + t325;
    double t702 = t276 + t294 + t326;
    double t703 = t277 + t295 + t327;
    double t704 = t278 + t296 + t328;

    double t705 = 2.0 * t6 * t9 * t590;
    double t706 = 2.0 * t9 * t11 * t590;
    double t707 = t279 + t298 + t337;
    double t708 = t280 + t299 + t338;
    double t709 = t281 + t300 + t339;
    double t710 = t282 + t301 + t340;
    double t711 = t284 + t302 + t341;
    double t712 = t286 + t303 + t342;
    double t713 = t288 + t304 + t343;
    double t714 = t289 + t305 + t344;
    double t715 = t290 + t306 + t345;
    double t716 = t308 + t330 + t360;
    double t717 = t332 + t348 + t362;
    double t718 = t9 * t51 * t595;
    double t719 = t9 * t51 * t596;
    double t720 = t9 * t51 * t597;
    double t721 = t9 * t52 * t595;
    double t722 = t9 * t52 * t596;
    double t723 = t9 * t52 * t597;
    double t724 = t9 * t53 * t598;
    double t725 = t9 * t53 * t599;
    double t726 = t9 * t53 * t600;
    double t727 = t9 * t56 * t598;
    double t728 = t9 * t56 * t599;
    double t729 = t9 * t56 * t600;
    double t749 = t453 * t454;
    double t750 = t453 * t455;
    double t751 = t453 * t456;
    double t752 = t452 * t457;
    double t753 = t452 * t458;
    double t754 = t452 * t459;
    double t779 = 1.0 / t748;
    double t780 = t161 + t174 + t352 + t432;
    double t781 = t173 + t199 + t309 + t431;
    double t784 = t453 * t550;
    double t785 = t453 * t551;
    double t786 = t453 * t552;
    double t787 = t452 * t553;
    double t788 = t452 * t554;
    double t789 = t452 * t555;
    double t811 = t452 * t595;
    double t812 = t453 * t595;
    double t813 = t452 * t596;
    double t814 = t453 * t596;
    double t815 = t452 * t597;
    double t816 = t453 * t597;
    double t817 = t452 * t598;
    double t818 = t453 * t598;
    double t819 = t452 * t599;
    double t820 = t453 * t599;
    double t821 = t452 * t600;
    double t822 = t453 * t600;
    double t870 = t500 * t590 * 2.0;
    double t883 = t170 + t202 + t331 + t346 + t357;
    double t884 = t166 + t195 + t350 + t354 + t358;
    double t909 = t545 * t590 * 2.0;
    double t443 = -t387;
    double t444 = -t392;
    double t448 = -t405;
    double t450 = -t410;
    double t503 = a_dot_dot * t463;
    double t504 = a_dot_dot * t464;
    double t505 = a_dot_dot * t469;
    double t506 = a_dot_dot * t470;
    double t507 = b_dot_dot * t471;
    double t508 = b_dot_dot * t472;
    double t509 = b_dot_dot * t483;
    double t510 = b_dot_dot * t484;
    double t511 = phiVar_dot + t493;
    double t512 = t494 + thetaVar_dot;
    double t513 = Ixx_a * t501;
    double t514 = Ixx_a * t502;
    double t515 = Ixx_b * t501;
    double t516 = Ixx_b * t502;
    double t517 = Ixx_s * t501;
    double t518 = Ixx_s * t502;
    double t519 = Ixy_a * t501;
    double t520 = Ixy_a * t502;
    double t521 = Ixy_b * t501;
    double t522 = Ixy_b * t502;
    double t523 = Ixy_s * t501;
    double t524 = Ixy_s * t502;
    double t525 = Ixz_a * t501;
    double t526 = Ixz_a * t502;
    double t527 = Ixz_b * t501;
    double t528 = Ixz_b * t502;
    double t529 = Ixz_s * t501;
    double t530 = Ixz_s * t502;
    double t531 = Iyy_a * t501;
    double t532 = Iyy_a * t502;
    double t533 = Iyy_b * t501;
    double t534 = Iyy_b * t502;
    double t535 = Iyy_s * t501;
    double t536 = Iyy_s * t502;
    double t537 = Iyz_a * t501;
    double t538 = Iyz_a * t502;
    double t539 = Iyz_b * t501;
    double t540 = Iyz_b * t502;
    double t541 = Iyz_s * t501;
    double t542 = Iyz_s * t502;
    double t543 = a_dot * t502;
    double t544 = b_dot * t501;
    double t546 = t501 * v_y;
    double t547 = t502 * v_y;
    double t548 = t501 * v_z;
    double t549 = t502 * v_z;
    double t559 = t12 * t460;
    double t560 = t12 * t461;
    double t561 = t12 * t462;
    double t610 = t6 * t9 * t460;
    double t611 = t6 * t9 * t461;
    double t612 = t6 * t9 * t462;
    double t616 = t9 * t11 * t460;
    double t617 = t9 * t11 * t461;
    double t618 = t9 * t11 * t462;
    double t629 = -t604;
    double t630 = -t605;
    double t631 = -t606;
    double t653 = pow(t624, 2);
    double t656 = t169 + t175 + t434;
    double t657 = t176 + t183 + t433;
    double t659 = -t654;
    double t666 = t6 * t647;
    double t667 = t6 * t648;
    double t668 = t6 * t649;
    double t678 = t6 * t650;
    double t679 = t11 * t647;
    double t680 = t6 * t651;
    double t681 = t11 * t648;
    double t682 = t6 * t652;
    double t683 = t11 * t649;
    double t687 = t11 * t650;
    double t689 = t11 * t651;
    double t691 = t11 * t652;
    double t693 = -t650;
    double t694 = -t651;
    double t695 = -t652;
    double t730 = -t670;
    double t731 = -t672;
    double t732 = -t674;
    double t736 = t12 * t702;
    double t737 = t12 * t703;
    double t738 = t12 * t704;
    double t739 = -t718;
    double t740 = -t719;
    double t741 = -t720;
    double t742 = -t724;
    double t743 = -t725;
    double t744 = -t726;
    double t745 = t12 * t713;
    double t746 = t12 * t714;
    double t747 = t12 * t715;
    double t755 = t5 * t9 * t696;
    double t756 = t5 * t9 * t697;
    double t757 = t5 * t9 * t698;
    double t758 = t6 * t9 * t702;
    double t759 = t6 * t9 * t703;
    double t760 = t6 * t9 * t704;
    double t761 = t9 * t10 * t699;
    double t762 = t9 * t10 * t700;
    double t763 = t9 * t10 * t701;
    double t764 = t9 * t11 * t702;
    double t765 = t9 * t11 * t703;
    double t766 = t9 * t11 * t704;
    double t767 = t5 * t9 * t707;
    double t768 = t5 * t9 * t708;
    double t769 = t5 * t9 * t709;
    double t770 = t6 * t9 * t713;
    double t771 = t6 * t9 * t714;
    double t772 = t6 * t9 * t715;
    double t773 = t9 * t10 * t710;
    double t774 = t9 * t10 * t711;
    double t775 = t9 * t10 * t712;
    double t776 = t9 * t11 * t713;
    double t777 = t9 * t11 * t714;
    double t778 = t9 * t11 * t715;
    double t782 = t167 + t179 + t355 + t435;
    double t783 = t182 + t201 + t329 + t436;
    double t790 = t454 * t502;
    double t791 = t455 * t502;
    double t792 = t456 * t502;
    double t793 = t457 * t501;
    double t794 = -t752;
    double t795 = t458 * t501;
    double t796 = -t753;
    double t797 = t459 * t501;
    double t798 = -t754;
    double t823 = t502 * t550;
    double t824 = t502 * t551;
    double t825 = t502 * t552;
    double t826 = t501 * t553;
    double t828 = t501 * t554;
    double t830 = t501 * t555;
    double t855 = t501 * t595;
    double t856 = t502 * t595;
    double t857 = t501 * t596;
    double t858 = t502 * t596;
    double t859 = t501 * t597;
    double t860 = t502 * t597;
    double t861 = t501 * t598;
    double t862 = t502 * t598;
    double t863 = t501 * t599;
    double t864 = t502 * t599;
    double t865 = t501 * t600;
    double t866 = t502 * t600;
    double t916 = t453 * t707;
    double t917 = t453 * t708;
    double t918 = t453 * t709;
    double t919 = t452 * t710;
    double t920 = t452 * t711;
    double t921 = t452 * t712;
    double t922 = pow(t883, 2);
    double t923 = pow(t884, 2);
    double t924 = -t909;
    double t925 = t453 * t696;
    double t926 = t453 * t697;
    double t927 = t453 * t698;
    double t928 = t452 * t699;
    double t929 = t452 * t700;
    double t930 = t452 * t701;
    double t946 = t502 * t707;
    double t947 = t502 * t708;
    double t948 = t502 * t709;
    double t949 = t501 * t710;
    double t950 = t501 * t711;
    double t951 = t501 * t712;
    double t963 = t502 * t696;
    double t964 = t502 * t697;
    double t965 = t502 * t698;
    double t966 = t501 * t699;
    double t967 = t501 * t700;
    double t968 = t501 * t701;
    double t969 = t222 + t253 + t398 + t404 + t427;
    double t970 = t210 + t248 + t411 + t418 + t430;
    double t975 = t9 * t10 * t884 * 2.0;
    double t982 = t5 * t9 * t883 * 2.0;
    double t1003 = t12 * t624 * t625 * 2.0;
    double t1059 = t6 * t9 * t624 * t625 * 2.0;
    double t1063 = t9 * t11 * t624 * t625 * 2.0;
    double t1073 = t54 + t213 + t225 + t363 + t388 + t394;
    double t1074 = t55 + t215 + t226 + t369 + t399 + t406;
    double t1158 = t453 * t883 * 2.0;
    double t1159 = t452 * t884 * 2.0;
    double t1160 = t502 * t883 * 2.0;
    double t1161 = t501 * t884 * 2.0;
    double t1188 = t500 * t624 * t625 * 2.0;
    double t1223 = t545 * t624 * t625 * 2.0;
    double t1231 = t717 * t884 * 2.0;
    double t1232 = t716 * t883 * 2.0;
    double t1235 = t218 + t249 + t260 + t267 + t391 + t419 + t446 + t449;
    double t1254 = t208 + t220 + t247 + t251 + t395 + t400 + t409 + t417 + t426 + t429;
    double t1255 = t206 + t246 + t262 + t367 + t407 + t412 + t416 + t428 + t445 + t447;
    double t562 = a_dot_dot * t513;
    double t563 = a_dot_dot * t514;
    double t564 = -t504;
    double t565 = a_dot_dot * t519;
    double t566 = a_dot_dot * t520;
    double t567 = b_dot_dot * t521;
    double t568 = b_dot_dot * t522;
    double t569 = -t508;
    double t570 = b_dot_dot * t533;
    double t571 = b_dot_dot * t534;
    double t572 = -t514;
    double t573 = -t516;
    double t574 = -t518;
    double t575 = -t519;
    double t576 = -t520;
    double t577 = -t521;
    double t578 = -t522;
    double t579 = -t523;
    double t580 = -t524;
    double t581 = -t526;
    double t582 = -t528;
    double t583 = -t530;
    double t584 = -t531;
    double t585 = -t533;
    double t586 = -t535;
    double t587 = -t537;
    double t588 = -t539;
    double t589 = -t541;
    double t591 = -t543;
    double t592 = -t544;
    double t593 = -t547;
    double t594 = -t548;
    double t619 = -t559;
    double t620 = -t560;
    double t621 = -t561;
    double t632 = -t616;
    double t633 = -t617;
    double t634 = -t618;
    double t799 = -t755;
    double t800 = -t756;
    double t801 = -t757;
    double t802 = -t761;
    double t803 = -t762;
    double t804 = -t763;
    double t805 = -t767;
    double t806 = -t768;
    double t807 = -t769;
    double t808 = -t773;
    double t809 = -t774;
    double t810 = -t775;
    double t832 = t463 + t520;
    double t833 = t470 + t513;
    double t834 = t465 + t522;
    double t835 = t472 + t515;
    double t836 = t467 + t524;
    double t837 = t474 + t517;
    double t838 = t469 + t532;
    double t839 = t482 + t519;
    double t840 = t471 + t534;
    double t841 = t484 + t521;
    double t842 = t473 + t536;
    double t843 = t486 + t523;
    double t844 = t475 + t538;
    double t845 = t488 + t525;
    double t846 = t477 + t540;
    double t847 = t490 + t527;
    double t848 = t479 + t542;
    double t849 = t492 + t529;
    double t850 = -t793;
    double t851 = -t795;
    double t852 = -t797;
    double t853 = t496 + t549;
    double t854 = t497 + t546;
    double t867 = -t826;
    double t868 = -t828;
    double t869 = -t830;
    double t877 = -t856;
    double t878 = -t858;
    double t879 = -t860;
    double t880 = -t861;
    double t881 = -t863;
    double t882 = -t865;
    double t976 = -t946;
    double t977 = -t947;
    double t978 = -t948;
    double t979 = -t949;
    double t980 = -t950;
    double t981 = -t951;
    double t983 = -t963;
    double t984 = -t964;
    double t985 = -t965;
    double t986 = -t966;
    double t987 = -t967;
    double t988 = -t968;
    double t1004 = -t1003;
    double t1150 = t214 + t219 + t269 + t393 + t413 + t450;
    double t1151 = t250 + t255 + t263 + t366 + t420 + t448;
    double t1162 = -t1160;
    double t1175 = -t1161;
    double t1216 = t207 + t368 + t396 + t402 + t443 + t444;
    double t1230 = -t1223;
    double t1233 = t58 + t364 + t414 + t969;
    double t1234 = t59 + t365 + t415 + t970;
    double t1236 = t783 * t883 * 2.0;
    double t1237 = t782 * t884 * 2.0;
    double t1239 = t660 + t675 + t693;
    double t1240 = t661 + t676 + t694;
    double t1241 = t662 + t677 + t695;
    double t1281 = t628 + t922 + t923;
    double t1306 = t659 + t975 + t982;
    double t1351 = t629 + t635 + t641 + t647 + t669 + t684;
    double t1352 = t630 + t636 + t642 + t648 + t671 + t685;
    double t1353 = t631 + t637 + t643 + t649 + t673 + t686;
    double t1460 = t924 + t1231 + t1232;
    double t1496 = thetaVar_dot * (t646 + t691 - t723 - t729 + t789 - t825);
    double t1549 = t512 * (t645 + t689 - t722 - t728 + t788 - t824);
    double t1553 = (t543 - thetaVar_dot) * (t644 + t687 - t721 - t727 + t787 - t823);
    double t622 = -t563;
    double t623 = -t568;
    double t626 = phiVar_dot + t592;
    double t871 = t12 * t844;
    double t872 = t12 * t845;
    double t873 = t12 * t846;
    double t874 = t12 * t847;
    double t875 = t12 * t848;
    double t876 = t12 * t849;
    double t885 = t5 * t9 * t832;
    double t886 = t5 * t9 * t833;
    double t887 = t5 * t9 * t834;
    double t888 = t5 * t9 * t835;
    double t889 = t5 * t9 * t836;
    double t890 = t5 * t9 * t837;
    double t891 = t6 * t9 * t844;
    double t892 = t6 * t9 * t845;
    double t893 = t6 * t9 * t846;
    double t894 = t6 * t9 * t847;
    double t895 = t6 * t9 * t848;
    double t896 = t6 * t9 * t849;
    double t897 = t9 * t10 * t838;
    double t898 = t9 * t10 * t839;
    double t899 = t9 * t10 * t840;
    double t900 = t9 * t10 * t841;
    double t901 = t9 * t10 * t842;
    double t902 = t9 * t10 * t843;
    double t903 = t9 * t11 * t844;
    double t904 = t9 * t11 * t845;
    double t905 = t9 * t11 * t846;
    double t906 = t9 * t11 * t847;
    double t907 = t9 * t11 * t848;
    double t908 = t9 * t11 * t849;
    double t937 = t125 + t469 + t572;
    double t938 = t127 + t471 + t573;
    double t939 = t129 + t473 + t574;
    double t940 = t131 + t481 + t576;
    double t941 = t133 + t483 + t578;
    double t942 = t135 + t485 + t580;
    double t943 = t137 + t487 + t581;
    double t944 = t139 + t489 + t582;
    double t945 = t141 + t491 + t583;
    double t952 = t170 + t498 + t593;
    double t953 = t195 + t495 + t594;
    double t954 = t91 + t464 + t575;
    double t955 = t92 + t466 + t577;
    double t956 = t93 + t468 + t579;
    double t957 = t94 + t470 + t584;
    double t958 = t95 + t472 + t585;
    double t959 = t96 + t474 + t586;
    double t960 = t97 + t476 + t587;
    double t961 = t98 + t478 + t588;
    double t962 = t99 + t480 + t589;
    double t1075 = t453 * t832;
    double t1076 = t453 * t833;
    double t1077 = t453 * t834;
    double t1078 = t453 * t835;
    double t1079 = t453 * t836;
    double t1080 = t453 * t837;
    double t1081 = t452 * t838;
    double t1082 = t452 * t839;
    double t1083 = t452 * t840;
    double t1084 = t452 * t841;
    double t1085 = t452 * t842;
    double t1086 = t452 * t843;
    double t1132 = t502 * t832;
    double t1133 = t502 * t833;
    double t1135 = t502 * t834;
    double t1136 = t502 * t835;
    double t1138 = t502 * t836;
    double t1139 = t502 * t837;
    double t1141 = t501 * t838;
    double t1142 = t501 * t839;
    double t1144 = t501 * t840;
    double t1145 = t501 * t841;
    double t1147 = t501 * t842;
    double t1148 = t501 * t843;
    double t1238 = -t1237;
    double t1242 = phiVar_dot * t1241;
    double t1243 = psiVar_dot * t1241;
    double t1244 = t1241 * thetaVar_dot;
    double t1247 = b_dot * t5 * t9 * t1240;
    double t1248 = a_dot * t5 * t12 * t1239;
    double t1249 = a_dot * t9 * t10 * t1239;
    double t1250 = b_dot * t10 * t12 * t1240;
    double t1252 = t297 * t1239;
    double t1253 = t307 * t1240;
    double t1258 = t511 * t1239;
    double t1259 = t512 * t1240;
    double t1262 = t666 + t812 + t880;
    double t1263 = t667 + t814 + t881;
    double t1264 = t668 + t816 + t882;
    double t1265 = t679 + t817 + t877;
    double t1266 = t681 + t819 + t878;
    double t1267 = t683 + t821 + t879;
    double t1271 = t1239 * (t543 - thetaVar_dot);
    double t1290 = sqrt(t1281);
    double t1301 = t607 + t613 + t619 + t663 + t730;
    double t1302 = t608 + t614 + t620 + t664 + t731;
    double t1303 = t609 + t615 + t621 + t665 + t732;
    double t1345 = t705 + t1158 + t1175;
    double t1346 = t706 + t1159 + t1162;
    double t1354 = psiVar_dot * t1353;
    double t1384 = t297 * t1351;
    double t1385 = t307 * t1352;
    double t1403 = t610 + t749 + t818 + t850 + t855;
    double t1404 = t611 + t750 + t820 + t851 + t857;
    double t1405 = t612 + t751 + t822 + t852 + t859;
    double t1407 = t632 + t790 + t794 + t811 + t862;
    double t1408 = t633 + t791 + t796 + t813 + t864;
    double t1409 = t634 + t792 + t798 + t815 + t866;
    double t1462 = t638 + t678 + t739 + t742 + t784 + t867;
    double t1463 = t639 + t680 + t740 + t743 + t785 + t868;
    double t1464 = t640 + t682 + t741 + t744 + t786 + t869;
    double t1554 = -t1553;
    double t910 = -t871;
    double t911 = -t872;
    double t912 = -t873;
    double t913 = -t874;
    double t914 = -t875;
    double t915 = -t876;
    double t931 = -t891;
    double t932 = -t893;
    double t933 = -t895;
    double t934 = -t904;
    double t935 = -t906;
    double t936 = -t908;
    double t971 = abs(t952);
    double t972 = abs(t953);
    double t973 = sign(t952);
    double t974 = sign(t953);
    double t989 = t9 * t960;
    double t990 = t9 * t961;
    double t991 = t9 * t962;
    double t992 = t9 * t943;
    double t993 = t12 * t960;
    double t994 = t9 * t944;
    double t995 = t12 * t961;
    double t996 = t9 * t945;
    double t997 = t12 * t962;
    double t999 = t12 * t943;
    double t1000 = t12 * t944;
    double t1001 = t12 * t945;

    double t1005 = t5 * t9 * t937;
    double t1006 = t5 * t12 * t954;
    double t1007 = t9 * t10 * t954;
    double t1008 = t5 * t9 * t938;
    double t1009 = t5 * t12 * t955;
    double t1010 = t9 * t10 * t955;
    double t1011 = t5 * t9 * t939;
    double t1012 = t5 * t12 * t956;
    double t1013 = t9 * t10 * t956;
    double t1014 = t5 * t9 * t940;
    double t1015 = t9 * t10 * t957;
    double t1016 = t5 * t9 * t941;
    double t1017 = t9 * t10 * t958;
    double t1018 = t5 * t9 * t942;
    double t1019 = t9 * t10 * t959;

    double t1029 = t5 * t12 * t937;
    double t1030 = t9 * t10 * t937;
    double t1031 = t5 * t12 * t938;
    double t1032 = t9 * t10 * t938;
    double t1033 = t5 * t12 * t939;
    double t1034 = t9 * t10 * t939;
    double t1035 = t9 * t10 * t940;
    double t1036 = t10 * t12 * t957;
    double t1037 = t9 * t10 * t941;
    double t1038 = t10 * t12 * t958;
    double t1039 = t9 * t10 * t942;
    double t1040 = t10 * t12 * t959;

    double t1050 = t10 * t12 * t940;
    double t1051 = t10 * t12 * t941;
    double t1052 = t10 * t12 * t942;

    double t1064 = t5 * t9 * t954;
    double t1065 = t5 * t9 * t955;
    double t1066 = t5 * t9 * t956;
    double t1067 = t5 * t9 * t957;
    double t1068 = t5 * t9 * t958;
    double t1069 = t5 * t9 * t959;

    double t1108 = t9 * t51 * t954;
    double t1109 = t9 * t51 * t955;
    double t1110 = t9 * t51 * t956;
    double t1111 = t9 * t51 * t937;
    double t1112 = t9 * t52 * t954;
    double t1113 = t9 * t51 * t938;
    double t1114 = t9 * t52 * t955;
    double t1115 = t9 * t51 * t939;
    double t1116 = t9 * t52 * t956;
    double t1117 = t9 * t53 * t957;
    double t1118 = t9 * t53 * t958;
    double t1119 = t9 * t53 * t959;
    double t1120 = t9 * t52 * t937;
    double t1121 = t9 * t52 * t938;
    double t1122 = t9 * t52 * t939;
    double t1123 = t9 * t53 * t940;
    double t1124 = t9 * t56 * t957;
    double t1125 = t9 * t53 * t941;
    double t1126 = t9 * t56 * t958;
    double t1127 = t9 * t53 * t942;
    double t1128 = t9 * t56 * t959;
    double t1129 = t9 * t56 * t940;
    double t1130 = t9 * t56 * t941;
    double t1131 = t9 * t56 * t942;

    double t1134 = -t1075;
    double t1137 = -t1077;
    double t1140 = -t1079;
    double t1143 = -t1082;
    double t1146 = -t1084;
    double t1149 = -t1086;
    double t1152 = -t1132;
    double t1153 = -t1135;
    double t1154 = -t1138;
    double t1155 = -t1142;
    double t1156 = -t1145;
    double t1157 = -t1148;

    double t1163 = t452 * t954;
    double t1164 = t453 * t954;
    double t1165 = t452 * t955;
    double t1166 = t453 * t955;
    double t1167 = t452 * t956;
    double t1168 = t453 * t956;
    double t1169 = t452 * t957;
    double t1170 = t453 * t957;
    double t1171 = t452 * t958;
    double t1172 = t453 * t958;
    double t1173 = t452 * t959;
    double t1174 = t453 * t959;
    double t1176 = t452 * t937;
    double t1177 = t453 * t937;
    double t1178 = t452 * t938;
    double t1179 = t453 * t938;
    double t1180 = t452 * t939;
    double t1181 = t453 * t939;
    double t1182 = t452 * t940;
    double t1183 = t453 * t940;
    double t1184 = t452 * t941;
    double t1185 = t453 * t941;
    double t1186 = t452 * t942;
    double t1187 = t453 * t942;
    double t1189 = t501 * t954;
    double t1190 = t502 * t954;
    double t1191 = t501 * t955;
    double t1192 = t502 * t955;
    double t1193 = t501 * t956;
    double t1194 = t502 * t956;
    double t1195 = t501 * t957;
    double t1196 = t502 * t957;
    double t1197 = t501 * t958;
    double t1198 = t502 * t958;
    double t1199 = t501 * t959;
    double t1200 = t502 * t959;
    double t1201 = t501 * t937;
    double t1202 = t502 * t937;
    double t1203 = t501 * t938;
    double t1204 = t502 * t938;
    double t1205 = t501 * t939;
    double t1206 = t502 * t939;
    double t1207 = t501 * t940;
    double t1208 = t502 * t940;
    double t1210 = t501 * t941;
    double t1211 = t502 * t941;
    double t1213 = t501 * t942;
    double t1214 = t502 * t942;
    double t1245 = -t1242;
    double t1246 = -t1244;
    double t1251 = -t1249;
    double t1260 = -t1258;
    double t1261 = -t1259;
    double t1268 = t626 * t1240;
    double t1272 = phiVar_dot * t1264;
    double t1273 = t1264 * thetaVar_dot;
    double t1274 = phiVar_dot * t1267;
    double t1275 = t1267 * thetaVar_dot;
    double t1286 = a_dot * t9 * t51 * t1262;
    double t1287 = b_dot * t9 * t53 * t1263;
    double t1288 = a_dot * t9 * t52 * t1265;
    double t1289 = b_dot * t9 * t56 * t1266;
    double t1295 = 1.0 / t1290;
    double t1296 = t494 * t1263;
    double t1297 = b_dot * t453 * t1263;
    double t1298 = a_dot * t452 * t1265;
    double t1299 = t493 * t1265;
    double t1307 = a_dot * t501 * t1262;
    double t1308 = t543 * t1262;
    double t1310 = t544 * t1266;
    double t1311 = b_dot * t502 * t1266;
    double t1312 = psiVar_dot * t1303;
    double t1313 = t511 * t1262;
    double t1314 = t512 * t1263;
    double t1315 = t511 * t1265;
    double t1316 = t512 * t1266;
    double t1324 = t626 * t1263;
    double t1325 = -t1262 * (t543 - thetaVar_dot);
    double t1326 = t626 * t1266;
    double t1327 = -t1265 * (t543 - thetaVar_dot);
    double t1329 = t1265 * (t543 - thetaVar_dot) * -2.0;
    double t1332 = t297 * t1301;
    double t1334 = t307 * t1302;
    double t1335 = (areaVar * cDrag * f * t439 * t1290) / 2.0;
    double t1336 = (areaVar * cDrag * f * t442 * t1290) / 2.0;
    double t1337 = (areaVar * cDrag * f * t499 * t1290) / 2.0;
    double t1347 = (areaVar * cDrag * f * t655 * t1290) / 2.0;
    double t1348 = (areaVar * cDrag * f * t656 * t1290) / 2.0;
    double t1349 = (areaVar * cDrag * f * t657 * t1290) / 2.0;
    double t1350 = (areaVar * cDrag * f * t658 * t1290) / 2.0;
    double t1373 = (areaVar * cDrag * f * t780 * t1290) / 2.0;
    double t1374 = (areaVar * cDrag * f * t781 * t1290) / 2.0;
    double t1406 = phiVar_dot * t1405;
    double t1410 = t1409 * thetaVar_dot;
    double t1414 = (areaVar * cDrag * f * t5 * t9 * t969 * t1290) / 2.0;
    double t1415 = (areaVar * cDrag * f * t9 * t10 * t970 * t1290) / 2.0;
    double t1416 = (areaVar * cDrag * f * t1073 * t1290) / 2.0;
    double t1417 = (areaVar * cDrag * f * t1074 * t1290) / 2.0;
    double t1433 = (areaVar * cDrag * f * t1150 * t1290) / 2.0;
    double t1434 = (areaVar * cDrag * f * t1151 * t1290) / 2.0;
    double t1454 = (areaVar * cDrag * f * t453 * t969 * t1290) / 2.0;
    double t1455 = (areaVar * cDrag * f * t452 * t970 * t1290) / 2.0;
    double t1457 = (areaVar * cDrag * f * t502 * t969 * t1290) / 2.0;
    double t1458 = (areaVar * cDrag * f * t501 * t970 * t1290) / 2.0;
    double t1459 = (areaVar * cDrag * f * t1216 * t1290) / 2.0;
    double t1476 = t511 * t1403;
    double t1486 = t512 * t1408;
    double t1487 = phiVar_dot * t1464;
    double t1497 = t626 * t1404;
    double t1508 = (areaVar * cDrag * f * t1233 * t1290) / 2.0;
    double t1509 = (areaVar * cDrag * f * t1234 * t1290) / 2.0;
    double t1510 = t1407 * (t543 - thetaVar_dot);
    double t1522 = t870 + t1236 + t1238;
    double t1533 = (areaVar * cDrag * f * t1235 * t1290) / 2.0;
    double t1547 = t511 * t1462;
    double t1552 = t626 * t1463;
    double t1559 = (areaVar * cDrag * f * t12 * t1254 * t1290) / 2.0;
    double t1568 = (areaVar * cDrag * f * t6 * t9 * t1254 * t1290) / 2.0;
    double t1569 = (areaVar * cDrag * f * t9 * t11 * t1254 * t1290) / 2.0;
    double t1575 = (areaVar * cDrag * f * t1255 * t1290) / 2.0;
    double t998 = pow(t971, 2);
    double t1002 = pow(t972, 2);
    double t1020 = t6 * t992;
    double t1021 = t6 * t993;
    double t1022 = t11 * t989;
    double t1023 = t6 * t994;
    double t1024 = t6 * t995;
    double t1025 = t11 * t990;
    double t1026 = t6 * t996;
    double t1027 = t6 * t997;
    double t1028 = t11 * t991;
    double t1041 = t6 * t999;
    double t1042 = t11 * t992;
    double t1043 = t11 * t993;
    double t1044 = t6 * t1000;
    double t1045 = t11 * t994;
    double t1046 = t11 * t995;
    double t1047 = t6 * t1001;
    double t1048 = t11 * t996;
    double t1049 = t11 * t997;
    double t1053 = t11 * t999;
    double t1054 = t11 * t1000;
    double t1055 = t11 * t1001;
    double t1056 = -t993;
    double t1057 = -t995;
    double t1058 = -t997;
    double t1060 = -t999;
    double t1061 = -t1000;
    double t1062 = -t1001;
    double t1070 = t6 * t989;
    double t1071 = t6 * t990;
    double t1072 = t6 * t991;
    double t1087 = -t1007;
    double t1088 = -t1010;
    double t1089 = -t1013;
    double t1090 = -t1014;
    double t1091 = -t1016;
    double t1092 = -t1018;
    double t1217 = -t1190;
    double t1218 = -t1192;
    double t1219 = -t1194;
    double t1220 = -t1195;
    double t1221 = -t1197;
    double t1222 = -t1199;
    double t1224 = -t1202;
    double t1225 = -t1204;
    double t1226 = -t1206;
    double t1227 = -t1207;
    double t1228 = -t1210;
    double t1229 = -t1213;
    double t1256 = t5 * t9 * t971 * t973 * 2.0;
    double t1257 = t9 * t10 * t972 * t974 * 2.0;
    double t1270 = -t1268;
    double t1276 = t1272 * 2.0;
    double t1277 = t1275 * 2.0;
    double t1278 = t453 * t971 * t973 * 2.0;
    double t1279 = t452 * t972 * t974 * 2.0;
    double t1280 = -t1274;
    double t1282 = t502 * t971 * t973 * 2.0;
    double t1283 = t501 * t972 * t974 * 2.0;
    double t1291 = -t1286;
    double t1292 = -t1287;
    double t1293 = -t1288;
    double t1294 = -t1289;
    double t1300 = t717 * t972 * t974 * 2.0;
    double t1304 = t716 * t971 * t973 * 2.0;
    double t1309 = -t1298;
    double t1317 = -t1311;
    double t1318 = t1313 * 2.0;
    double t1319 = t1316 * 2.0;
    double t1321 = -t1314;
    double t1322 = -t1315;
    double t1328 = t1324 * 2.0;
    double t1330 = t853 * t971 * t973 * 2.0;
    double t1331 = t854 * t972 * t974 * 2.0;
    double t1338 = t953 * t971 * t973 * 2.0;
    double t1339 = t952 * t972 * t974 * 2.0;
    double t1413 = -t1410;
    double t1456 = -t1454;
    double t1461 = -t1457;
    double t1498 = -t1486;
    double t1550 = t73 + t227 + t1509;
    double t1551 = t451 + t1508;
    double t1574 = -t1569;
    double t1595 = t72 + t386 + t1575;
    double t1608 = t745 + t805 + t808 + t992 + t1029 + t1050;
    double t1609 = t746 + t806 + t809 + t994 + t1031 + t1051;
    double t1610 = t747 + t807 + t810 + t996 + t1033 + t1052;
    double t1611 = t736 + t799 + t802 + t989 + t1006 + t1036;
    double t1612 = t737 + t800 + t803 + t990 + t1009 + t1038;
    double t1613 = t738 + t801 + t804 + t991 + t1012 + t1040;
    double t1628 = t892 + t1076 + t1155 + t1170 + t1189;
    double t1629 = t894 + t1078 + t1156 + t1172 + t1191;
    double t1630 = t896 + t1080 + t1157 + t1174 + t1193;
    double t1631 = t903 + t1081 + t1152 + t1176 + t1208;
    double t1632 = t905 + t1083 + t1153 + t1178 + t1211;
    double t1633 = t907 + t1085 + t1154 + t1180 + t1214;
    double t1636 = t934 + t1133 + t1143 + t1163 + t1196;
    double t1637 = t935 + t1136 + t1146 + t1165 + t1198;
    double t1638 = t936 + t1139 + t1149 + t1167 + t1200;
    double t1639 = t931 + t1134 + t1141 + t1183 + t1201;
    double t1640 = t932 + t1137 + t1144 + t1185 + t1203;
    double t1641 = t933 + t1140 + t1147 + t1187 + t1205;
    double t1684 = (areaVar * cDrag * f * t1233 * t1295 * t1306) / 4.0;
    double t1685 = (areaVar * cDrag * f * t1234 * t1295 * t1306) / 4.0;
    double t1687 = (areaVar * cDrag * f * t1234 * t1295 * t1346) / 4.0;
    double t1688 = (areaVar * cDrag * f * t1233 * t1295 * t1345) / 4.0;
    double t1689 = (areaVar * cDrag * f * t1233 * t1295 * t1346) / 4.0;
    double t1690 = (areaVar * cDrag * f * t1234 * t1295 * t1345) / 4.0;
    double t1693 = (areaVar * cDrag * f * t1255 * t1295 * t1306) / 4.0;
    double t1695 = t1252 + t1313 + t1327;
    double t1696 = t1253 + t1316 + t1324;
    double t1728 = (areaVar * cDrag * f * t1255 * t1295 * t1345) / 4.0;
    double t1729 = (areaVar * cDrag * f * t1255 * t1295 * t1346) / 4.0;
    double t1739 = (areaVar * cDrag * f * t1233 * t1295 * t1460) / 4.0;
    double t1742 = (areaVar * cDrag * f * t1234 * t1295 * t1460) / 4.0;
    double t1743 = (areaVar * cDrag * f * t1233 * t1295 * t1522) / 4.0;
    double t1744 = (areaVar * cDrag * f * t1234 * t1295 * t1522) / 4.0;
    double t1746 = (areaVar * cDrag * f * t1255 * t1295 * t1460) / 4.0;
    double t1747 = (areaVar * cDrag * f * t1255 * t1295 * t1522) / 4.0;
    double t1802 = t1354 + t1487 + t1496;
    double t1093 = -t1021;
    double t1094 = -t1024;
    double t1095 = -t1027;
    double t1096 = -t1041;
    double t1098 = -t1043;
    double t1099 = -t1044;
    double t1101 = -t1046;
    double t1102 = -t1047;
    double t1104 = -t1049;
    double t1105 = -t1053;
    double t1106 = -t1054;
    double t1107 = -t1055;
    double t1284 = -t1282;
    double t1285 = -t1283;
    double t1305 = t653 + t998 + t1002;
    double t1333 = -t1331;
    double t1342 = -t1339;
    double t1355 = t1015 + t1056 + t1064;
    double t1356 = t1017 + t1057 + t1065;
    double t1357 = t1019 + t1058 + t1066;
    double t1358 = t1005 + t1035 + t1060;
    double t1359 = t1008 + t1037 + t1061;
    double t1360 = t1011 + t1039 + t1062;
    double t1420 = t1020 + t1177 + t1227;
    double t1421 = t1023 + t1179 + t1228;
    double t1422 = t1026 + t1181 + t1229;
    double t1423 = t1042 + t1182 + t1224;
    double t1424 = t1045 + t1184 + t1225;
    double t1425 = t1048 + t1186 + t1226;
    double t1427 = t1070 + t1164 + t1220;
    double t1428 = t1071 + t1166 + t1221;
    double t1429 = t1072 + t1168 + t1222;
    double t1430 = t1022 + t1169 + t1217;
    double t1431 = t1025 + t1171 + t1218;
    double t1432 = t1028 + t1173 + t1219;
    double t1555 = t5 * t12 * t1550;
    double t1556 = t9 * t10 * t1550;
    double t1557 = t5 * t9 * t1551;
    double t1558 = t10 * t12 * t1551;
    double t1560 = t9 * t51 * t1550;
    double t1561 = t9 * t52 * t1550;
    double t1564 = t9 * t53 * t1551;
    double t1565 = t9 * t56 * t1551;
    double t1566 = t1273 + t1280;
    double t1577 = t452 * t1550;
    double t1578 = t453 * t1550;
    double t1579 = t452 * t1551;
    double t1580 = t453 * t1551;
    double t1581 = t501 * t1550;
    double t1582 = t502 * t1550;
    double t1584 = t501 * t1551;
    double t1585 = t502 * t1551;
    double t1587 = t886 + t898 + t911 + t1067 + t1087;
    double t1588 = t888 + t900 + t913 + t1068 + t1088;
    double t1589 = t890 + t902 + t915 + t1069 + t1089;
    double t1590 = t885 + t897 + t910 + t1030 + t1090;
    double t1591 = t887 + t899 + t912 + t1032 + t1091;
    double t1592 = t889 + t901 + t914 + t1034 + t1092;
    double t1600 = t9 * t1595;
    double t1602 = t6 * t12 * t1595;
    double t1604 = t11 * t12 * t1595;
    double t1607 = t1004 + t1256 + t1257;
    double t1614 = psiVar_dot * t1610;
    double t1615 = psiVar_dot * t1613;
    double t1618 = t297 * t1611;
    double t1619 = t307 * t1612;
    double t1620 = t297 * t1608;
    double t1621 = t307 * t1609;
    double t1634 = phiVar_dot * t1630;
    double t1635 = t1633 * thetaVar_dot;
    double t1642 = t1638 * thetaVar_dot;
    double t1643 = phiVar_dot * t1641;
    double t1663 = t512 * t1632;
    double t1664 = t511 * t1628;
    double t1665 = t512 * t1637;
    double t1666 = t511 * t1639;
    double t1667 = -t1631 * (t543 - thetaVar_dot);
    double t1668 = t626 * t1629;
    double t1672 = t626 * t1640;
    double t1677 = t1636 * (t543 - thetaVar_dot);
    double t1683 = t1230 + t1300 + t1304;
    double t1691 = -t1689;
    double t1692 = -t1690;
    double t1694 = -t1693;
    double t1697 = a_dot * t9 * t51 * t1695;
    double t1698 = a_dot * t9 * t52 * t1695;
    double t1699 = b_dot * t9 * t53 * t1696;
    double t1700 = b_dot * t9 * t56 * t1696;
    double t1716 = t494 * t1696;
    double t1717 = b_dot * t453 * t1696;
    double t1718 = a_dot * t452 * t1695;
    double t1719 = t493 * t1695;
    double t1720 = t544 * t1696;
    double t1721 = b_dot * t502 * t1696;
    double t1723 = a_dot * t501 * t1695;
    double t1724 = t543 * t1695;
    double t1730 = -t1728;
    double t1745 = -t1744;
    double t1748 = -t1747;
    double t1749 = t1335 + t1684;
    double t1750 = t1336 + t1685;
    double t1754 = t1312 + t1406 + t1413;
    double t1763 = t1347 + t1688;
    double t1766 = t1350 + t1687;
    double t1774 = t1296 + t1310 + t1321 + t1326;
    double t1775 = t1299 + t1308 + t1322 + t1325;
    double t1792 = t1373 + t1729;
    double t1803 = phiVar_dot * t1802;
    double t1804 = t1802 * thetaVar_dot;
    double t1805 = t1416 + t1739;
    double t1806 = t1417 + t1742;
    double t1823 = t1433 + t1743;
    double t1862 = t1459 + t1746;
    double t1885 = t1247 + t1297 + t1317 + t1334 + t1497 + t1498;
    double t1886 = t1251 + t1307 + t1309 + t1332 + t1476 + t1510;
    double t1897 = t1250 + t1292 + t1294 + t1385 + t1549 + t1552;
    double t1898 = t1248 + t1291 + t1293 + t1384 + t1547 + t1554;
    double t1320 = sqrt(t1305);
    double t1365 = psiVar_dot * t1357;
    double t1366 = t1357 * thetaVar_dot;
    double t1367 = phiVar_dot * t1360;
    double t1368 = psiVar_dot * t1360;
    double t1376 = b_dot * t5 * t9 * t1356;
    double t1377 = a_dot * t5 * t12 * t1355;
    double t1378 = a_dot * t9 * t10 * t1355;
    double t1379 = b_dot * t10 * t12 * t1356;
    double t1380 = b_dot * t5 * t9 * t1359;
    double t1381 = a_dot * t5 * t12 * t1358;
    double t1382 = a_dot * t9 * t10 * t1358;
    double t1383 = b_dot * t10 * t12 * t1359;
    double t1392 = t297 * t1355;
    double t1393 = t307 * t1356;
    double t1394 = t297 * t1358;
    double t1395 = t307 * t1359;
    double t1411 = t512 * t1356;
    double t1412 = t511 * t1358;
    double t1426 = t626 * t1359;
    double t1436 = phiVar_dot * t1429;
    double t1437 = psiVar_dot * t1429;
    double t1438 = t1429 * thetaVar_dot;
    double t1439 = psiVar_dot * t1432;
    double t1440 = t1432 * thetaVar_dot;
    double t1441 = phiVar_dot * t1422;
    double t1442 = psiVar_dot * t1422;
    double t1443 = phiVar_dot * t1425;
    double t1444 = psiVar_dot * t1425;
    double t1445 = t1425 * thetaVar_dot;
    double t1465 = a_dot * t9 * t51 * t1427;
    double t1466 = b_dot * t9 * t53 * t1428;
    double t1467 = a_dot * t9 * t52 * t1430;
    double t1468 = a_dot * t9 * t51 * t1420;
    double t1469 = b_dot * t9 * t56 * t1431;
    double t1473 = b_dot * t9 * t53 * t1421;
    double t1474 = a_dot * t9 * t52 * t1423;
    double t1475 = b_dot * t9 * t56 * t1424;
    double t1477 = t297 * t1427;
    double t1478 = t297 * t1430;
    double t1479 = t307 * t1428;
    double t1480 = t307 * t1431;
    double t1481 = t297 * t1420;
    double t1482 = t297 * t1423;
    double t1483 = t307 * t1421;
    double t1484 = t307 * t1424;
    double t1500 = t494 * t1428;
    double t1501 = b_dot * t453 * t1428;
    double t1502 = a_dot * t452 * t1430;
    double t1503 = t493 * t1430;
    double t1504 = t494 * t1421;
    double t1505 = b_dot * t453 * t1421;
    double t1506 = a_dot * t452 * t1423;
    double t1507 = t493 * t1423;
    double t1511 = a_dot * t501 * t1427;
    double t1514 = t544 * t1431;
    double t1515 = b_dot * t502 * t1431;
    double t1516 = a_dot * t501 * t1420;
    double t1517 = t543 * t1420;
    double t1521 = b_dot * t502 * t1424;
    double t1523 = t511 * t1427;
    double t1524 = t512 * t1428;
    double t1525 = t512 * t1431;
    double t1526 = t511 * t1420;
    double t1527 = t511 * t1423;
    double t1528 = t512 * t1424;
    double t1531 = t592 * t1424;
    double t1537 = t626 * t1428;
    double t1539 = -t1430 * (t543 - thetaVar_dot);
    double t1540 = t626 * t1421;
    double t1541 = t626 * t1424;
    double t1542 = -t1423 * (t543 - thetaVar_dot);
    double t1543 = t1430 * (t543 - thetaVar_dot) * -2.0;
    double t1545 = t1427 * (t543 - thetaVar_dot);
    double t1562 = -t1555;
    double t1563 = -t1556;
    double t1567 = -t1557;
    double t1570 = -t1561;
    double t1571 = phiVar_dot * t1566;
    double t1572 = t1566 * thetaVar_dot;
    double t1573 = -t1564;
    double t1583 = -t1577;
    double t1586 = -t1581;
    double t1593 = psiVar_dot * t1589;
    double t1594 = psiVar_dot * t1592;
    double t1596 = t297 * t1587;
    double t1597 = t307 * t1588;
    double t1598 = t297 * t1590;
    double t1599 = t307 * t1591;
    double t1601 = t6 * t1600;
    double t1603 = t11 * t1600;
    double t1606 = -t1604;
    double t1616 = -t1614;
    double t1617 = -t1615;
    double t1623 = -t1619;
    double t1625 = -t1621;
    double t1626 = t1063 + t1279 + t1284;
    double t1627 = t1059 + t1278 + t1285;
    double t1644 = -t1642;
    double t1645 = -t1643;
    double t1646 = t1338 + t1342;
    double t1647 = t776 + t919 + t976 + t1105 + t1120 + t1129;
    double t1648 = t777 + t920 + t977 + t1106 + t1121 + t1130;
    double t1649 = t778 + t921 + t978 + t1107 + t1122 + t1131;
    double t1650 = t758 + t925 + t986 + t1093 + t1108 + t1117;
    double t1651 = t759 + t926 + t987 + t1094 + t1109 + t1118;
    double t1652 = t760 + t927 + t988 + t1095 + t1110 + t1119;
    double t1653 = t764 + t928 + t983 + t1098 + t1112 + t1124;
    double t1654 = t765 + t929 + t984 + t1101 + t1114 + t1126;
    double t1655 = t766 + t930 + t985 + t1104 + t1116 + t1128;
    double t1656 = t770 + t916 + t979 + t1096 + t1111 + t1123;
    double t1657 = t771 + t917 + t980 + t1099 + t1113 + t1125;
    double t1658 = t772 + t918 + t981 + t1102 + t1115 + t1127;
    double t1669 = -t1665;
    double t1670 = -t1666;
    double t1686 = t1188 + t1330 + t1333;
    double t1701 = t1420 + t1430;
    double t1702 = t1421 + t1431;
    double t1703 = t1422 + t1432;
    double t1704 = -t1697;
    double t1705 = -t1698;
    double t1706 = -t1699;
    double t1707 = -t1700;
    double t1713 = phiVar_dot * (t1072 + t1168 - t1199 - t1425);
    double t1714 = thetaVar_dot * (t1072 + t1168 - t1199 - t1425);
    double t1722 = -t1716;
    double t1725 = -t1718;
    double t1726 = -t1719;
    double t1727 = -t1721;
    double t1733 = t511 * (t1070 + t1164 - t1195 - t1423);
    double t1735 = t512 * (t1071 + t1166 - t1197 - t1424);
    double t1740 = t626 * (t1071 + t1166 - t1197 - t1424);
    double t1751 = t9 * t10 * t1749;
    double t1752 = t5 * t9 * t1750;
    double t1755 = t452 * t1749;
    double t1756 = t453 * t1750;
    double t1757 = phiVar_dot * t1754;
    double t1758 = t1754 * thetaVar_dot;
    double t1759 = t501 * t1749;
    double t1760 = t502 * t1750;
    double t1761 = t1337 + t1694;
    double t1767 = t1348 + t1692;
    double t1768 = t1349 + t1691;
    double t1770 = t9 * t10 * t1763;
    double t1771 = t5 * t9 * t1766;
    double t1776 = t452 * t1763;
    double t1777 = t453 * t1766;
    double t1780 = t501 * t1763;
    double t1782 = t502 * t1766;
    double t1788 = t511 * t1775;
    double t1789 = t512 * t1774;
    double t1791 = t626 * t1774;
    double t1793 = t1374 + t1730;
    double t1794 = t1775 * (t543 - thetaVar_dot);
    double t1796 = t12 * t1792;
    double t1798 = t6 * t9 * t1792;
    double t1799 = t9 * t11 * t1792;
    double t1807 = t9 * t10 * t1805;
    double t1808 = t5 * t9 * t1806;
    double t1822 = t453 * t1806;
    double t1824 = t452 * t1805;
    double t1830 = t1434 + t1745;
    double t1836 = t502 * t1806;
    double t1837 = t501 * t1805;
    double t1840 = t9 * t10 * t1823;
    double t1856 = t452 * t1823;
    double t1858 = t501 * t1823;
    double t1863 = t12 * t1862;
    double t1864 = t6 * t9 * t1862;
    double t1865 = t9 * t11 * t1862;
    double t1867 = t1533 + t1748;
    double t1887 = t512 * t1885;
    double t1888 = t511 * t1886;
    double t1891 = t626 * t1885;
    double t1894 = -t1886 * (t543 - thetaVar_dot);
    double t1899 = t511 * t1898;
    double t1900 = t512 * t1897;
    double t1903 = -t1898 * (t543 - thetaVar_dot);
    double t1904 = t626 * t1897;
    double t1323 = 1.0 / t1320;
    double t1340 = (areaVar * cDrag * f * t6 * t9 * t12 * t1320) / 2.0;
    double t1341 = (areaVar * cDrag * f * t9 * t11 * t12 * t1320) / 2.0;
    double t1343 = (areaVar * cDrag * f * t6 * t11 * t25 * t1320) / 2.0;
    double t1361 = (areaVar * cDrag * f * t5 * t9 * t453 * t1320) / 2.0;
    double t1362 = (areaVar * cDrag * f * t9 * t10 * t452 * t1320) / 2.0;
    double t1369 = (areaVar * cDrag * f * t5 * t9 * t502 * t1320) / 2.0;
    double t1370 = (areaVar * cDrag * f * t9 * t10 * t501 * t1320) / 2.0;
    double t1371 = t1365 * 2.0;
    double t1372 = t1368 * 2.0;
    double t1387 = -t1378;
    double t1388 = -t1379;
    double t1391 = -t1383;
    double t1396 = t1392 * 2.0;
    double t1397 = t1393 * 2.0;
    double t1398 = t1394 * 2.0;
    double t1399 = t1395 * 2.0;
    double t1401 = (areaVar * cDrag * f * t452 * t501 * t1320) / 2.0;
    double t1402 = (areaVar * cDrag * f * t453 * t502 * t1320) / 2.0;
    double t1446 = t1440 * 2.0;
    double t1447 = t1441 * 2.0;
    double t1448 = -t1437;
    double t1449 = -t1438;
    double t1450 = -t1439;
    double t1452 = -t1443;
    double t1453 = -t1444;
    double t1488 = -t1477;
    double t1489 = -t1478;
    double t1490 = -t1479;
    double t1491 = -t1480;
    double t1493 = -t1482;
    double t1495 = -t1484;
    double t1513 = -t1502;
    double t1518 = -t1504;
    double t1529 = -t1515;
    double t1530 = -t1516;
    double t1532 = t1525 * 2.0;
    double t1534 = t1526 * 2.0;
    double t1535 = -t1524;
    double t1536 = -t1527;
    double t1544 = t1540 * 2.0;
    double t1546 = -t1541;
    double t1576 = -t1572;
    double t1605 = -t1601;
    double t1659 = phiVar_dot * t1652;
    double t1660 = t1649 * thetaVar_dot;
    double t1661 = t1655 * thetaVar_dot;
    double t1662 = phiVar_dot * t1658;
    double t1673 = t511 * t1656;
    double t1674 = t511 * t1650;
    double t1675 = t512 * t1648;
    double t1676 = t512 * t1654;
    double t1679 = t626 * t1657;
    double t1680 = t626 * t1651;
    double t1708 = phiVar_dot * t1703;
    double t1709 = t1703 * thetaVar_dot;
    double t1715 = -t1714;
    double t1731 = t511 * t1701;
    double t1732 = t512 * t1702;
    double t1736 = t626 * t1702;
    double t1737 = -t1701 * (t543 - thetaVar_dot);
    double t1738 = -t1735;
    double t1753 = -t1752;
    double t1762 = t12 * t1761;
    double t1764 = t6 * t9 * t1761;
    double t1765 = t9 * t11 * t1761;
    double t1772 = t5 * t9 * t1767;
    double t1773 = t9 * t10 * t1768;
    double t1778 = t453 * t1767;
    double t1779 = t452 * t1768;
    double t1781 = -t1776;
    double t1783 = -t1777;
    double t1784 = t502 * t1767;
    double t1786 = t501 * t1768;
    double t1795 = -t1791;
    double t1797 = t12 * t1793;
    double t1800 = t6 * t9 * t1793;
    double t1801 = t9 * t11 * t1793;
    double t1809 = -t1807;
    double t1810 = t1392 + t1523 + t1539;
    double t1811 = t1393 + t1525 + t1537;
    double t1812 = t1394 + t1526 + t1542;
    double t1813 = t1395 + t1528 + t1540;
    double t1841 = t5 * t9 * t1830;
    double t1857 = t453 * t1830;
    double t1859 = -t1856;
    double t1860 = t502 * t1830;
    double t1866 = -t1864;
    double t1868 = t12 * t1867;
    double t1869 = t6 * t9 * t1867;
    double t1870 = t9 * t11 * t1867;
    double t1871 = t1593 + t1634 + t1644;
    double t1872 = t1594 + t1635 + t1645;
    double t1927 = -t307 * (t1380 + t1505 - t1521 - t1599 - t1663 + t1672);
    double t1937 = -t626 * (t1380 + t1505 - t1521 - t1599 - t1663 + t1672);
    double t1363 = -t1361;
    double t1364 = -t1362;
    double t1769 = -t1765;
    double t1785 = -t1778;
    double t1787 = -t1779;
    double t1814 = a_dot * t5 * t12 * t1812;
    double t1815 = a_dot * t9 * t10 * t1812;
    double t1816 = b_dot * t5 * t9 * t1813;
    double t1817 = b_dot * t10 * t12 * t1813;
    double t1818 = a_dot * t5 * t12 * t1810;
    double t1819 = a_dot * t9 * t10 * t1810;
    double t1820 = b_dot * t5 * t9 * t1811;
    double t1821 = b_dot * t10 * t12 * t1811;
    double t1825 = a_dot * t9 * t51 * t1812;
    double t1826 = b_dot * t9 * t53 * t1813;
    double t1831 = a_dot * t9 * t52 * t1810;
    double t1832 = b_dot * t9 * t56 * t1811;
    double t1842 = t494 * t1813;
    double t1843 = b_dot * t453 * t1813;
    double t1844 = a_dot * t452 * t1810;
    double t1845 = t493 * t1810;
    double t1846 = a_dot * t501 * t1812;
    double t1847 = t543 * t1812;
    double t1852 = t544 * t1811;
    double t1853 = b_dot * t502 * t1811;
    double t1861 = -t1857;
    double t1873 = psiVar_dot * t1871;
    double t1874 = t1871 * thetaVar_dot;
    double t1875 = phiVar_dot * t1872;
    double t1876 = psiVar_dot * t1872;
    double t1878 = t1617 + t1659 + t1661;
    double t1879 = t1616 + t1660 + t1662;
    double t1889 = t1365 + t1709 + t1713;
    double t1890 = t1368 + t1708 + t1715;
    double t1901 = t1274 + t1315 + t1326 + t1450 + t1489 + t1491;
    double t1906 = t1392 + t1507 + t1517 + t1733 + t1737;
    double t1907 = t1395 + t1500 + t1514 + t1736 + t1738;
    double t1909 = t1393 + t1518 + t1531 + t1732 + t1740;
    double t1914 = t297 * (t1394 - t1503 + t1731 + t591 * t1427 + (t543 - thetaVar_dot) * (t1070 + t1164 - t1195 - t1423));
    double t1917 = (t543 - thetaVar_dot) * (t1394 - t1503 + t1731 + t591 * t1427 + (t543 - thetaVar_dot) * (t1070 + t1164 - t1195 - t1423));
    double t1920 = t1382 + t1506 + t1530 + t1598 + t1667 + t1670;
    double t1921 = t1376 + t1501 + t1529 + t1597 + t1668 + t1669;
    double t1923 = t1387 + t1511 + t1513 + t1596 + t1664 + t1677;
    double t1930 = t1751 + t1753 + t1762;
    double t1935 = t1391 + t1473 + t1475 + t1625 + t1675 + t1679;
    double t1936 = t1388 + t1466 + t1469 + t1623 + t1676 + t1680;
    double t1939 = -t297 * (t1381 - t1468 - t1474 + t1620 - t1673 + t1647 * (t543 - thetaVar_dot));
    double t1940 = -t297 * (t1377 - t1465 - t1467 + t1618 - t1674 + t1653 * (t543 - thetaVar_dot));
    double t1944 = t1756 + t1759 + t1764;
    double t1947 = t511 * (t1381 - t1468 - t1474 + t1620 - t1673 + t1647 * (t543 - thetaVar_dot));
    double t1948 = (t543 - thetaVar_dot) * (t1377 - t1465 - t1467 + t1618 - t1674 + t1653 * (t543 - thetaVar_dot));
    double t1951 = t1771 + t1773 + t1796;
    double t1952 = t1770 + t1772 + t1797;
    double t1953 = t1243 + t1252 + t1253 + t1272 + t1277 + t1313 + t1319 + t1324 + t1329 + t1453 + t1493 + t1495;
    double t1954 = t1783 + t1786 + t1798;
    double t1956 = t1243 + t1252 + t1253 + t1275 + t1276 + t1316 + t1318 + t1327 + t1328 + t1448 + t1488 + t1490;
    double t1957 = t1781 + t1784 + t1801;
    double t1959 = t1245 + t1260 + t1270 + t1371 + t1396 + t1397 + t1436 + t1440 + t1523 + t1525 + t1537 + t1539;
    double t1960 = t1246 + t1261 + t1271 + t1372 + t1398 + t1399 + t1441 + t1445 + t1526 + t1528 + t1540 + t1542;
    double t1961 = t1368 + t1394 + t1395 + t1445 + t1447 + t1449 + t1528 + t1534 + t1535 + t1542 + t1544 + t1545;
    double t1962 = t1365 + t1392 + t1393 + t1436 + t1446 + t1452 + t1523 + t1532 + t1536 + t1537 + t1543 + t1546;
    double t1827 = -t1814;
    double t1828 = -t1816;
    double t1829 = -t1817;
    double t1833 = -t1818;
    double t1834 = -t1820;
    double t1835 = -t1821;
    double t1838 = -t1825;
    double t1839 = -t1826;
    double t1848 = -t1842;
    double t1849 = -t1843;
    double t1850 = -t1844;
    double t1851 = -t1845;
    double t1854 = -t1846;
    double t1855 = -t1853;
    double t1877 = -t1873;
    double t1880 = phiVar_dot * t1879;
    double t1881 = psiVar_dot * t1879;
    double t1882 = psiVar_dot * t1878;
    double t1883 = t1878 * thetaVar_dot;
    double t1892 = phiVar_dot * t1889;
    double t1893 = psiVar_dot * t1889;
    double t1895 = psiVar_dot * t1890;
    double t1896 = t1890 * thetaVar_dot;
    double t1910 = t297 * t1906;
    double t1911 = t307 * t1907;
    double t1912 = t307 * t1909;
    double t1915 = t511 * t1906;
    double t1916 = t512 * t1907;
    double t1918 = t626 * t1909;
    double t1919 = -t1917;
    double t1924 = t297 * t1920;
    double t1925 = t307 * t1921;
    double t1926 = t297 * t1923;
    double t1931 = t511 * t1920;
    double t1932 = t512 * t1921;
    double t1938 = -t1923 * (t543 - thetaVar_dot);
    double t1941 = t307 * t1935;
    double t1942 = t307 * t1936;
    double t1943 = t512 * t1936;
    double t1946 = t1755 + t1760 + t1769;
    double t1949 = t626 * t1935;
    double t1955 = t1782 + t1787 + t1799;
    double t1958 = t1780 + t1785 + t1800;
    double t1884 = -t1880;
    double t1928 = -t1925;
    double t1929 = -t1926;
    double t1950 = -t1949;
    double t1963 = t236 + t240 + t505 + t509 + t622 + t623 + t1576 + t1579 + t1582 + t1603 + t1720 + t1726 + t1789 + t1794 + t1858 + t1861 + t1869 + t1893 + t1910 + t1912;
    double t1965 = t506 + t510 + t562 + t567 + t1456 + t1458 + t1568 + t1580 + t1586 + t1725 + t1727 + t1758 + t1815 + t1828 + t1876 + t1887 + t1894 + t1924 + t1927;
    double t1967 = t375 + t377 + t380 + t381 + t389 + t390 + t1565 + t1570 + t1606 + t1704 + t1706 + t1803 + t1824 + t1833 + t1835 + t1836 + t1865 + t1882 + t1899 + t1904 + t1940 + t1942;
    double t1968 = t235 + t239 + t437 + t438 + t440 + t441 + t1560 + t1573 + t1602 + t1705 + t1707 + t1804 + t1822 + t1827 + t1829 + t1837 + t1866 + t1881 + t1900 + t1903 + t1939 + t1941;
    double t1969 = t234 + t238 + t382 + t384 + t1414 + t1415 + t1559 + t1563 + t1567 + t1849 + t1850 + t1854 + t1855 + t1874 + t1875 + t1931 + t1932 + t1937 + t1938;
    double t1970 = t1840 + t1841 + t1847 + t1848 + t1851 + t1852 + t1868 + t1892 + t1896 + t1915 + t1916 + t1918 + t1919;
    double t1966 = t503 + t507 + t566 + t571 + t1455 + t1461 + t1574 + t1583 + t1585 + t1717 + t1723 + t1757 + t1819 + t1834 + t1877 + t1888 + t1891 + t1928 + t1929;
    double t1971 = t64 + t65 + t233 + t237 + t241 + t243 + t1558 + t1562 + t1600 + t1808 + t1809 + t1831 + t1832 + t1838 + t1839 + t1863 + t1883 + t1884 + t1943 + t1947 + t1948 + t1950;

    double mt1[] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    
        -t57 * (
            (areaVar * cDrag * f * pow(t12, 2) * t1320) / 2.0 +
            (areaVar * cDrag * f * pow(t5, 2) * t25 * t1320) / 2.0 +
            (areaVar * cDrag * f * pow(t10, 2) * t25 * t1320) / 2.0 -
            (areaVar * cDrag * f * t12 * t590 * t1323 * t1607) / 4.0 +
            (areaVar * cDrag * f * t5 * t9 * t952 * t1323 * t1607) / 4.0 +
            (areaVar * cDrag * f * t9 * t10 * t953 * t1323 * t1607) / 4.0
        ),
    
        t57 * (
            t1341 + t1364 + t1369 -
            (areaVar * cDrag * f * t452 * t953 * t1323 * t1607) / 4.0 +
            (areaVar * cDrag * f * t502 * t952 * t1323 * t1607) / 4.0 -
            (areaVar * cDrag * f * t9 * t11 * t590 * t1323 * t1607) / 4.0
        ),
    
        t57 * (
            t1340 + t1363 + t1370 -
            (areaVar * cDrag * f * t453 * t952 * t1323 * t1607) / 4.0 +
            (areaVar * cDrag * f * t501 * t953 * t1323 * t1607) / 4.0 -
            (areaVar * cDrag * f * t6 * t9 * t590 * t1323 * t1607) / 4.0
        ),
    
        0.0
    };

    double mt2[] = {
        0.0, 0.0,
    
        // Index 2
        t371 * t779 * t1944 - t372 * t779 * t1946 + t423 * t779 * t1930,
    
        // Index 3
        -t372 * t779 * t1930 + t370 * t779 * t1944 + t422 * t779 * t1946,
    
        // Index 4
        -t371 * t779 * t1930 - t370 * t779 * t1946 - t421 * t779 * t1944,
    
        0.0,
        1.0,
        0.0,
    
        // Index 8
        t57 * (
            t1341 + t1364 + t1369 +
            (areaVar * cDrag * f * t12 * t590 * t1323 * t1626) / 4.0 -
            (areaVar * cDrag * f * t5 * t9 * t952 * t1323 * t1626) / 4.0 -
            (areaVar * cDrag * f * t9 * t10 * t953 * t1323 * t1626) / 4.0
        ),
    
        // Index 9
        -t57 * (
            (areaVar * cDrag * f * pow(t452, 2) * t1320) / 2.0 +
            (areaVar * cDrag * f * pow(t502, 2) * t1320) / 2.0 +
            (areaVar * cDrag * f * pow(t11, 2) * t25 * t1320) / 2.0 +
            (areaVar * cDrag * f * t452 * t953 * t1323 * t1626) / 4.0 -
            (areaVar * cDrag * f * t502 * t952 * t1323 * t1626) / 4.0 +
            (areaVar * cDrag * f * t9 * t11 * t590 * t1323 * t1626) / 4.0
        )
    };

    double mt3[] = {
        // Index 0
        -t57 * (
            t1343 - t1401 - t1402 +
            (areaVar * cDrag * f * t453 * t952 * t1323 * t1626) / 4.0 -
            (areaVar * cDrag * f * t501 * t953 * t1323 * t1626) / 4.0 +
            (areaVar * cDrag * f * t6 * t9 * t590 * t1323 * t1626) / 4.0
        ),
    
        0.0, 0.0, 0.0,
    
        // Index 4–6
        -t371 * t779 * t1954 - t372 * t779 * t1955 - t423 * t779 * t1951,
        t372 * t779 * t1951 - t370 * t779 * t1954 + t422 * t779 * t1955,
        t371 * t779 * t1951 - t370 * t779 * t1955 + t421 * t779 * t1954,
    
        0.0, 0.0, 1.0,
    
        // Index 10
        t57 * (
            t1340 + t1363 + t1370 +
            (areaVar * cDrag * f * t12 * t590 * t1323 * t1627) / 4.0 -
            (areaVar * cDrag * f * t5 * t9 * t952 * t1323 * t1627) / 4.0 -
            (areaVar * cDrag * f * t9 * t10 * t953 * t1323 * t1627) / 4.0
        ),
    
        // Index 11
        -t57 * (
            t1343 - t1401 - t1402 +
            (areaVar * cDrag * f * t452 * t953 * t1323 * t1627) / 4.0 -
            (areaVar * cDrag * f * t502 * t952 * t1323 * t1627) / 4.0 +
            (areaVar * cDrag * f * t9 * t11 * t590 * t1323 * t1627) / 4.0
        )
    };

    double mt4[] = {
        // Index 0
        -t57 * (
            (areaVar * cDrag * f * pow(t453, 2) * t1320) / 2.0 +
            (areaVar * cDrag * f * pow(t501, 2) * t1320) / 2.0 +
            (areaVar * cDrag * f * pow(t6, 2) * t25 * t1320) / 2.0 +
            (areaVar * cDrag * f * t453 * t952 * t1323 * t1627) / 4.0 -
            (areaVar * cDrag * f * t501 * t953 * t1323 * t1627) / 4.0 +
            (areaVar * cDrag * f * t6 * t9 * t590 * t1323 * t1627) / 4.0
        ),
    
        0.0, 0.0, 0.0,
    
        // Index 4–6
        t371 * t779 * t1958 + t372 * t779 * t1957 + t423 * t779 * t1952,
        -t372 * t779 * t1952 + t370 * t779 * t1958 - t422 * t779 * t1957,
        -t371 * t779 * t1952 + t370 * t779 * t1957 - t421 * t779 * t1958,
    
        0.0, 0.0, 0.0,
    
        // Index 10
        t57 * (
            (areaVar * cDrag * f * t12 * t500 * t1320) / 2.0 -
            (areaVar * cDrag * f * t5 * t9 * t853 * t1320) / 2.0 +
            (areaVar * cDrag * f * t9 * t10 * t854 * t1320) / 2.0 +
            (areaVar * cDrag * f * t12 * t590 * t1323 * t1686) / 4.0 -
            (areaVar * cDrag * f * t5 * t9 * t952 * t1323 * t1686) / 4.0 -
            (areaVar * cDrag * f * t9 * t10 * t953 * t1323 * t1686) / 4.0
        )
    };
    
    double mt5[] = {
        -t57 * (
            T * t8 * t501
            - T * t3 * t7 * t453
            + T * t2 * t3 * t6 * t9
    
            - (areaVar * cDrag * f * t452 * t854 * t1320) / 2.0
            - (areaVar * cDrag * f * t502 * t853 * t1320) / 2.0
            + (areaVar * cDrag * f * t453 * t952 * t1320) / 2.0
            - (areaVar * cDrag * f * t501 * t953 * t1320) / 2.0
    
            + (areaVar * cDrag * f * t9 * t11 * t500 * t1320) / 2.0
            + (areaVar * cDrag * f * t6 * t9 * t590 * t1320) / 2.0
    
            + (areaVar * cDrag * f * t452 * t953 * t1323 * t1686) / 4.0
            - (areaVar * cDrag * f * t502 * t952 * t1323 * t1686) / 4.0
            + (areaVar * cDrag * f * t9 * t11 * t590 * t1323 * t1686) / 4.0
        )
    };

    double mt6[] = {
        -t57 * (
            T * t8 * t452
            - T * t3 * t7 * t502
            - T * t2 * t3 * t9 * t11
    
            + (areaVar * cDrag * f * t453 * t853 * t1320) / 2.0
            + (areaVar * cDrag * f * t501 * t854 * t1320) / 2.0
            - (areaVar * cDrag * f * t452 * t953 * t1320) / 2.0
            + (areaVar * cDrag * f * t502 * t952 * t1320) / 2.0
    
            + (areaVar * cDrag * f * t6 * t9 * t500 * t1320) / 2.0
            - (areaVar * cDrag * f * t9 * t11 * t590 * t1320) / 2.0
    
            + (areaVar * cDrag * f * t453 * t952 * t1323 * t1686) / 4.0
            - (areaVar * cDrag * f * t501 * t953 * t1323 * t1686) / 4.0
            + (areaVar * cDrag * f * t6 * t9 * t590 * t1323 * t1686) / 4.0
        ),
    
        0.0, 0.0, 0.0,
    
        // Final element
        t372 * t779 * (
            -t229 - t231 + t564 + t565 + t569 + t570
            + t1571 + t1578 + t1584 + t1605
            + t1722 + t1724 + t1788 + t1795 + t1859 + t1860 + t1870 + t1895 + t1911 + t1914
        )
        + t371 * t779 * t1963
        + t423 * t779 * t1970
    };

    double mt7[] = {
        -t422 * t779 * (
            -t229 - t231 + t564 + t565 + t569 + t570
            + t1571 + t1578 + t1584 + t1605
            + t1722 + t1724 + t1788 + t1795 + t1859 + t1860 + t1870 + t1895 + t1911 + t1914
        )
        + t370 * t779 * t1963
        - t372 * t779 * t1970,
    
        t370 * t779 * (
            -t229 - t231 + t564 + t565 + t569 + t570
            + t1571 + t1578 + t1584 + t1605
            + t1722 + t1724 + t1788 + t1795 + t1859 + t1860 + t1870 + t1895 + t1911 + t1914
        )
        - t371 * t779 * t1970
        - t421 * t779 * t1963,
    
        0.0, 0.0, 0.0
    };

    double mt8[] = {
        -t57 * (
            -T * t2 * t3 * t9
            + T * t8 * t10 * t12
            + T * t3 * t5 * t7 * t12
    
            + (areaVar * cDrag * f * t12 * t545 * t1320) / 2.0
            - (areaVar * cDrag * f * t9 * t590 * t1320) / 2.0
            + (areaVar * cDrag * f * t5 * t9 * t716 * t1320) / 2.0
            + (areaVar * cDrag * f * t9 * t10 * t717 * t1320) / 2.0
    
            - (areaVar * cDrag * f * t5 * t12 * t952 * t1320) / 2.0
            - (areaVar * cDrag * f * t10 * t12 * t953 * t1320) / 2.0
    
            - (areaVar * cDrag * f * t12 * t590 * t1323 * t1683) / 4.0
            + (areaVar * cDrag * f * t5 * t9 * t952 * t1323 * t1683) / 4.0
            + (areaVar * cDrag * f * t9 * t10 * t953 * t1323 * t1683) / 4.0
        )
    };

    double mt9[] = {
        t57 * (
            T * t8 * t9 * t56
            + T * t2 * t3 * t11 * t12
            + T * t3 * t7 * t9 * t52
    
            - (areaVar * cDrag * f * t452 * t717 * t1320) / 2.0
            + (areaVar * cDrag * f * t502 * t716 * t1320) / 2.0
            + (areaVar * cDrag * f * t9 * t11 * t545 * t1320) / 2.0
            + (areaVar * cDrag * f * t11 * t12 * t590 * t1320) / 2.0
    
            - (areaVar * cDrag * f * t9 * t52 * t952 * t1320) / 2.0
            - (areaVar * cDrag * f * t9 * t56 * t953 * t1320) / 2.0
    
            - (areaVar * cDrag * f * t452 * t953 * t1323 * t1683) / 4.0
            + (areaVar * cDrag * f * t502 * t952 * t1323 * t1683) / 4.0
            - (areaVar * cDrag * f * t9 * t11 * t590 * t1323 * t1683) / 4.0
        )
    };
    

    double mt10[] = {
        t57 * (
            T * t8 * t9 * t53
            + T * t2 * t3 * t6 * t12
            + T * t3 * t7 * t9 * t51
    
            - (areaVar * cDrag * f * t453 * t716 * t1320) / 2.0
            + (areaVar * cDrag * f * t501 * t717 * t1320) / 2.0
            + (areaVar * cDrag * f * t6 * t9 * t545 * t1320) / 2.0
            + (areaVar * cDrag * f * t6 * t12 * t590 * t1320) / 2.0
    
            - (areaVar * cDrag * f * t9 * t51 * t952 * t1320) / 2.0
            - (areaVar * cDrag * f * t9 * t53 * t953 * t1320) / 2.0
    
            - (areaVar * cDrag * f * t453 * t952 * t1323 * t1683) / 4.0
            + (areaVar * cDrag * f * t501 * t953 * t1323 * t1683) / 4.0
            - (areaVar * cDrag * f * t6 * t9 * t590 * t1323 * t1683) / 4.0
        ),
    
        0.0, 0.0, 0.0,
    
        t371 * t779 * t1968
        - t372 * t779 * t1967
        - t423 * t779 * t1971,
    
        t370 * t779 * t1968
        + t372 * t779 * t1971
        + t422 * t779 * t1967,
    
        -t370 * t779 * t1967
        + t371 * t779 * t1971
        - t421 * t779 * t1968,
    
        0.0, 0.0, 0.0
    };
    
    double mt11[] = {
        t57 * (
            T * t5 * t8 * t9
            - T * t3 * t7 * t9 * t10
            - (areaVar * cDrag * f * t12 * t590 * t1323 * t1646) / 4.0
            + (areaVar * cDrag * f * t5 * t9 * t952 * t1323 * t1646) / 4.0
            + (areaVar * cDrag * f * t9 * t10 * t953 * t1323 * t1646) / 4.0
        ),
    
        -t57 * (
            T * t8 * t502
            + T * t3 * t7 * t452
            - (areaVar * cDrag * f * t452 * t953 * t1323 * t1646) / 4.0
            + (areaVar * cDrag * f * t502 * t952 * t1323 * t1646) / 4.0
            - (areaVar * cDrag * f * t9 * t11 * t590 * t1323 * t1646) / 4.0
        ),
    
        t57 * (
            T * t8 * t453
            + T * t3 * t7 * t501
            + (areaVar * cDrag * f * t453 * t952 * t1323 * t1646) / 4.0
            - (areaVar * cDrag * f * t501 * t953 * t1323 * t1646) / 4.0
            + (areaVar * cDrag * f * t6 * t9 * t590 * t1323 * t1646) / 4.0
        ),
    
        0.0, 0.0, 0.0,
    
        -t371 * t779 * t1965
        + t372 * t779 * t1966
        - t423 * t779 * t1969,
    
        -t370 * t779 * t1965
        + t372 * t779 * t1969
        - t422 * t779 * t1966,
    
        t370 * t779 * t1966
        + t371 * t779 * t1969
        + t421 * t779 * t1965,
    
        0.0
    };

    double mt12[] = {
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
    
        t423 * t779 * (-t1366 + t1367 - t1411 + t1412 + t1426 + t1355 * (t543 - thetaVar_dot))
        + t371 * t779 * t1960
        - t372 * t779 * t1959,
    
        -t372 * t779 * (-t1366 + t1367 - t1411 + t1412 + t1426 + t1355 * (t543 - thetaVar_dot))
        + t370 * t779 * t1960
        + t422 * t779 * t1959,
    
        -t371 * t779 * (-t1366 + t1367 - t1411 + t1412 + t1426 + t1355 * (t543 - thetaVar_dot))
        - t370 * t779 * t1959
        - t421 * t779 * t1960,
    
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    
        t372 * t779 * t1901
        - t371 * t779 * t1953
        - t423 * t779 * t1962,
    
        -t370 * t779 * t1953
        - t422 * t779 * t1901
        + t372 * t779 * t1962,
    
        t370 * t779 * t1901
        + t371 * t779 * t1962
        + t421 * t779 * t1953,
    
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0
    };

    double mt13[] = {
        t372 * t779 * t1956
        + t423 * t779 * t1961
        + t371 * t779 * (-t1273 + t1321 + t1442 + t1481 + t1483 + t1262 * (t543 - thetaVar_dot)),
    
        -t372 * t779 * t1961
        - t422 * t779 * t1956
        + t370 * t779 * (-t1273 + t1321 + t1442 + t1481 + t1483 + t1262 * (t543 - thetaVar_dot)),
    
        t370 * t779 * t1956
        - t371 * t779 * t1961
        - t421 * t779 * (-t1273 + t1321 + t1442 + t1481 + t1483 + t1262 * (t543 - thetaVar_dot))
    };
    
    Matrix A_flat(144, 1, 0.0);  // 144 rows, 1 column
    unsigned int idx = 0;

    for (size_t i = 0; i < sizeof(mt1) / sizeof(mt1[0]); ++i) A_flat(idx++, 0) = mt1[i];
    for (size_t i = 0; i < sizeof(mt2) / sizeof(mt2[0]); ++i) A_flat(idx++, 0) = mt2[i];
    for (size_t i = 0; i < sizeof(mt3) / sizeof(mt3[0]); ++i) A_flat(idx++, 0) = mt3[i];
    for (size_t i = 0; i < sizeof(mt4) / sizeof(mt4[0]); ++i) A_flat(idx++, 0) = mt4[i];
    for (size_t i = 0; i < sizeof(mt5) / sizeof(mt5[0]); ++i) A_flat(idx++, 0) = mt5[i];
    for (size_t i = 0; i < sizeof(mt6) / sizeof(mt6[0]); ++i) A_flat(idx++, 0) = mt6[i];
    for (size_t i = 0; i < sizeof(mt7) / sizeof(mt7[0]); ++i) A_flat(idx++, 0) = mt7[i];
    for (size_t i = 0; i < sizeof(mt8) / sizeof(mt8[0]); ++i) A_flat(idx++, 0) = mt8[i];
    for (size_t i = 0; i < sizeof(mt9) / sizeof(mt9[0]); ++i) A_flat(idx++, 0) = mt9[i];
    for (size_t i = 0; i < sizeof(mt10) / sizeof(mt10[0]); ++i) A_flat(idx++, 0) = mt10[i];
    for (size_t i = 0; i < sizeof(mt11) / sizeof(mt11[0]); ++i) A_flat(idx++, 0) = mt11[i];
    for (size_t i = 0; i < sizeof(mt12) / sizeof(mt12[0]); ++i) A_flat(idx++, 0) = mt12[i];
    for (size_t i = 0; i < sizeof(mt13) / sizeof(mt13[0]); ++i) A_flat(idx++, 0) = mt13[i];


    Matrix A(12, 12, 0);

    for (unsigned int i = 0; i < 12; ++i) {
        for (unsigned int j = 0; j < 12; ++j) {
            A(i, j) = A_flat(j * 12 + i, 0); 
        }
    }

    return A;

}