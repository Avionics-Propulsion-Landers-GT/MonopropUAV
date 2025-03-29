#include <Eigen/Dense>
#include "../Matrix.h"
#include "../Vector.h"
#include <iostream>

using Eigen::Matrix3d;
using Eigen::MatrixXd;
using Eigen::Vector3d;
using Eigen::VectorXd;

#include <cmath>

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
 * @return std::pair<MatrixXd, MatrixXd>  A and B matrices (12x12, 12x7)
 */
Matrix calculateB(
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

    double t2 = cos(a);
    double t3 = cos(b);
    // double t4 = conj(m);  // If 'm' is real, remove conj. If complex, replace with std::conj(m);
    double t4 = m;  // Assuming real
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
    double t27 = Ixz_b * t12;
    double t28 = Iyz_a * t12;
    double t29 = Iyz_b * t12;
    double t30 = Izz_a * t12;
    double t31 = Izz_b * t12;

    double t32 = rt_x * t2;
    double t33 = rt_z * t2;
    double t34 = rt_x * t7;
    double t35 = rt_z * t7;
    double t36 = rt_x * t8;
    double t37 = rt_z * t8;

    double t38 = t5 * t6;
    double t39 = t5 * t11;
    double t40 = t6 * t10;
    double t41 = t10 * t11;

    double t42 = 1.0 / t4;

    double t43 = 2.0 * Iyz * t16;
    double t44 = Ixx * t15;
    double t45 = Iyy * t14;
    double t46 = Izz * t13;

    double t47 = T * rt_x * t3;
    double t48 = T * rt_z * t3;

    double t49 = -t17;
    double t50 = -t18;
    double t51 = -t20;
    double t52 = -t22;
    double t53 = -t23;
    double t54 = -t24;

    double t55 = -t43;
    double t56 = -t25;

    double t57 = Ixx_a * t5 * t9;
    double t58 = Ixx_b * t5 * t9;
    double t59 = Ixy_a * t5 * t9;
    double t60 = Ixy_b * t5 * t9;
    double t61 = Ixz_a * t5 * t9;
    double t62 = Ixz_b * t5 * t9;

    double t63 = Ixz_a * t6 * t9;
    double t64 = Ixz_b * t6 * t9;
    double t65 = Iyz_a * t6 * t9;
    double t66 = Iyz_b * t6 * t9;
    double t67 = Izz_a * t6 * t9;
    double t68 = Izz_b * t6 * t9;

    double t69 = rt_y * t2 * t3;

    double t70 = Ixy_a * t9 * t10;
    double t71 = Ixy_b * t9 * t10;
    double t72 = Iyy_a * t9 * t10;
    double t73 = Iyy_b * t9 * t10;
    double t74 = Iyz_a * t9 * t10;
    double t75 = Iyz_b * t9 * t10;

    double t76 = Ixz_a * t9 * t11;
    double t77 = Ixz_b * t9 * t11;
    double t78 = Iyz_a * t9 * t11;
    double t79 = Iyz_b * t9 * t11;
    double t80 = Izz_a * t9 * t11;
    double t81 = Izz_b * t9 * t11;

    double t82 = rt_y * t3 * t7;

    double t83 = a_dot * t5 * t9;
    double t84 = b_dot * t9 * t10;

    double t85 = -t26;
    double t86 = -t27;
    double t87 = -t28;
    double t88 = -t29;
    double t89 = -t30;
    double t90 = -t31;
    double t91 = -t34;

    double t92 = t12 * t41;
    double t93 = T * rt_y * t2 * t8;
    double t94 = T * rt_y * t7 * t8;

    double t95 = t12 * t38;
    double t96 = t12 * t39;
    double t97 = t12 * t40;

    double t98 = psiVar_dot + t83;
    double t99 = psiVar_dot + t84;

    double t100 = -t82;
    double t101 = t16 + t50;
    double t102 = t19 + t51;
    double t103 = t21 + t53;

    double t104 = T * t12 * t69;
    double t105 = t32 + t35;
    double t106 = -t93;
    double t107 = t13 + t49;
    double t108 = t14 + t52;
    double t109 = t15 + t54;

    double t110 = -t96;
    double t111 = -t97;
    double t112 = T * t6 * t9 * t69;
    double t113 = t37 + t69;
    double t114 = T * t5 * t9 * t82;
    double t115 = T * t9 * t11 * t69;

    double t116 = t33 + t91;
    double t117 = t47 + t94;
    double t118 = t36 + t100;

    double t119 = t38 + t92;
    double t120 = t41 + t95;
    double t121 = t48 + t106;

    double t122 = Ixx_a * t120;

    double t138 = t12 * t117;
    double t139 = t5 * t9 * t113;
    double t140 = t3 * t9 * t10 * t105;

    double t158 = T * t8 * t9 * t10 * t105;
    double t159 = t6 * t9 * t117;
    double t160 = t9 * t11 * t117;

    double t178 = t57 + t70 + t85;
    double t179 = t58 + t71 + t86;
    double t180 = t59 + t72 + t87;
    double t181 = t60 + t73 + t88;
    double t182 = t61 + t74 + t89;
    double t183 = t62 + t75 + t90;

    double t212 = t44 + t45 + t46 + t55 + t56;

    double t123 = Ixx_b * t120;
    double t124 = Ixy_a * t119;
    double t125 = Ixy_a * t120;
    double t126 = Ixy_b * t119;
    double t127 = Ixy_b * t120;
    double t128 = Ixz_a * t120;
    double t129 = Ixz_b * t120;
    double t130 = Iyy_a * t119;
    double t131 = Iyy_b * t119;
    double t132 = Iyz_a * t119;
    double t133 = Iyz_b * t119;
    double t134 = a_dot * t120;
    double t135 = b_dot * t119;
    double t136 = t39 + t111;
    double t137 = t40 + t110;

    double t141 = t12 * t118;
    double t142 = phiVar_dot + t134;
    double t143 = thetaVar_dot + t135;

    double t144 = Ixx_a * t137;
    double t145 = Ixx_b * t137;
    double t146 = Ixy_a * t136;
    double t147 = Ixy_a * t137;
    double t148 = Ixy_b * t136;
    double t149 = Ixy_b * t137;
    double t150 = Ixz_a * t137;
    double t151 = Ixz_b * t137;
    double t152 = Iyy_a * t136;
    double t153 = Iyy_b * t136;
    double t154 = Iyz_a * t136;
    double t155 = Iyz_b * t136;
    double t156 = a_dot * t137;
    double t157 = b_dot * t136;
    double t161 = -t140;
    double t162 = t6 * t9 * t118;
    double t163 = t9 * t11 * t118;

    double t164 = -t144;
    double t165 = -t145;
    double t166 = -t146;
    double t167 = -t147;
    double t168 = -t148;
    double t169 = -t149;
    double t170 = -t150;
    double t171 = -t151;
    double t172 = -t152;
    double t173 = -t153;
    double t174 = -t154;
    double t175 = -t155;
    double t177 = -t157;


    // double t178 = t57 + t70 + (-t26);
    // double t179 = t58 + t71 + (-t27);
    // double t180 = t59 + t72 + (-t28);
    // double t181 = t60 + t73 + (-t29);
    // double t182 = t61 + t74 + (-t30);
    // double t183 = t62 + t75 + (-t31);
    double t184 = T * t3 * t9 * t10 * t116;
    double t185 = t5 * t9 * t121;
    double t187 = -t162;
    double t188 = phiVar_dot + t177;
    double t190 = T * t82 * t137;

    double t191 = T * t100 * t120;
    double t192 = t12 * t182;
    double t193 = t12 * t183;
    double t194 = t5 * t9 * t178;
    double t195 = t5 * t9 * t179;
    double t196 = t6 * t9 * t182;
    double t197 = t6 * t9 * t183;
    double t198 = t9 * t10 * t180;
    double t199 = t9 * t10 * t181;
    double t200 = t9 * t11 * t182;
    double t201 = t9 * t11 * t183;
    double t202 = -t192;
    double t203 = -t193;
    double t204 = t3 * t105 * t119;
    double t205 = t113 * t120;
    double t206 = T * t8 * t105 * t119;
    double t207 = t3 * t105 * t136;
    double t208 = t113 * t137;
    double t209 = T * t3 * t116 * t119;
    double t210 = T * t8 * t105 * t136;
    double t211 = -t206;
    double t213 = t120 * t121;
    double t214 = T * t3 * t116 * t136;
    double t215 = -t209;
    double t216 = 1.0 / t212;
    double t217 = t121 * t137;
    double t218 = -t213;
    double t219 = t120 * t178;
    double t220 = t120 * t179;
    double t221 = t119 * t180;
    double t222 = t119 * t181;
    double t223 = t137 * t178;
    double t224 = t137 * t179;
    double t225 = t136 * t180;
    double t226 = t136 * t181;
    double t227 = -t223;
    double t228 = -t224;
    double t229 = -t225;
    double t230 = -t226;
    double t231 = t104 + t114 + t184;

    double t232 = t76 + t124 + t164;
    double t233 = t77 + t126 + t165;
    double t234 = t78 + t130 + t167;
    double t235 = t79 + t131 + t169;
    double t236 = t80 + t132 + t170;
    double t237 = t81 + t133 + t171;
    double t238 = t63 + t122 + t166;
    double t239 = t64 + t123 + t168;
    double t240 = t65 + t125 + t172;
    double t241 = t66 + t127 + t173;
    double t242 = t67 + t128 + t174;
    double t243 = t68 + t129 + t175;

    double t244 = t12 * t242;
    double t245 = t12 * t243;
    double t246 = t12 * t236;
    double t247 = t12 * t237;

    double t248 = t5 * t9 * t232;
    double t249 = t5 * t9 * t233;
    double t250 = t9 * t10 * t240;
    double t251 = t9 * t10 * t241;
    double t252 = t6 * t9 * t236;
    double t253 = t9 * t11 * t242;
    double t254 = t6 * t9 * t237;
    double t255 = t9 * t11 * t243;
    double t256 = t9 * t10 * t234;
    double t257 = t9 * t10 * t235;
    double t258 = t9 * t11 * t236;
    double t259 = t9 * t11 * t237;

    double t260 = -t244;
    double t261 = -t245;
    double t262 = -t246;
    double t263 = -t247;

    double t264 = t5 * t9 * t238;
    double t265 = t5 * t9 * t239;
    double t266 = t6 * t9 * t242;
    double t267 = t6 * t9 * t243;

    double t268 = t139 + t141 + t161;
    double t269 = t138 + t158 + t185;

    double t270 = t120 * t238;
    double t271 = t120 * t239;
    double t272 = t119 * t240;
    double t273 = t119 * t241;
    double t274 = t120 * t232;
    double t275 = t120 * t233;
    double t276 = t119 * t234;
    double t277 = t119 * t235;

    double t278 = t137 * t238;
    double t279 = t137 * t239;
    double t280 = t136 * t240;
    double t281 = t136 * t241;
    double t282 = t137 * t232;
    double t283 = t137 * t233;
    double t284 = t136 * t234;
    double t285 = t136 * t235;

    double t286 = -t278;
    double t287 = -t279;
    double t288 = -t280;
    double t289 = -t281;
    double t290 = -t282;
    double t291 = -t283;
    double t292 = -t284;
    double t293 = -t285;

    double t294 = t112 + t191 + t214;
    double t295 = t115 + t190 + t215;
    double t296 = t163 + t204 + t208;
    double t297 = t187 + t205 + t207;
    double t298 = t194 + t198 + t202;
    double t299 = t195 + t199 + t203;

    double t300 = t160 + t211 + t217;
    double t301 = t5 * t9 * t298;
    double t302 = t9 * t10 * t299;
    double t303 = t159 + t210 + t218;
    double t304 = t98 * t298;
    double t305 = t99 * t299;
    double t306 = t196 + t219 + t229;
    double t307 = t197 + t220 + t230;
    double t308 = t200 + t221 + t227;
    double t309 = t201 + t222 + t228;

    double t310 = t120 * t306;
    double t311 = t119 * t309;
    double t312 = t136 * t307;
    double t313 = t137 * t308;

    double t314 = t142 * t306;
    double t315 = t143 * t309;
    double t316 = -t312;
    double t317 = -t313;
    double t318 = t188 * t307;
    double t319 = -t308 * (t156 - thetaVar_dot);

    double t320 = t250 + t260 + t264;
    double t321 = t251 + t261 + t265;
    double t322 = t248 + t256 + t262;
    double t323 = t249 + t257 + t263;

    double t324 = t5 * t9 * t320;
    double t325 = t9 * t10 * t321;
    double t326 = t5 * t9 * t322;
    double t327 = t9 * t10 * t323;

    double t328 = t98 * t320;
    double t329 = t99 * t321;
    double t330 = t98 * t322;
    double t331 = t99 * t323;

    double t332 = t252 + t274 + t292;
    double t333 = t254 + t275 + t293;
    double t334 = t258 + t276 + t290;
    double t335 = t259 + t277 + t291;
    double t336 = t266 + t270 + t288;
    double t337 = t267 + t271 + t289;
    double t338 = t253 + t272 + t286;
    double t339 = t255 + t273 + t287;

    double t340 = t120 * t336;
    double t341 = t119 * t339;
    double t342 = t120 * t332;
    double t343 = t119 * t335;
    double t344 = t136 * t337;
    double t345 = t137 * t338;
    double t346 = t136 * t333;
    double t347 = t137 * t334;

    double t348 = t142 * t336;
    double t349 = t143 * t339;

    double t360 = t301 + t310 + t317;
    double t361 = t302 + t311 + t316;

    double t362 = t304 + t314 + t319;
    double t363 = t305 + t315 + t318;

    double t371 = t120 * t362;
    double t372 = t119 * t363;
    double t373 = t137 * t362;
    double t375 = t136 * t363;

    double t350 = t142 * t332;
    double t351 = t143 * t335;
    double t356 = t188 * t337;
    double t357 = -t338 * (t156 - thetaVar_dot);
    double t358 = t188 * t333;
    double t359 = -t334 * (t156 - thetaVar_dot);

    double t364 = t142 * t360;
    double t365 = t143 * t361;
    double t368 = t188 * t361;
    double t369 = t360 * (t156 - thetaVar_dot);

    double t352 = -t344;
    double t353 = -t345;
    double t354 = -t346;
    double t355 = -t347;
    double t370 = -t368;

    double t385 = t328 + t348 + t357;
    double t386 = t329 + t349 + t356;
    double t387 = t330 + t350 + t359;
    double t388 = t331 + t351 + t358;

    double t376 = t326 + t342 + t355;
    double t377 = t327 + t343 + t354;
    double t378 = t324 + t340 + t353;
    double t379 = t325 + t341 + t352;

    double t389 = t9 * t10 * t388;
    double t390 = t5 * t9 * t385;
    double t391 = t9 * t10 * t386;
    double t392 = t5 * t9 * t387;

    double t400 = t119 * t386;
    double t401 = t120 * t387;
    double t402 = t137 * t385;
    double t403 = t136 * t388;

    double t380 = t98 * t378;
    double t381 = t99 * t379;
    double t382 = t98 * t376;
    double t383 = t99 * t377;

    double t393 = t143 * t379;
    double t394 = t142 * t376;

    double t395 = -t389;
    double t396 = t188 * t377;
    double t399 = t378 * (t156 - thetaVar_dot);
    double t384 = -t383;
    double t398 = -t396;

    double t406 = t369 + t373 + t382 + t392;
    double t407 = t370 + t375 + t381 + t391;
    double t408 = t394 + t399 + t401 + t402;
    double t404 = t365 + t372 + t384 + t395;
    double t409 = t393 + t398 + t400 + t403;

    std::vector<double> mt1 = {
        0.0, 0.0, 0.0,
        t42 * (t2 * t3 * t12 + t8 * t9 * t10 + t3 * t5 * t7 * t9),
        -t42 * (-t8 * t119 + t3 * t7 * t137 + t2 * t3 * t9 * t11),
        -t42 * (t8 * t136 - t3 * t7 * t120 + t2 * t3 * t6 * t9),
        0.0, 0.0, 0.0,
        -t109 * t216 * t268 + t102 * t216 * t297 - t103 * t216 * t296,
         t103 * t216 * t268 + t101 * t216 * t297 + t108 * t216 * t296,
         t102 * t216 * t268 - t101 * t216 * t296 - t107 * t216 * t297,
        0.0, 0.0, 0.0,
        -t42 * (T * t3 * t7 * t12 - T * t2 * t3 * t5 * t9),
        -t42 * (T * t2 * t3 * t137 - T * t3 * t7 * t9 * t11),
         t42 * (T * t2 * t3 * t120 + T * t3 * t6 * t7 * t9),
        0.0, 0.0, 0.0,
         t109 * t216 * t231 + t102 * t216 * t294 + t103 * t216 * t295,
        -t103 * t216 * t231 + t101 * t216 * t294 - t108 * t216 * t295
    };

    std::vector<double> mt2 = {
        -t102 * t216 * t231 + t101 * t216 * t295 - t107 * t216 * t294,
        0.0, 0.0, 0.0,
        -t42 * (T * t2 * t8 * t12 - T * t3 * t9 * t10 + T * t5 * t7 * t8 * t9),
         t42 * (T * t3 * t119 + T * t7 * t8 * t137 + T * t2 * t8 * t9 * t11),
        -t42 * (T * t3 * t136 + T * t7 * t8 * t120 - T * t2 * t6 * t8 * t9),
        0.0, 0.0, 0.0,
        -t109 * t216 * t269 - t103 * t216 * t300 - t102 * t216 * t303,
         t103 * t216 * t269 - t101 * t216 * t303 + t108 * t216 * t300,
         t102 * t216 * t269 - t101 * t216 * t300 + t107 * t216 * t303,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
         t103 * t216 * (t364 + t371 - t380 - t390) + t102 * t216 * t406 + t109 * t216 * t408,
        -t108 * t216 * (t364 + t371 - t380 - t390) + t101 * t216 * t406 - t103 * t216 * t408
    };

    std::vector<double> mt3 = {
        t101 * t216 * (t364 + t371 - t380 - t390) - t102 * t216 * t408 - t107 * t216 * t406,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       -t102 * t216 * t404 - t103 * t216 * t407 - t109 * t216 * t409,
       -t101 * t216 * t404 + t103 * t216 * t409 + t108 * t216 * t407,
       -t101 * t216 * t407 + t102 * t216 * t409 + t107 * t216 * t404,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        t109 * t178 * t216 - t103 * t216 * t232 - t102 * t216 * t238,
       -t103 * t178 * t216 - t101 * t216 * t238 + t108 * t216 * t232,
       -t102 * t178 * t216 - t101 * t216 * t232 + t107 * t216 * t238,
       0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        t109 * t181 * t216 - t103 * t216 * t235 - t102 * t216 * t241,
       -t103 * t181 * t216 - t101 * t216 * t241 + t108 * t216 * t235
    };

    std::vector<double> mt4 = {
        -t102 * t181 * t216 - t101 * t216 * t235 + t107 * t216 * t241
    };

    std::vector<double> B_flat;
    B_flat.reserve(84);  // 12 rows * 7 columns

    B_flat.insert(B_flat.end(), mt1.begin(), mt1.end());
    B_flat.insert(B_flat.end(), mt2.begin(), mt2.end());
    B_flat.insert(B_flat.end(), mt3.begin(), mt3.end());
    B_flat.insert(B_flat.end(), mt4.begin(), mt4.end());

    // Create the B matrix (12 rows, 7 columns)
    Matrix B(12, 7, 0.0);  // 12 rows, 7 columns

    // Fill the matrix with the flattened vector data (7x12)
    for (unsigned int i = 0; i < 7; ++i) {   // Iterate over columns (7 columns)
        for (unsigned int j = 0; j < 12; ++j) {  // Iterate over rows (12 rows)
            B(j, i) = B_flat[i * 12 + j];  
        }
    }
    
    return B;
    
}