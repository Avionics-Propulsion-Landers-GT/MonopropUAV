#include "../Matrix.h"
#include "../Vector.h"
#include <vector>
#include <iostream>
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
 * @param inertia_a    Inertia tensor of TVC element a (3x3)
 * @param inertia_b    Inertia tensor of TVC element b (3x3)
 * @return std::pair<MatrixXd, MatrixXd>  A and B matrices (12x12, 12x7)
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
    const Matrix& inertia_a,
    const Matrix& inertia_b,
    const Vector& angular_states
) {

    // Extract from full inertia tensor
    double Ixx   = inertia(0, 0);
    double Ixy   = inertia(0, 1);
    double Ixz   = inertia(0, 2);
    double Iyy   = inertia(1, 1);
    double Iyz   = inertia(1, 2);
    double Izz   = inertia(2, 2);


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
    double a_dot      = angular_states[0];
    double b_dot      = angular_states[1];
    double a_dot_dot     = angular_states[2];
    double b_dot_dot     = angular_states[3];


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
    double t2 = cos(a);
    double t3 = cos(b);
    double t4 = m; // assuming m is real, no conj
    double t5 = cos(phiVar);
    double t6 = cos(psiVar);
    double t7 = sin(a);
    double t8 = sin(b);
    double t9 = cos(thetaVar);
    double t10 = sin(phiVar);
    double t11 = sin(psiVar);
    double t12 = sin(thetaVar);
    double t13 = Ixx_a * a_dot_dot;
    double t14 = Ixy_a * a_dot_dot;
    double t15 = Ixz_a * a_dot_dot;
    double t16 = Ixy_b * b_dot_dot;
    double t17 = Iyy_b * b_dot_dot;
    double t18 = Iyz_b * b_dot_dot;
    double t19 = Ixy * Ixy;
    double t20 = Ixz * Ixz;
    double t21 = Iyz * Iyz;
    double t22 = Ixy * Ixz;
    double t23 = Ixx * Iyy;
    double t24 = Ixx * Iyz;
    double t25 = Ixy * Iyz;
    double t26 = Ixz * Iyy;
    double t27 = Ixz * Iyz;
    double t28 = Ixx * Izz;
    double t29 = Ixy * Izz;
    double t30 = Iyy * Izz;
    double t31 = t9 * t9;
    double t32 = Izz * t23;
    double t33 = rt_x * t2;
    double t34 = rt_z * t7;
    double t35 = t9 * v_x;
    double t36 = rc_x * t12;
    double t37 = rc_y * t12;
    double t38 = t12 * v_x;
    double t39 = t5 * t6;
    double t40 = t5 * t11;
    double t41 = t6 * t10;
    double t44 = t10 * t11;
    double t45 = 1.0 / t4;
    double t48 = Iyz * t22 * 2.0;
    double t49 = Ixx * t21;
    double t50 = Iyy * t20;
    double t51 = Izz * t19;
    double t52 = -t23;
    double t53 = -t24;
    double t54 = -t26;
    double t55 = -t28;
    double t56 = -t29;
    double t57 = -t30;
    double t58 = T * rt_x * t8;
    double t59 = T * rt_z * t8;
    double t64 = rc_y * t5 * t9;
    double t65 = rc_z * t5 * t9;
    double t68 = rc_x * t6 * t9;
    double t69 = rc_y * t6 * t9;
    double t71 = t6 * t9 * v_y;
    double t72 = t6 * t9 * v_z;
    double t77 = rc_x * t9 * t10;
    double t78 = rc_z * t9 * t10;
    double t83 = rc_x * t9 * t11;
    double t84 = rc_y * t9 * t11;
    double t87 = t9 * t11 * v_y;
    double t88 = t6 * t12 * v_z;
    double t89 = t9 * t11 * v_z;
    double t95 = t11 * t12 * v_y;
    double t118 = T * rt_y * t2 * t3;
    double t119 = T * rt_y * t3 * t7;
    double t42 = rc_x * t35;
    double t43 = rc_y * t35;
    double t46 = t36 * v_x;
    double t47 = t37 * v_x;
    double t60 = -t48;
    double t61 = -t32;
    double t62 = rc_x * t39;
    double t63 = rc_z * t39;
    double t66 = t39 * v_y;
    double t67 = t39 * v_z;
    double t70 = t5 * t35;
    double t73 = rc_x * t40;
    double t74 = rc_y * t41;
    double t75 = rc_z * t40;
    double t76 = rc_z * t41;
    double t79 = t40 * v_y;
    double t80 = t41 * v_y;
    double t81 = t40 * v_z;
    double t82 = t41 * v_z;
    double t85 = t5 * t38;
    double t86 = t10 * t35;
    double t90 = rc_y * t44;
    double t91 = rc_z * t44;
    double t92 = t44 * v_y;
    double t93 = t44 * v_z;
    double t94 = t10 * t38;
    double t96 = -t38;
    double t102 = t83 * v_y;
    double t103 = t84 * v_y;
    double t104 = t6 * t36 * v_z;
    double t105 = t83 * v_z;
    double t106 = t6 * t37 * v_z;
    double t107 = t84 * v_z;
    double t108 = t12 * t44;
    double t116 = t11 * t36 * v_y;
    double t117 = t11 * t37 * v_y;
    double t120 = t12 * t39;
    double t129 = t68 * v_y;
    double t130 = t69 * v_y;
    double t131 = t68 * v_z;
    double t132 = t69 * v_z;
    double t133 = -t58;
    double t134 = t12 * t40;
    double t135 = t12 * t41;
    double t147 = t37 * t39;
    double t153 = t36 * t41;
    double t154 = t37 * t40;
    double t162 = -t68;
    double t163 = t36 * t44;
    double t167 = -t77;
    double t170 = -t84;
    double t172 = -t89;
    double t181 = t22 + t53;
    double t182 = t25 + t54;
    double t183 = t27 + t56;
    double t184 = t33 + t34;
    double t217 = t19 + t52;
    double t218 = t20 + t55;
    double t219 = t21 + t57;
    double t233 = t36 + t65;
    double t234 = t37 + t78;
    double t255 = t35 + t88 + t95;
    double t97 = t10 * t42;
    double t98 = t5 * t47;
    double t99 = t10 * t43;
    double t100 = rc_z * t85;
    double t101 = rc_z * t86;
    double t109 = t90 * v_y;
    double t110 = t91 * v_y;
    double t111 = rc_x * t93;
    double t112 = t90 * v_z;
    double t113 = t91 * v_z;
    double t114 = t10 * t46;
    double t115 = rc_z * t94;
    double t121 = t62 * v_y;
    double t122 = rc_y * t66;
    double t123 = t63 * v_y;
    double t124 = t62 * v_z;
    double t125 = t63 * v_z;
    double t126 = t5 * t42;
    double t127 = t5 * t43;
    double t128 = rc_z * t70;
    double t136 = t73 * v_y;
    double t137 = rc_x * t80;
    double t138 = t74 * v_y;
    double t139 = t75 * v_y;
    double t140 = t76 * v_y;
    double t141 = t73 * v_z;
    double t142 = rc_y * t81;
    double t143 = t74 * v_z;
    double t144 = t75 * v_z;
    double t145 = t76 * v_z;
    double t146 = t9 * t67;
    double t148 = t12 * t63;
    double t149 = t12 * t66;
    double t150 = t9 * t79;
    double t151 = t12 * t67;
    double t152 = t9 * t82;
    double t155 = t12 * t75;
    double t156 = t12 * t76;
    double t157 = t12 * t79;
    double t158 = t12 * t80;
    double t159 = t9 * t92;
    double t160 = t12 * t81;
    double t161 = t12 * t82;
    double t164 = t12 * t91;
    double t165 = t12 * t92;
    double t166 = t12 * t93;
    double t168 = -t80;
    double t169 = -t81;
    double t171 = -t85;
    double t173 = -t94;
    double t175 = -t102;
    double t176 = -t103;
    double t177 = -t107;
    double t187 = t37 * t66;
    double t191 = t36 * t67;
    double t193 = t37 * t67;
    double t196 = t36 * t79;
    double t199 = t37 * t79;
    double t203 = t36 * t82;
    double t205 = t37 * t82;
    double t209 = -t129;
    double t210 = -t131;
    double t211 = -t132;
    double t212 = t36 * t92;
    double t213 = t37 * t92;
    double t215 = t36 * t93;
    double t220 = -t134;
    double t221 = -t135;
    double t227 = -t153;
    double t228 = -t154;
    double t243 = T * t3 * t184;
    double t244 = t39 + t108;
    double t245 = t44 + t120;
    double t251 = t64 + t167;
    double t252 = t71 + t172;
    double t260 = t72 + t87 + t96;
    double t277 = t49 + t50 + t51 + t60 + t61;
    double t174 = -t100;
    double t178 = -t112;
    double t179 = -t114;
    double t180 = -t115;
    double t185 = rc_y * t146;
    double t186 = t9 * t125;
    double t188 = rc_y * t150;
    double t189 = t12 * t123;
    double t190 = t9 * t139;
    double t192 = rc_x * t152;
    double t194 = t12 * t125;
    double t195 = t9 * t145;
    double t198 = rc_x * t159;
    double t200 = t12 * t139;
    double t201 = t12 * t140;
    double t202 = t9 * t110;
    double t206 = t12 * t144;
    double t207 = t12 * t145;
    double t208 = -t127;
    double t214 = t12 * t110;
    double t216 = t12 * t113;
    double t222 = -t137;
    double t223 = -t140;
    double t224 = -t141;
    double t225 = -t142;
    double t226 = -t144;
    double t229 = -t155;
    double t230 = -t156;
    double t231 = -t158;
    double t232 = -t160;
    double t237 = -t193;
    double t238 = t36 * t168;
    double t239 = -t199;
    double t241 = t37 * t169;
    double t246 = -t243;
    double t247 = t244 * v_y;
    double t248 = t245 * v_y;
    double t249 = t244 * v_z;
    double t250 = t245 * v_z;
    double t253 = t40 + t221;
    double t254 = t41 + t220;
    double t263 = std::abs(t260);
    double t264 = sign(t260);
    double t265 = t260 * t260;
    double t267 = 2.0 * t12 * t260;
    double t268 = t91 + t148 + t162;
    double t271 = t63 + t164 + t170;
    double t273 = 2.0 * t6 * t9 * t260;
    double t274 = 2.0 * t9 * t11 * t260;
    double t275 = t146 + t150 + t171;
    double t276 = t152 + t159 + t173;
    double t278 = 1.0 / t277;
    double t279 = t62 + t74 + t163 + t228;
    double t280 = t73 + t90 + t147 + t227;
    double t285 = 2.0 * t252 * t260;
    double t286 = t70 + t93 + t151 + t157 + t168;
    double t287 = t66 + t86 + t161 + t165 + t169;
    double t288 = 2.0 * t255 * t260;
    double t235 = -t185;
    double t236 = -t188;
    double t240 = -t201;
    double t242 = -t206;
    double t256 = t253 * v_y;
    double t257 = t254 * v_y;
    double t258 = t253 * v_z;
    double t259 = t254 * v_z;
    double t266 = t263 * t263;
    double t269 = t69 + t75 + t230;
    double t270 = t76 + t83 + t229;
    double t272 = -t267;
    double t281 = t67 + t79 + t166 + t231;
    double t282 = t82 + t92 + t149 + t232;
    double t289 = t286 * t286;
    double t290 = t287 * t287;
    double t291 = -t288;
    double t294 = t113 + t128 + t194 + t200 + t223;
    double t295 = t101 + t123 + t207 + t214 + t226;
    double t300 = 2.0 * t9 * t10 * t287;
    double t301 = 2.0 * t5 * t9 * t286;
    double t304 = 2.0 * t12 * t263 * t264;
    double t306 = 2.0 * t6 * t9 * t263 * t264;
    double t307 = 2.0 * t9 * t11 * t263 * t264;
    double t308 = t42 + t104 + t116 + t174 + t186 + t190;
    double t309 = t43 + t106 + t117 + t180 + t195 + t202;
    double t312 = 2.0 * t245 * t286;
    double t313 = 2.0 * t244 * t287;
    double t314 = 2.0 * t254 * t286;
    double t315 = 2.0 * t253 * t287;
    double t318 = 2.0 * t252 * t263 * t264;
    double t320 = 2.0 * t255 * t263 * t264;
    double t322 = 2.0 * t276 * t287;
    double t323 = 2.0 * t275 * t286;
    double t326 = t109 + t124 + t136 + t143 + t187 + t215 + t238 + t241;
    double t330 = t99 + t111 + t122 + t126 + t191 + t196 + t205 + t213 + t222 + t225;
    double t331 = t97 + t121 + t138 + t178 + t203 + t208 + t212 + t224 + t237 + t239;
    double t261 = -t257;
    double t262 = -t258;
    double t283 = t248 + t259;
    double t284 = t249 + t256;
    double t305 = -t304;
    double t310 = t105 + t110 + t145 + t189 + t209 + t242;
    double t311 = t125 + t130 + t139 + t177 + t216 + t240;
    double t316 = -t314;
    double t317 = -t315;
    double t319 = t98 + t179 + t192 + t198 + t235 + t236;
    double t321 = -t320;
    double t324 = t46 + t175 + t210 + t294;
    double t325 = t47 + t176 + t211 + t295;
    double t327 = 2.0 * t282 * t286;
    double t328 = 2.0 * t281 * t287;
    double t336 = t265 + t289 + t290;
    double t346 = t272 + t300 + t301;
    double t390 = t291 + t322 + t323;
    double t292 = t70 + t250 + t261;
    double t293 = t86 + t247 + t262;
    double t329 = -t328;
    double t341 = std::sqrt(t336);
    double t362 = t273 + t312 + t317;
    double t363 = t274 + t313 + t316;
    double t296 = std::abs(t292);
    double t297 = std::abs(t293);
    double t298 = sign(t292);
    double t299 = sign(t293);
    double t342 = 1.0 / t341;
    double t352 = (areaVar * cDrag * f * t233 * t341) / 2.0;
    double t353 = (areaVar * cDrag * f * t234 * t341) / 2.0;
    double t354 = (areaVar * cDrag * f * t251 * t341) / 2.0;
    double t364 = (areaVar * cDrag * f * t268 * t341) / 2.0;
    double t365 = (areaVar * cDrag * f * t269 * t341) / 2.0;
    double t366 = (areaVar * cDrag * f * t270 * t341) / 2.0;
    double t367 = (areaVar * cDrag * f * t271 * t341) / 2.0;
    double t374 = (areaVar * cDrag * f * t279 * t341) / 2.0;
    double t375 = (areaVar * cDrag * f * t280 * t341) / 2.0;
    double t378 = (areaVar * cDrag * f * t5 * t9 * t294 * t341) / 2.0;
    double t379 = (areaVar * cDrag * f * t9 * t10 * t295 * t341) / 2.0;
    double t380 = (areaVar * cDrag * f * t308 * t341) / 2.0;
    double t381 = (areaVar * cDrag * f * t309 * t341) / 2.0;
    double t382 = (areaVar * cDrag * f * t310 * t341) / 2.0;
    double t383 = (areaVar * cDrag * f * t311 * t341) / 2.0;
    double t384 = (areaVar * cDrag * f * t245 * t294 * t341) / 2.0;
    double t385 = (areaVar * cDrag * f * t244 * t295 * t341) / 2.0;
    double t387 = (areaVar * cDrag * f * t254 * t294 * t341) / 2.0;
    double t388 = (areaVar * cDrag * f * t253 * t295 * t341) / 2.0;
    double t389 = (areaVar * cDrag * f * t319 * t341) / 2.0;
    double t392 = (areaVar * cDrag * f * t324 * t341) / 2.0;
    double t393 = (areaVar * cDrag * f * t325 * t341) / 2.0;
    double t395 = t285 + t327 + t329;
    double t396 = (areaVar * cDrag * f * t326 * t341) / 2.0;
    double t397 = (areaVar * cDrag * f * t12 * t330 * t341) / 2.0;
    double t399 = (areaVar * cDrag * f * t6 * t9 * t330 * t341) / 2.0;
    double t400 = (areaVar * cDrag * f * t9 * t11 * t330 * t341) / 2.0;
    double t403 = (areaVar * cDrag * f * t331 * t341) / 2.0;
    double t302 = t296 * t296;
    double t303 = t297 * t297;
    double t332 = 2.0 * t5 * t9 * t296 * t298;
    double t333 = 2.0 * t9 * t10 * t297 * t299;
    double t334 = 2.0 * t245 * t296 * t298;
    double t335 = 2.0 * t244 * t297 * t299;
    double t337 = 2.0 * t254 * t296 * t298;
    double t338 = 2.0 * t253 * t297 * t299;
    double t343 = 2.0 * t276 * t297 * t299;
    double t344 = 2.0 * t275 * t296 * t298;
    double t349 = 2.0 * t283 * t296 * t298;
    double t350 = 2.0 * t284 * t297 * t299;
    double t355 = 2.0 * t293 * t296 * t298;
    double t356 = 2.0 * t292 * t297 * t299;
    double t386 = -t384;
    double t391 = -t387;
    double t394 = -t393;
    double t401 = t14 + t17 + t246 + t392;
    double t402 = -t400;
    double t425 = t15 + t18 + t119 + t133 + t403;
    double t438 = (areaVar * cDrag * f * t324 * t342 * t346) / 4.0;
    double t439 = (areaVar * cDrag * f * t325 * t342 * t346) / 4.0;
    double t441 = (areaVar * cDrag * f * t325 * t342 * t363) / 4.0;
    double t442 = (areaVar * cDrag * f * t324 * t342 * t362) / 4.0;
    double t443 = (areaVar * cDrag * f * t324 * t342 * t363) / 4.0;
    double t444 = (areaVar * cDrag * f * t325 * t342 * t362) / 4.0;
    double t447 = (areaVar * cDrag * f * t331 * t342 * t346) / 4.0;
    double t449 = (areaVar * cDrag * f * t331 * t342 * t362) / 4.0;
    double t450 = (areaVar * cDrag * f * t331 * t342 * t363) / 4.0;
    double t452 = (areaVar * cDrag * f * t324 * t342 * t390) / 4.0;
    double t453 = (areaVar * cDrag * f * t325 * t342 * t390) / 4.0;
    double t454 = (areaVar * cDrag * f * t324 * t342 * t395) / 4.0;
    double t455 = (areaVar * cDrag * f * t325 * t342 * t395) / 4.0;
    double t457 = (areaVar * cDrag * f * t331 * t342 * t390) / 4.0;
    double t458 = (areaVar * cDrag * f * t331 * t342 * t395) / 4.0;
    double t339 = -t337;
    double t340 = -t338;
    double t345 = t266 + t302 + t303;
    double t351 = -t350;
    double t359 = -t356;
    double t398 = t13 + t16 + t59 + t118 + t394;
    double t406 = t5 * t9 * t401;
    double t407 = t10 * t12 * t401;
    double t410 = t9 * t41 * t401;
    double t411 = t9 * t44 * t401;
    double t417 = t244 * t401;
    double t418 = t245 * t401;
    double t422 = t253 * t401;
    double t423 = t254 * t401;
    double t426 = t9 * t425;
    double t428 = t305 + t332 + t333;
    double t429 = t6 * t12 * t425;
    double t431 = t11 * t12 * t425;
    double t437 = t321 + t343 + t344;
    double t445 = -t443;
    double t446 = -t444;
    double t448 = -t447;
    double t451 = -t449;
    double t456 = -t455;
    double t459 = -t458;
    double t460 = t352 + t438;
    double t461 = t353 + t439;
    double t471 = t364 + t442;
    double t474 = t367 + t441;
    double t494 = t374 + t450;
    double t502 = t380 + t452;
    double t503 = t381 + t453;
    double t508 = t382 + t454;
    double t521 = t389 + t457;
    double t347 = std::sqrt(t345);
    double t404 = t5 * t12 * t398;
    double t405 = t9 * t10 * t398;
    double t408 = t9 * t39 * t398;
    double t409 = t9 * t40 * t398;
    double t412 = -t406;
    double t415 = t244 * t398;
    double t416 = t245 * t398;
    double t419 = t253 * t398;
    double t420 = t254 * t398;
    double t427 = t6 * t426;
    double t430 = t11 * t426;
    double t433 = -t431;
    double t434 = t307 + t335 + t339;
    double t435 = t306 + t334 + t340;
    double t436 = t355 + t359;
    double t440 = t318 + t349 + t351;
    double t462 = t9 * t10 * t460;
    double t463 = t5 * t9 * t461;
    double t465 = t244 * t460;
    double t466 = t245 * t461;
    double t467 = t253 * t460;
    double t468 = t254 * t461;
    double t469 = t354 + t448;
    double t475 = t365 + t446;
    double t476 = t366 + t445;
    double t478 = t9 * t10 * t471;
    double t479 = t5 * t9 * t474;
    double t482 = t244 * t471;
    double t483 = t245 * t474;
    double t486 = t253 * t471;
    double t488 = t254 * t474;
    double t495 = t375 + t451;
    double t496 = t12 * t494;
    double t498 = t6 * t9 * t494;
    double t499 = t9 * t11 * t494;
    double t504 = t9 * t10 * t502;
    double t505 = t5 * t9 * t503;
    double t507 = t245 * t503;
    double t509 = t244 * t502;
    double t510 = t383 + t456;
    double t511 = t254 * t503;
    double t512 = t253 * t502;
    double t513 = t9 * t10 * t508;
    double t515 = t244 * t508;
    double t517 = t253 * t508;
    double t522 = t12 * t521;
    double t523 = t6 * t9 * t521;
    double t524 = t9 * t11 * t521;
    double t526 = t396 + t459;
    double t348 = 1.0 / t347;
    double t357 = (areaVar * cDrag * f * t6 * t9 * t12 * t347) / 2.0;
    double t358 = (areaVar * cDrag * f * t9 * t11 * t12 * t347) / 2.0;
    double t360 = (areaVar * cDrag * f * t6 * t11 * t31 * t347) / 2.0;
    double t368 = (areaVar * cDrag * f * t5 * t9 * t245 * t347) / 2.0;
    double t369 = (areaVar * cDrag * f * t9 * t10 * t244 * t347) / 2.0;
    double t372 = (areaVar * cDrag * f * t5 * t9 * t254 * t347) / 2.0;
    double t373 = (areaVar * cDrag * f * t9 * t10 * t253 * t347) / 2.0;
    double t376 = (areaVar * cDrag * f * t244 * t253 * t347) / 2.0;
    double t377 = (areaVar * cDrag * f * t245 * t254 * t347) / 2.0;
    double t424 = -t420;
    double t464 = -t463;
    double t470 = t12 * t469;
    double t472 = t6 * t9 * t469;
    double t473 = t9 * t11 * t469;
    double t480 = t5 * t9 * t475;
    double t481 = t9 * t10 * t476;
    double t484 = t245 * t475;
    double t485 = t244 * t476;
    double t487 = -t482;
    double t489 = -t483;
    double t490 = t254 * t475;
    double t492 = t253 * t476;
    double t497 = t12 * t495;
    double t500 = t6 * t9 * t495;
    double t501 = t9 * t11 * t495;
    double t506 = -t504;
    double t514 = t5 * t9 * t510;
    double t516 = t245 * t510;
    double t519 = t254 * t510;
    double t527 = t12 * t526;
    double t528 = t6 * t9 * t526;
    double t529 = t9 * t11 * t526;
    double t530 = t378 + t379 + t397 + t405 + t412;
    double t531 = t386 + t388 + t399 + t418 + t419;
    double t532 = t385 + t391 + t402 + t415 + t423;
    double t544 = t409 + t411 + t433 + t509 + t511 + t524;
    double t370 = -t368;
    double t371 = -t369;
    double t477 = -t473;
    double t491 = -t484;
    double t493 = -t485;
    double t520 = -t516;
    double t533 = t462 + t464 + t470;
    double t534 = t466 + t467 + t472;
    double t536 = t479 + t481 + t496;
    double t537 = t478 + t480 + t497;
    double t538 = t489 + t492 + t498;
    double t540 = t487 + t490 + t501;
    double t542 = t513 + t514 + t527;
    double t543 = t404 + t407 + t426 + t505 + t506 + t522;
    double t535 = t465 + t468 + t477;
    double t539 = t488 + t493 + t499;
    double t541 = t486 + t491 + t500;
    double t546 = t417 + t424 + t430 + t517 + t520 + t528;

    std::vector<double> mt1 = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        1.0, 0.0, 0.0,
        
        -t45 * (
            (areaVar * cDrag * f * pow(t12, 2) * t347) / 2.0 +
            (areaVar * cDrag * f * pow(t5, 2) * t31 * t347) / 2.0 +
            (areaVar * cDrag * f * pow(t10, 2) * t31 * t347) / 2.0 -
            (areaVar * cDrag * f * t12 * t260 * t348 * t428) / 4.0 +
            (areaVar * cDrag * f * t5 * t9 * t292 * t348 * t428) / 4.0 +
            (areaVar * cDrag * f * t9 * t10 * t293 * t348 * t428) / 4.0
        ),
        
        t45 * (
            t358 + t371 + t372 -
            (areaVar * cDrag * f * t244 * t293 * t348 * t428) / 4.0 +
            (areaVar * cDrag * f * t254 * t292 * t348 * t428) / 4.0 -
            (areaVar * cDrag * f * t9 * t11 * t260 * t348 * t428) / 4.0
        ),
        
        t45 * (
            t357 + t370 + t373 -
            (areaVar * cDrag * f * t245 * t292 * t348 * t428) / 4.0 +
            (areaVar * cDrag * f * t253 * t293 * t348 * t428) / 4.0 -
            (areaVar * cDrag * f * t6 * t9 * t260 * t348 * t428) / 4.0
        ),
    
        0.0
    };

    std::vector<double> mt2 = {
        0.0,
        0.0,
        
        -t182 * t278 * t534 + t183 * t278 * t535 - t219 * t278 * t533,
        -t181 * t278 * t534 + t183 * t278 * t533 - t218 * t278 * t535,
         t182 * t278 * t533 + t181 * t278 * t535 + t217 * t278 * t534,
        
        0.0,
        1.0,
        0.0,
        
        t45 * (
            t358 + t371 + t372 +
            (areaVar * cDrag * f * t12 * t260 * t348 * t434) / 4.0 -
            (areaVar * cDrag * f * t5 * t9 * t292 * t348 * t434) / 4.0 -
            (areaVar * cDrag * f * t9 * t10 * t293 * t348 * t434) / 4.0
        ),
        
        -t45 * (
            (areaVar * cDrag * f * std::pow(t244, 2) * t347) / 2.0 +
            (areaVar * cDrag * f * std::pow(t254, 2) * t347) / 2.0 +
            (areaVar * cDrag * f * std::pow(t11, 2) * t31 * t347) / 2.0 +
            (areaVar * cDrag * f * t244 * t293 * t348 * t434) / 4.0 -
            (areaVar * cDrag * f * t254 * t292 * t348 * t434) / 4.0 +
            (areaVar * cDrag * f * t9 * t11 * t260 * t348 * t434) / 4.0
        )
    };

    std::vector<double> mt3 = {
        -t45 * (
            t360 - t376 - t377 +
            (areaVar * cDrag * f * t245 * t292 * t348 * t434) / 4.0 -
            (areaVar * cDrag * f * t253 * t293 * t348 * t434) / 4.0 +
            (areaVar * cDrag * f * t6 * t9 * t260 * t348 * t434) / 4.0
        ),
        
        0.0,
        0.0,
        0.0,
        
        t182 * t278 * t538 + t183 * t278 * t539 + t219 * t278 * t536,
        t181 * t278 * t538 - t183 * t278 * t536 - t218 * t278 * t539,
       -t182 * t278 * t536 + t181 * t278 * t539 - t217 * t278 * t538,
        
        0.0,
        0.0,
        1.0,
        
        t45 * (
            t357 + t370 + t373 +
            (areaVar * cDrag * f * t12 * t260 * t348 * t435) / 4.0 -
            (areaVar * cDrag * f * t5 * t9 * t292 * t348 * t435) / 4.0 -
            (areaVar * cDrag * f * t9 * t10 * t293 * t348 * t435) / 4.0
        ),
        
        -t45 * (
            t360 - t376 - t377 +
            (areaVar * cDrag * f * t244 * t293 * t348 * t435) / 4.0 -
            (areaVar * cDrag * f * t254 * t292 * t348 * t435) / 4.0 +
            (areaVar * cDrag * f * t9 * t11 * t260 * t348 * t435) / 4.0
        )
    };

    std::vector<double> mt4 = {
        -t45 * (
            (areaVar * cDrag * f * std::pow(t245, 2) * t347) / 2.0 +
            (areaVar * cDrag * f * std::pow(t253, 2) * t347) / 2.0 +
            (areaVar * cDrag * f * std::pow(t6, 2) * t31 * t347) / 2.0 +
            (areaVar * cDrag * f * t245 * t292 * t348 * t435) / 4.0 -
            (areaVar * cDrag * f * t253 * t293 * t348 * t435) / 4.0 +
            (areaVar * cDrag * f * t6 * t9 * t260 * t348 * t435) / 4.0
        ),
    
        0.0,
        0.0,
        0.0,
    
        -t182 * t278 * t541 - t183 * t278 * t540 - t219 * t278 * t537,
         t183 * t278 * t537 - t181 * t278 * t541 + t218 * t278 * t540,
         t182 * t278 * t537 - t181 * t278 * t540 + t217 * t278 * t541,
    
        0.0,
        0.0,
        0.0
    };

    std::vector<double> mt5 = {
        t45 * (
            (areaVar * cDrag * f * t12 * t252 * t347) / 2.0 -
            (areaVar * cDrag * f * t5 * t9 * t283 * t347) / 2.0 +
            (areaVar * cDrag * f * t9 * t10 * t284 * t347) / 2.0 +
            (areaVar * cDrag * f * t12 * t260 * t348 * t440) / 4.0 -
            (areaVar * cDrag * f * t5 * t9 * t292 * t348 * t440) / 4.0 -
            (areaVar * cDrag * f * t9 * t10 * t293 * t348 * t440) / 4.0
        ),
    
        -t45 * (
            T * t8 * t253 -
            T * t3 * t7 * t245 +
            T * t2 * t3 * t6 * t9 -
            (areaVar * cDrag * f * t244 * t284 * t347) / 2.0 +
            (areaVar * cDrag * f * t245 * t292 * t347) / 2.0 -
            (areaVar * cDrag * f * t254 * t283 * t347) / 2.0 -
            (areaVar * cDrag * f * t253 * t293 * t347) / 2.0 +
            (areaVar * cDrag * f * t9 * t11 * t252 * t347) / 2.0 +
            (areaVar * cDrag * f * t6 * t9 * t260 * t347) / 2.0 +
            (areaVar * cDrag * f * t244 * t293 * t348 * t440) / 4.0 -
            (areaVar * cDrag * f * t254 * t292 * t348 * t440) / 4.0 +
            (areaVar * cDrag * f * t9 * t11 * t260 * t348 * t440) / 4.0
        )
    };

    std::vector<double> mt6 = {
        -t45 * (
            T * t8 * t244 -
            T * t3 * t7 * t254 -
            T * t2 * t3 * t9 * t11 +
            (areaVar * cDrag * f * t245 * t283 * t347) / 2.0 -
            (areaVar * cDrag * f * t244 * t293 * t347) / 2.0 +
            (areaVar * cDrag * f * t253 * t284 * t347) / 2.0 +
            (areaVar * cDrag * f * t254 * t292 * t347) / 2.0 +
            (areaVar * cDrag * f * t6 * t9 * t252 * t347) / 2.0 -
            (areaVar * cDrag * f * t9 * t11 * t260 * t347) / 2.0 +
            (areaVar * cDrag * f * t245 * t292 * t348 * t440) / 4.0 -
            (areaVar * cDrag * f * t253 * t293 * t348 * t440) / 4.0 +
            (areaVar * cDrag * f * t6 * t9 * t260 * t348 * t440) / 4.0
        ),
    
        0.0,
        0.0,
        0.0,
    
        t183 * t278 * (t416 - t422 + t427 + t515 - t519 - t529) -
        t182 * t278 * t546 -
        t219 * t278 * t542,
    
        -t218 * t278 * (t416 - t422 + t427 + t515 - t519 - t529) +
        t183 * t278 * t542 -
        t181 * t278 * t546
    };

    std::vector<double> mt7 = {
        t181 * t278 * (t416 - t422 + t427 + t515 - t519 - t529) +
        t182 * t278 * t542 +
        t217 * t278 * t546,
    
        0.0,
        0.0,
        0.0,
    
        -t45 * (
            -T * t2 * t3 * t9 +
             T * t8 * t10 * t12 +
             T * t3 * t5 * t7 * t12 +
            (areaVar * cDrag * f * t12 * t255 * t347) / 2.0 -
            (areaVar * cDrag * f * t9 * t260 * t347) / 2.0 +
            (areaVar * cDrag * f * t5 * t9 * t275 * t347) / 2.0 +
            (areaVar * cDrag * f * t9 * t10 * t276 * t347) / 2.0 -
            (areaVar * cDrag * f * t5 * t12 * t292 * t347) / 2.0 -
            (areaVar * cDrag * f * t10 * t12 * t293 * t347) / 2.0 -
            (areaVar * cDrag * f * t12 * t260 * t348 * t437) / 4.0 +
            (areaVar * cDrag * f * t5 * t9 * t292 * t348 * t437) / 4.0 +
            (areaVar * cDrag * f * t9 * t10 * t293 * t348 * t437) / 4.0
        )
    };

    std::vector<double> mt8 = {
        t45 * (
            T * t8 * t9 * t44 +
            T * t2 * t3 * t11 * t12 +
            T * t3 * t7 * t9 * t40 -
            (areaVar * cDrag * f * t244 * t276 * t347) / 2.0 +
            (areaVar * cDrag * f * t254 * t275 * t347) / 2.0 +
            (areaVar * cDrag * f * t9 * t11 * t255 * t347) / 2.0 +
            (areaVar * cDrag * f * t11 * t12 * t260 * t347) / 2.0 -
            (areaVar * cDrag * f * t9 * t40 * t292 * t347) / 2.0 -
            (areaVar * cDrag * f * t9 * t44 * t293 * t347) / 2.0 -
            (areaVar * cDrag * f * t244 * t293 * t348 * t437) / 4.0 +
            (areaVar * cDrag * f * t254 * t292 * t348 * t437) / 4.0 -
            (areaVar * cDrag * f * t9 * t11 * t260 * t348 * t437) / 4.0
        )
    };

    std::vector<double> mt9 = {
        t45 * (
            T * t8 * t9 * t41 +
            T * t2 * t3 * t6 * t12 +
            T * t3 * t7 * t9 * t39 -
            (areaVar * cDrag * f * t245 * t275 * t347) / 2.0 +
            (areaVar * cDrag * f * t253 * t276 * t347) / 2.0 +
            (areaVar * cDrag * f * t6 * t9 * t255 * t347) / 2.0 +
            (areaVar * cDrag * f * t6 * t12 * t260 * t347) / 2.0 -
            (areaVar * cDrag * f * t9 * t39 * t292 * t347) / 2.0 -
            (areaVar * cDrag * f * t9 * t41 * t293 * t347) / 2.0 -
            (areaVar * cDrag * f * t245 * t292 * t348 * t437) / 4.0 +
            (areaVar * cDrag * f * t253 * t293 * t348 * t437) / 4.0 -
            (areaVar * cDrag * f * t6 * t9 * t260 * t348 * t437) / 4.0
        ),
    
        0.0, 0.0, 0.0,
    
        t182 * t278 * (t408 + t410 - t429 - t507 - t512 + t523) +
        t183 * t278 * t544 +
        t219 * t278 * t543,
    
        t181 * t278 * (t408 + t410 - t429 - t507 - t512 + t523) -
        t183 * t278 * t543 -
        t218 * t278 * t544
    };

    std::vector<double> mt10 = {
        -t217 * t278 * (t408 + t410 - t429 - t507 - t512 + t523) +
         t181 * t278 * t544 -
         t182 * t278 * t543,
    
        0.0, 0.0, 0.0,
    
        t45 * (
            T * t5 * t8 * t9 -
            T * t3 * t7 * t9 * t10 -
            (areaVar * cDrag * f * t12 * t260 * t348 * t436) / 4.0 +
            (areaVar * cDrag * f * t5 * t9 * t292 * t348 * t436) / 4.0 +
            (areaVar * cDrag * f * t9 * t10 * t293 * t348 * t436) / 4.0
        ),
    
        -t45 * (
            T * t8 * t254 +
            T * t3 * t7 * t244 -
            (areaVar * cDrag * f * t244 * t293 * t348 * t436) / 4.0 +
            (areaVar * cDrag * f * t254 * t292 * t348 * t436) / 4.0 -
            (areaVar * cDrag * f * t9 * t11 * t260 * t348 * t436) / 4.0
        ),
    
        t45 * (
            T * t8 * t245 +
            T * t3 * t7 * t253 +
            (areaVar * cDrag * f * t245 * t292 * t348 * t436) / 4.0 -
            (areaVar * cDrag * f * t253 * t293 * t348 * t436) / 4.0 +
            (areaVar * cDrag * f * t6 * t9 * t260 * t348 * t436) / 4.0
        ),
    
        0.0, 0.0, 0.0
    };

    std::vector<double> mt11 = {
        t182 * t278 * t531 - t183 * t278 * t532 + t219 * t278 * t530,
        t181 * t278 * t531 - t183 * t278 * t530 + t218 * t278 * t532,
        -t182 * t278 * t530 - t181 * t278 * t532 - t217 * t278 * t531,
        0.0,0.0,0.0,
        0.0,0.0,0.0,
        1.0,0.0,0.0,
        0.0,0.0,0.0,
        0.0,0.0,0.0,
        0.0,0.0,0.0,
        0.0,1.0,0.0,
        0.0,0.0,0.0,
        0.0,0.0,0.0,
        0.0,0.0,0.0,
        0.0,0.0,1.0,
        0.0,0.0,0.0
    };
    
    std::vector<double> A_flat;
    A_flat.reserve(144);  // Reserve memory for performance

    A_flat.insert(A_flat.end(), mt1.begin(), mt1.end());
    A_flat.insert(A_flat.end(), mt2.begin(), mt2.end());
    A_flat.insert(A_flat.end(), mt3.begin(), mt3.end());
    A_flat.insert(A_flat.end(), mt4.begin(), mt4.end());
    A_flat.insert(A_flat.end(), mt5.begin(), mt5.end());
    A_flat.insert(A_flat.end(), mt6.begin(), mt6.end());
    A_flat.insert(A_flat.end(), mt7.begin(), mt7.end());
    A_flat.insert(A_flat.end(), mt8.begin(), mt8.end());
    A_flat.insert(A_flat.end(), mt9.begin(), mt9.end());
    A_flat.insert(A_flat.end(), mt10.begin(), mt10.end());
    A_flat.insert(A_flat.end(), mt11.begin(), mt11.end());

    Matrix A(12, 12, 0);

    for (unsigned int i = 0; i < 12; ++i) {
        for (unsigned int j = 0; j < 12; ++j) {
            A(i, j) = A_flat[j * 12 + i]; 
        }
    }

    return A;

}