#include "../CustomLinear/Matrix.h"
#include "../CustomLinear/Vector.h"
#include <vector>

inline double sign(double x) {
    return (x > 0) - (x < 0);
}

/**
 * @brief Computes the continuous-time A matrix of the nonlinear system linearized around a state/input in the body frame.
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
Matrix calculateABF(
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

    double t2 = m; 
    double t3 = cos(phiVar);
    double t4 = cos(psiVar);
    double t5 = cos(thetaVar);
    double t6 = sin(phiVar);
    double t7 = sin(psiVar);
    double t8 = sin(thetaVar);
    double t9 = pow(Ixy, 2);
    double t10 = pow(Ixz, 2);
    double t11 = pow(Iyz, 2);
    double t12 = Ixy * Ixz;
    double t13 = Ixx * Iyy;
    double t14 = Ixx * Iyz;
    double t15 = Ixy * Iyz;
    double t16 = Ixz * Iyy;
    double t17 = Ixz * Iyz;
    double t18 = Ixx * Izz;
    double t19 = Ixy * Izz;
    double t20 = Iyy * Izz;
    double t21 = Izz * t13;
    double t22 = t5 * v_x;
    double t23 = rc_x * t8;
    double t24 = rc_y * t8;
    double t25 = t8 * v_x;
    double t26 = t3 * t4;
    double t27 = t3 * t7;
    double t28 = t4 * t6;
    double t31 = t6 * t7;
    double t32 = 1.0 / t2;
    double t35 = Iyz * t12 * 2.0;
    double t36 = Ixx * t11;
    double t37 = Iyy * t10;
    double t38 = Izz * t9;
    double t39 = -t13;
    double t40 = -t14;
    double t41 = -t16;
    double t42 = -t18;
    double t43 = -t19;
    double t44 = -t20;
    double t49 = rc_y * t3 * t5;
    double t50 = rc_z * t3 * t5;
    double t53 = rc_x * t4 * t5;
    double t54 = rc_y * t4 * t5;
    double t56 = t4 * t5 * v_y;
    double t57 = t4 * t5 * v_z;
    double t62 = rc_x * t5 * t6;
    double t63 = rc_z * t5 * t6;
    double t68 = rc_x * t5 * t7;
    double t69 = rc_y * t5 * t7;
    double t72 = t5 * t7 * v_y;
    double t73 = t4 * t8 * v_z;
    double t74 = t5 * t7 * v_z;
    double t80 = t7 * t8 * v_y;
    double t29 = rc_x * t22;
    double t30 = rc_y * t22;
    double t33 = t23 * v_x;
    double t34 = t24 * v_x;
    double t45 = -t35;
    double t46 = -t21;
    double t47 = rc_x * t26;
    double t48 = rc_z * t26;
    double t51 = t26 * v_y;
    double t52 = t26 * v_z;
    double t55 = t3 * t22;
    double t58 = rc_x * t27;
    double t59 = rc_y * t28;
    double t60 = rc_z * t27;
    double t61 = rc_z * t28;
    double t64 = t27 * v_y;
    double t65 = t28 * v_y;
    double t66 = t27 * v_z;
    double t67 = t28 * v_z;
    double t70 = t3 * t25;
    double t71 = t6 * t22;
    double t75 = rc_y * t31;
    double t76 = rc_z * t31;
    double t77 = t31 * v_y;
    double t78 = t31 * v_z;
    double t79 = t6 * t25;
    double t81 = -t25;
    double t87 = t68 * v_y;
    double t88 = t69 * v_y;
    double t89 = t4 * t23 * v_z;
    double t90 = t68 * v_z;
    double t91 = t4 * t24 * v_z;
    double t92 = t69 * v_z;
    double t93 = t8 * t31;    
    double t101 = t7 * t23 * v_y;
    double t102 = t7 * t24 * v_y;
    double t103 = t8 * t26;
    double t112 = t53 * v_y;
    double t113 = t54 * v_y;
    double t114 = t53 * v_z;
    double t115 = t54 * v_z;
    double t116 = t8 * t27;
    double t117 = t8 * t28;
    double t129 = t24 * t26;
    double t135 = t23 * t28;
    double t136 = t24 * t27;
    double t144 = -t53;
    double t145 = t23 * t31;
    double t149 = -t62;
    double t152 = -t69;
    double t154 = -t74;
    double t163 = t12 + t40;
    double t164 = t15 + t41;
    double t165 = t17 + t43;
    double t198 = t9 + t39;
    double t199 = t10 + t42;
    double t200 = t11 + t44;
    double t214 = t23 + t50;
    double t215 = t24 + t63;
    double t234 = t22 + t73 + t80;
    double t82 = t6 * t29;
    double t83 = t3 * t34;
    double t84 = t6 * t30;
    double t85 = rc_z * t70;
    double t86 = rc_z * t71;
    double t94 = t75 * v_y;
    double t95 = t76 * v_y;
    double t96 = rc_x * t78;
    double t97 = t75 * v_z;
    double t98 = t76 * v_z;
    double t99 = t6 * t33;
    double t100 = rc_z * t79;
    double t104 = t47 * v_y;
    double t105 = rc_y * t51;
    double t106 = t48 * v_y;
    double t107 = t47 * v_z;
    double t108 = t48 * v_z;
    double t109 = t3 * t29;
    double t110 = t3 * t30;
    double t111 = rc_z * t55;
    double t118 = t58 * v_y;
    double t119 = rc_x * t65;
    double t120 = t59 * v_y;
    double t121 = t60 * v_y;
    double t122 = t61 * v_y;
    double t123 = t58 * v_z;
    double t124 = rc_y * t66;
    double t125 = t59 * v_z;
    double t126 = t60 * v_z;
    double t127 = t61 * v_z;
    double t128 = t5 * t52;
    double t130 = t8 * t48;
    double t131 = t8 * t51;
    double t132 = t5 * t64;
    double t133 = t8 * t52;
    double t134 = t5 * t67;
    double t137 = t8 * t60;
    double t138 = t8 * t61;
    double t139 = t8 * t64;
    double t140 = t8 * t65;
    double t141 = t5 * t77;
    double t142 = t8 * t66;
    double t143 = t8 * t67;
    double t146 = t8 * t76;
    double t147 = t8 * t77;
    double t148 = t8 * t78;
    double t150 = -t65;
    double t151 = -t66;
    double t153 = -t70;
    double t155 = -t79;
    double t157 = -t87;
    double t158 = -t88;
    double t159 = -t92;
    double t168 = t24 * t51;
    double t172 = t23 * t52;
    double t174 = t24 * t52;
    double t177 = t23 * t64;
    double t180 = t24 * t64;
    double t184 = t23 * t67;
    double t186 = t24 * t67;
    double t190 = -t112;
    double t191 = -t114;
    double t192 = -t115;
    double t193 = t23 * t77;
    double t194 = t24 * t77;
    double t196 = t23 * t78;
    double t201 = -t116;
    double t202 = -t117;
    double t208 = -t135;
    double t209 = -t136;
    double t224 = t26 + t93;
    double t225 = t31 + t103;
    double t230 = t49 + t149;
    double t231 = t56 + t154;
    double t239 = t57 + t72 + t81;
    double t256 = t36 + t37 + t38 + t45 + t46;
    double t156 = -t85;
    double t160 = -t97;
    double t161 = -t99;
    double t162 = -t100;
    double t166 = rc_y * t128;
    double t167 = t5 * t108;
    double t169 = rc_y * t132;
    double t170 = t8 * t106;
    double t171 = t5 * t121;
    double t173 = rc_x * t134;
    double t175 = t8 * t108;
    double t176 = t5 * t127;
    double t179 = rc_x * t141;
    double t181 = t8 * t121;
    double t182 = t8 * t122;
    double t183 = t5 * t95;
    double t187 = t8 * t126;
    double t188 = t8 * t127;
    double t189 = -t110;
    double t195 = t8 * t95;
    double t197 = t8 * t98;
    double t203 = -t119;
    double t204 = -t122;
    double t205 = -t123;
    double t206 = -t124;
    double t207 = -t126;
    double t210 = -t137;
    double t211 = -t138;
    double t212 = -t140;
    double t213 = -t142;
    double t218 = -t174;
    double t219 = t23 * t150;
    double t220 = -t180;
    double t222 = t24 * t151;
    double t226 = t224 * v_y;
    double t227 = t225 * v_y;
    double t228 = t224 * v_z;
    double t229 = t225 * v_z;
    double t232 = t27 + t202;
    double t233 = t28 + t201;
    double t242 = abs(t239);
    double t243 = sign(t239);
    double t244 = pow(t239, 2);
    double t246 = t8 * t239 * 2.0;
    double t247 = t76 + t130 + t144;
    double t250 = t48 + t146 + t152;
    double t252 = t4 * t5 * t239 * 2.0;
    double t253 = t5 * t7 * t239 * 2.0;
    double t254 = t128 + t132 + t153;
    double t255 = t134 + t141 + t155;
    double t257 = 1.0 / t256;
    double t258 = t47 + t59 + t145 + t209;
    double t259 = t58 + t75 + t129 + t208;
    double t264 = t231 * t239 * 2.0;
    double t265 = t55 + t78 + t133 + t139 + t150;
    double t266 = t51 + t71 + t143 + t147 + t151;
    double t267 = t234 * t239 * 2.0;
    double t216 = -t166;
    double t217 = -t169;
    double t221 = -t182;
    double t223 = -t187;
    double t235 = t232 * v_y;
    double t236 = t233 * v_y;
    double t237 = t232 * v_z;
    double t238 = t233 * v_z;
    double t245 = pow(t242, 2);
    double t248 = t54 + t60 + t211;
    double t249 = t61 + t68 + t210;
    double t251 = -t246;
    double t260 = t52 + t64 + t148 + t212;
    double t261 = t67 + t77 + t131 + t213;
    double t268 = pow(t265, 2);
    double t269 = pow(t266, 2);
    double t270 = -t267;
    double t273 = t98 + t111 + t175 + t181 + t204;
    double t274 = t86 + t106 + t188 + t195 + t207;
    double t279 = t5 * t6 * t266 * 2.0;
    double t280 = t3 * t5 * t265 * 2.0;
    double t283 = t8 * t242 * t243 * 2.0;
    double t285 = t4 * t5 * t242 * t243 * 2.0;
    double t286 = t5 * t7 * t242 * t243 * 2.0;
    double t287 = t29 + t89 + t101 + t156 + t167 + t171;
    double t288 = t30 + t91 + t102 + t162 + t176 + t183;
    double t291 = t225 * t265 * 2.0;
    double t292 = t224 * t266 * 2.0;
    double t293 = t233 * t265 * 2.0;
    double t294 = t232 * t266 * 2.0;
    double t297 = t231 * t242 * t243 * 2.0;
    double t299 = t234 * t242 * t243 * 2.0;
    double t301 = t255 * t266 * 2.0;
    double t302 = t254 * t265 * 2.0;
    double t305 = t94 + t107 + t118 + t125 + t168 + t196 + t219 + t222;
    double t309 = t84 + t96 + t105 + t109 + t172 + t177 + t186 + t194 + t203 + t206;
    double t310 = t82 + t104 + t120 + t160 + t184 + t189 + t193 + t205 + t218 + t220;
    double t240 = -t236;
    double t241 = -t237;
    double t262 = t227 + t238;
    double t263 = t228 + t235;
    double t284 = -t283;
    double t289 = t90 + t95 + t127 + t170 + t190 + t223;
    double t290 = t108 + t113 + t121 + t159 + t197 + t221;
    double t295 = -t293;
    double t296 = -t294;
    double t298 = t83 + t161 + t173 + t179 + t216 + t217;
    double t300 = -t299;
    double t303 = t33 + t157 + t191 + t273;
    double t304 = t34 + t158 + t192 + t274;
    double t306 = t261 * t265 * 2.0;
    double t307 = t260 * t266 * 2.0;
    double t315 = t244 + t268 + t269;
    double t325 = t251 + t279 + t280;
    double t350 = t270 + t301 + t302;
    double t271 = t55 + t229 + t240;
    double t272 = t71 + t226 + t241;
    double t308 = -t307;
    double t320 = sqrt(t315);
    double t337 = t252 + t291 + t296;
    double t338 = t253 + t292 + t295;
    double t275 = abs(t271);
    double t276 = abs(t272);
    double t277 = sign(t271);
    double t278 = sign(t272);
    double t321 = 1.0 / t320;
    double t331 = (areaVar * cDrag * f * t214 * t320) / 2.0;
    double t332 = (areaVar * cDrag * f * t215 * t320) / 2.0;
    double t333 = (areaVar * cDrag * f * t230 * t320) / 2.0;
    double t339 = (areaVar * cDrag * f * t247 * t320) / 2.0;
    double t340 = (areaVar * cDrag * f * t248 * t320) / 2.0;
    double t341 = (areaVar * cDrag * f * t249 * t320) / 2.0;
    double t342 = (areaVar * cDrag * f * t250 * t320) / 2.0;
    double t343 = (areaVar * cDrag * f * t258 * t320) / 2.0;
    double t344 = (areaVar * cDrag * f * t259 * t320) / 2.0;
    double t345 = (areaVar * cDrag * f * t287 * t320) / 2.0;
    double t346 = (areaVar * cDrag * f * t288 * t320) / 2.0;
    double t347 = (areaVar * cDrag * f * t289 * t320) / 2.0;
    double t348 = (areaVar * cDrag * f * t290 * t320) / 2.0;
    double t349 = (areaVar * cDrag * f * t298 * t320) / 2.0;
    double t351 = t264 + t306 + t308;
    double t352 = (areaVar * cDrag * f * t305 * t320) / 2.0;
    double t281 = pow(t275, 2);
    double t282 = pow(t276, 2);
    double t311 = t3 * t5 * t275 * t277 * 2.0;
    double t312 = t5 * t6 * t276 * t278 * 2.0;
    double t313 = t225 * t275 * t277 * 2.0;
    double t314 = t224 * t276 * t278 * 2.0;
    double t316 = t233 * t275 * t277 * 2.0;
    double t317 = t232 * t276 * t278 * 2.0;
    double t322 = t255 * t276 * t278 * 2.0;
    double t323 = t254 * t275 * t277 * 2.0;
    double t328 = t262 * t275 * t277 * 2.0;
    double t329 = t263 * t276 * t278 * 2.0;
    double t334 = t272 * t275 * t277 * 2.0;
    double t335 = t271 * t276 * t278 * 2.0;
    double t358 = (areaVar * cDrag * f * t303 * t321 * t325) / 4.0;
    double t359 = (areaVar * cDrag * f * t304 * t321 * t325) / 4.0;
    double t361 = (areaVar * cDrag * f * t304 * t321 * t338) / 4.0;
    double t362 = (areaVar * cDrag * f * t303 * t321 * t337) / 4.0;
    double t363 = (areaVar * cDrag * f * t303 * t321 * t338) / 4.0;
    double t364 = (areaVar * cDrag * f * t304 * t321 * t337) / 4.0;
    double t367 = (areaVar * cDrag * f * t310 * t321 * t325) / 4.0;
    double t369 = (areaVar * cDrag * f * t310 * t321 * t337) / 4.0;
    double t370 = (areaVar * cDrag * f * t310 * t321 * t338) / 4.0;
    double t372 = (areaVar * cDrag * f * t303 * t321 * t350) / 4.0;
    double t373 = (areaVar * cDrag * f * t304 * t321 * t350) / 4.0;
    double t374 = (areaVar * cDrag * f * t303 * t321 * t351) / 4.0;
    double t375 = (areaVar * cDrag * f * t304 * t321 * t351) / 4.0;
    double t377 = (areaVar * cDrag * f * t310 * t321 * t350) / 4.0;
    double t378 = (areaVar * cDrag * f * t310 * t321 * t351) / 4.0;
    double t318 = -t316;
    double t319 = -t317;
    double t324 = t245 + t281 + t282;
    double t330 = -t329;
    double t336 = -t335;
    double t353 = t284 + t311 + t312;
    double t357 = t300 + t322 + t323;
    double t365 = -t363;
    double t366 = -t364;
    double t368 = -t367;
    double t371 = -t369;
    double t376 = -t375;
    double t379 = -t378;
    double t380 = t331 + t358;
    double t381 = t332 + t359;
    double t383 = t339 + t362;
    double t384 = t342 + t361;
    double t387 = t343 + t370;
    double t389 = t345 + t372;
    double t390 = t346 + t373;
    double t391 = t347 + t374;
    double t393 = t349 + t377;
    double t326 = sqrt(t324);
    double t354 = t286 + t314 + t318;
    double t355 = t285 + t313 + t319;
    double t356 = t334 + t336;
    double t360 = t297 + t328 + t330;
    double t382 = t333 + t368;
    double t385 = t340 + t366;
    double t386 = t341 + t365;
    double t388 = t344 + t371;
    double t392 = t348 + t376;
    double t394 = t352 + t379;
    double t327 = 1.0 / t326;

    std::vector<double> mt1 = {
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 
        
        -t32 * (
            (areaVar * cDrag * f * t3 * t5 * t326) / 2.0 + 
            (areaVar * cDrag * f * t271 * t327 * t353) / 4.0
        ), 
        
        -t32 * (
            (areaVar * cDrag * f * t5 * t6 * t326) / 2.0 + 
            (areaVar * cDrag * f * t272 * t327 * t353) / 4.0
        ), 
        
        t32 * (
            (areaVar * cDrag * f * t8 * t326) / 2.0 - 
            (areaVar * cDrag * f * t239 * t327 * t353) / 4.0
        ), 
        
        0.0, 0.0, 0.0,        
        t165 * t257 * t380 - t164 * t257 * t382 + t200 * t257 * t381, 
        -t163 * t257 * t382 - t165 * t257 * t381 - t199 * t257 * t380,     
        t163 * t257 * t380 - t164 * t257 * t381 + t198 * t257 * t382,      
        0.0, 1.0, 0.0, 
        
        t32 * (
            (areaVar * cDrag * f * t233 * t326) / 2.0 - 
            (areaVar * cDrag * f * t271 * t327 * t354) / 4.0
        )
    };
    
    std::vector<double> mt2 = {
        -t32 * (
            (areaVar * cDrag * f * t224 * t326) / 2.0 + 
            (areaVar * cDrag * f * t272 * t327 * t354) / 4.0
        ),

        -t32 * (
            (areaVar * cDrag * f * t5 * t7 * t326) / 2.0 + 
            (areaVar * cDrag * f * t239 * t327 * t354) / 4.0
        ),

        0.0, 0.0, 0.0,
        t164 * t257 * t387 - t165 * t257 * t386 + t200 * t257 * t384,
        -t165 * t257 * t384 + t163 * t257 * t387 + t199 * t257 * t386,
        -t164 * t257 * t384 - t163 * t257 * t386 - t198 * t257 * t387,
        0.0, 0.0, 1.0,

        -t32 * (
            (areaVar * cDrag * f * t225 * t326) / 2.0 + 
            (areaVar * cDrag * f * t271 * t327 * t355) / 4.0
        ),
        
        t32 * (
            (areaVar * cDrag * f * t232 * t326) / 2.0 - 
            (areaVar * cDrag * f * t272 * t327 * t355) / 4.0
        ),

        -t32 * (
            (areaVar * cDrag * f * t4 * t5 * t326) / 2.0 + 
            (areaVar * cDrag * f * t239 * t327 * t355) / 4.0
        ),

        0.0, 0.0, 0.0,
        t165 * t257 * t383 - t164 * t257 * t388 - t200 * t257 * t385
    };
    
    std::vector<double> mt3 = {
        t165 * t257 * t385 - t163 * t257 * t388 - t199 * t257 * t383,
        t163 * t257 * t383 + t164 * t257 * t385 + t198 * t257 * t388,
        0.0, 0.0, 0.0,
        
        -t32 * (
            (areaVar * cDrag * f * t262 * t326) / 2.0 + 
            (areaVar * cDrag * f * t271 * t327 * t360) / 4.0
        ),

        t32 * (
            (areaVar * cDrag * f * t263 * t326) / 2.0 - 
            (areaVar * cDrag * f * t272 * t327 * t360) / 4.0
        ),

        -t32 * (
            (areaVar * cDrag * f * t231 * t326) / 2.0 + 
            (areaVar * cDrag * f * t239 * t327 * t360) / 4.0
        ),

        0.0, 0.0, 0.0,
        t165 * t257 * t391 - t164 * t257 * t394 - t200 * t257 * t392,
        -t163 * t257 * t394 + t165 * t257 * t392 - t199 * t257 * t391,
        t163 * t257 * t391 + t164 * t257 * t392 + t198 * t257 * t394,
        0.0, 0.0, 0.0,
        
        -t32 * (
            (areaVar * cDrag * f * t254 * t326) / 2.0 + 
            (areaVar * cDrag * f * t271 * t327 * t357) / 4.0
        )
    };
    
    std::vector<double> mt3 = {
        t165 * t257 * t385 - t163 * t257 * t388 - t199 * t257 * t383,
        t163 * t257 * t383 + t164 * t257 * t385 + t198 * t257 * t388,
        0.0, 0.0, 0.0,
        
        -t32 * (
            (areaVar * cDrag * f * t262 * t326) / 2.0 + 
            (areaVar * cDrag * f * t271 * t327 * t360) / 4.0
        ),

        t32 * (
            (areaVar * cDrag * f * t263 * t326) / 2.0 - 
            (areaVar * cDrag * f * t272 * t327 * t360) / 4.0
        ),

        -t32 * (
            (areaVar * cDrag * f * t231 * t326) / 2.0 + 
            (areaVar * cDrag * f * t239 * t327 * t360) / 4.0
        ),

        0.0, 0.0, 0.0,
        t165 * t257 * t391 - t164 * t257 * t394 - t200 * t257 * t392,
        -t163 * t257 * t394 + t165 * t257 * t392 - t199 * t257 * t391,
        t163 * t257 * t391 + t164 * t257 * t392 + t198 * t257 * t394,
        0.0, 0.0, 0.0,
        
        -t32 * (
            (areaVar * cDrag * f * t254 * t326) / 2.0 + 
            (areaVar * cDrag * f * t271 * t327 * t357) / 4.0
        )
    };
    
    std::vector<double> mt4 = {
        -t32 * (
            (areaVar * cDrag * f * t255 * t326) / 2.0 + 
            (areaVar * cDrag * f * t272 * t327 * t357) / 4.0
        ),

        t32 * (
            (areaVar * cDrag * f * t234 * t326) / 2.0 - 
            (areaVar * cDrag * f * t239 * t327 * t357) / 4.0
        ),

        0.0, 0.0, 0.0,
        t165 * t257 * t389 + t164 * t257 * t393 + t200 * t257 * t390,
        -t165 * t257 * t390 + t163 * t257 * t393 - t199 * t257 * t389,
        t163 * t257 * t389 - t164 * t257 * t390 - t198 * t257 * t393,
        0.0, 0.0, 0.0,

        t32 * (
            (areaVar * cDrag * f * t272 * t326) / 2.0 + 
            (areaVar * cDrag * f * t271 * t327 * t356) / 4.0
        ),

        -t32 * (
            (areaVar * cDrag * f * t271 * t326) / 2.0 - 
            (areaVar * cDrag * f * t272 * t327 * t356) / 4.0
        ),

        (areaVar * cDrag * f * t32 * t239 * t327 * t356) / 4.0,
        0.0, 0.0, 0.0,

        areaVar * cDrag * f * t165 * t257 * t274 * t320 * (-1.0 / 2.0) +
        (areaVar * cDrag * f * t164 * t257 * t309 * t320) / 2.0 +
        (areaVar * cDrag * f * t200 * t257 * t273 * t320) / 2.0
    };
    
    std::vector<double> mt5 = {
        areaVar * cDrag * f * t165 * t257 * t273 * t320 * (-1.0 / 2.0) +
        (areaVar * cDrag * f * t163 * t257 * t309 * t320) / 2.0 +
        (areaVar * cDrag * f * t199 * t257 * t274 * t320) / 2.0,

        areaVar * cDrag * f * t163 * t257 * t274 * t320 * (-1.0 / 2.0) -
        (areaVar * cDrag * f * t164 * t257 * t273 * t320) / 2.0 -
        (areaVar * cDrag * f * t198 * t257 * t309 * t320) / 2.0,

        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
        0.0, 0.0, 0.0
    };

    /* ------------ Final matrix based off previous calculateA ------------ */
   
    std::vector<double> A_flat;
    A_flat.reserve(144);  // Reserve memory for performance

    A_flat.insert(A_flat.end(), mt1.begin(), mt1.end());
    A_flat.insert(A_flat.end(), mt2.begin(), mt2.end());
    A_flat.insert(A_flat.end(), mt3.begin(), mt3.end());
    A_flat.insert(A_flat.end(), mt4.begin(), mt4.end());
    A_flat.insert(A_flat.end(), mt5.begin(), mt5.end());

    Matrix A(12, 12, 0);

    for (unsigned int i = 0; i < 12; ++i) {
        for (unsigned int j = 0; j < 12; ++j) {
            A(i, j) = A_flat[j * 12 + i]; 
        }
    }

    return A;
}