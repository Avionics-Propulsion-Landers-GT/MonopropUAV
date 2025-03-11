state = [0 0 0, 0 3 10, 0 1 0, 0 0 0]';
input = [14.0000 0.1000 0.1000 0 0 0 0]';
f_num = 1.23;
cDrag_num = 0.4;
areaVar_num = 1;
inerta_num = eye(3);
inertia_s_num = 0.1 * eye(3);
inertia_a_num = 0.01 * eye(3);
inertia_b_num = 0.01 * eye(3);
t = 0.005;
Q = eye(12);
R = eye(7);

[K, S, P] = calculateLQRNum(Q, R, t, m_num, state, input, rc_num, rt_num, f_num, cd_num, area_num, inertia_num, inerta_s_num, inertia_a_num, inertia_b_num);