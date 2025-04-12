function [K,S,P] = calculateLQRNum(symA, symB, Q, R, t, m_num, state, input, rc_num, rt_num, f_num, cDrag_num, areaVar_num, inertia_num, inertia_s_num, inertia_a_num, inertia_b_num)
    A = subs(symA, [m f cDrag areaVar], [m_num f_num cDrag_num areaVar_num]);
    A = subs(symA, full_input, input);
    A = subs(symA, full_state, state);
    A = subs(symA, rc, rc_num);
    A = subs(symA, rt, rt_num);
    A = subs(symA, [inertia inertia_s inertia_a inertia_b], [inertia_num inertia_s_num inertia_a_num inertia_b_num]);

    B = subs(symB, [m f cDrag areaVar], [m_num f_num cDrag_num areaVar_num]);
    B = subs(symB, full_input, input);
    B = subs(symB, full_state, state);
    B = subs(symB, rc, rc_num);
    B = subs(symB, rt, rt_num);
    B = subs(symB, [inertia inertia_s inertia_a inertia_b], [inertia_num inertia_s_num inertia_a_num inertia_b_num]);

    A = double(A);
    B = double(B);

    sys_c = ss(A, B, C, D);
    sys_d = c2d(sys_c, t, 'zoh');
    A_d = sys_d.A;
    B_d = sys_d.B;

    % debugging - displays the rank of the controllability matrix
    % must be the same as the number of states for system to be
    % fully controllable.
    Co = ctrb(A_d, B_d);
    rank_Co = rank(Co);
    disp(rank_Co);

    [K, S, P] = dlqr(A_d, B_d, Q, R);
end