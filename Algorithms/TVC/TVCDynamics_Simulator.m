% Modeling the rotation between 3D orientation axes of the TVC.

F_init = [0,0,1]; % Static neutral state to compare to [pitch, yaw, roll]
F_init = F_init/norm(F_init);

F_desired = [5,2,1]; % Desired orientation [pitch, yaw, roll]
F_desired = F_desired/norm(F_desired);

% Calculate yaw / pitch angles
yaw1 = atan2(F_desired(1), F_desired(3));
pitch1 = atan2(F_desired(2), F_desired(3));

net_pitch = rad2deg(pitch1);
net_yaw = rad2deg(yaw1);

% Convert yaw and pitch to pulse width signals
pulse_pitch = (10 * net_pitch) + 1500;
pulse_yaw = (10 * net_yaw) + 1500;

% Print pulse width for pitch and yaw
sprintf("%.2f", pulse_pitch)
sprintf("%.2f", pulse_yaw)

% Convert width signals back to angle rotations
re_pitch = (pulse_pitch - 1500) / 10;
re_yaw = (pulse_yaw - 1500) / 10;

% Convert angular rotations back to a normalized vector
F_re = [tan(deg2rad(re_yaw)), tan(deg2rad(re_pitch)), 1];
F_re = F_re / norm(F_re);

% Print original and recalculated output vectors
F_desired
F_re

%{
net_rotation_angle = acos(dot(F_init,F_desired)/(norm(F_desired)*norm(F_init)));

max_tolerable = 30*pi/360;

rotation_axis = cross(F_init,F_desired);
rotation_axis = rotation_axis/norm(rotation_axis);

kx = rotation_axis(1);
ky = rotation_axis(2);
kz = rotation_axis(3);

skewSymmetric_ofk = [
    0 -kz ky;
    kz 0 -kx;
    -ky kx 0];

i = eye(3);

rodriguesRotMatrix = i + sin(net_rotation_angle)*skewSymmetric_ofk ...
    + (1-cos(net_rotation_angle))*(skewSymmetric_ofk * skewSymmetric_ofk);

% Zero roll rotation matrix:
% [ cos(phi)*cos(theta)    -sin(phi)*cos(theta)     sin(theta)
%   sin(phi)               cos(phi)                 -sin(phi)
%   -cos(phi)*sin(theta)   0                        cos(theta)]

theta = atan2(-1*rodriguesRotMatrix(3,1), ...
    sqrt(rodriguesRotMatrix(1,1)^2 + rodriguesRotMatrix(2,1)^2));

phi = atan2(rodriguesRotMatrix(2,1),rodriguesRotMatrix(1,1));

% Verifying
if theta <= max_tolerable && phi <= max_tolerable
    disp('yay')
else
    disp('no')
end

% pitch rotation
Ry = [cos(theta), 0, sin(theta);
      0, 1, 0;
     -sin(theta), 0, cos(theta)];

% yaw rotation
Rz = [cos(psi), -sin(psi), 0;
      sin(psi),  cos(psi), 0;
      0,        0,        1];

R_pitchyaw = Ry*Rz;
%size(R_pitchyaw);
%size(F_init.')
F_pitchyaw = (R_pitchyaw*F_init.').';
F_pitchyaw = F_pitchyaw/norm(F_pitchyaw)

F_rodrigues = (rodriguesRotMatrix * F_init.').';
F_rodrigues = F_rodrigues/norm(F_rodrigues)

diff = (F_pitchyaw/norm(F_pitchyaw) - F_desired);
diffMag = norm(diff)*100 % Offset percentage


%% Quaternion method

% computing the dot product
rotation_angle = acos(dot(F_init, F_desired));
% constructing the quaternion of rotation
q = quaternion(cos(rotation_angle/2), sin(rotation_angle/2*kx), ...
   sin(rotation_angle/2*ky), sin(rotation_angle/2*kz));
%}

