% Modeling the migration between 3D orientation axes
% of the TVC.

F_init = [5,4,15]; % To be edited
init_mag = norm(F_init);
F_init = F_init/norm(F_init);

F_desired = [2,3,17]; % Also to be edited
mag_desired = norm(F_desired);
F_desired = F_desired/norm(F_desired);

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

psi = atan2(rodriguesRotMatrix(2,1),rodriguesRotMatrix(1,1));

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