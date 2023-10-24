function p = legKinematics(q,leg)
% Leg kinematics in local frame, where the orientation of local frame is
% the same as the body frame, and its origin is at the center of abad
% p-> foot position in local frame
s = @(x) sin(x);
c = @(x) cos(x);
Rx = @(x) [1,0,0;0 c(x) -s(x);0 s(x) c(x)];
Ry = @(x) [c(x),0,s(x);0 1 0;-s(x),0,c(x)];
R2T = @(R) [R, zeros(3,1);zeros(1,3), 1];

l0 = 0.062; % abad link length
l1 = 0.209; % hip link length
l2 = 0.195; % knee link length
legSign = getLegSign(leg);

T1 = R2T(Rx(q(1))) * trvec2tform(legSign*[0 l0 0]);
T2 = R2T(Ry(q(2))) * trvec2tform([0 0 -l1]);
T3 = R2T(Ry(q(3)));

p = eye(3,4)*T1*T2*T3*cart2hom([0 0 -l2])';
end