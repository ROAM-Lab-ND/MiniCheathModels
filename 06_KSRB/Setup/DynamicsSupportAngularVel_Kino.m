function SRBFuncs = DynamicsSupportAngularVel_Kino(SRB_param)
% Generate mex function for kinodynamics which represents orientation using
% ZYX Euler angle reprsentation. Don't mess up with XYZ fixed-anlge.
import casadi.*
error("This version of KSRB dynamics was not fully developed!");

I = SRB_param.RotInertia; % Mini Cheetah body inertia
m = SRB_param.mass;
g = 9.81;

% mode selection
% Free              : All forces and velocity directly applied on the torso
% Hybrid_Redundant  : Forces apply when feet in stance and vel in flight
% Hybrid_Compact    : U acts as force in stance and vel in flight
%% Symbolic setup using Casadi
eul = SX.sym('eul', [3, 1]);
p = SX.sym('p', [3, 1]);
w = SX.sym('w',[3, 1]);             % angular velocity in body frame
v = SX.sym('v', [3, 1]);            % CoM velocity in world frame
pf = SX.sym('pf', [12, 1]);         % foothold location in world frame

F = SX.sym('F', [12, 1]);       % GRF in global frame
c = SX.sym('c', [4, 1]);        % contact status

vf = SX.sym('vf', [12, 1]);           % Foot velocity in global frame

%% Dynamicsz
R_body = eul2Rot(eul);        % orientation of body w.r.t. global frame
g_world = [0 0 -g]';
euldot = omegab2euldMat(eul) * w;
pdot = v;
Mt = 0;
Ft = 0; % total force in global frame
for leg = 1:4    
    r = R_body'*(pf(3*(leg-1)+1:3*leg) - p);    % foot location relative to CoM in body frame
    Mt = Mt + c(leg) * skew(r) * (R_body'*F(3*(leg-1)+1:3*leg));
    Ft = Ft + c(leg) * F(3*(leg-1)+1:3*leg);
end
wdot = I\(-skew(w)*I*w + Mt);
vdot = g_world + Ft/m;


%% Continuous-time dynamics
x = [p; eul; v; w; pf];       % state variable
xdot = [pdot;
        euldot;        
        vdot;
        wdot;
        vf];
u = [F;vf];                  % control variable
Ac = jacobian(xdot, x);
Bc = jacobian(xdot, u);

% Generate function handle using Casadi
Dynamics = Function('KinoSRBDynamics',{x, u, c}, {xdot});
DynamicsDerivatives = Function('KinoSRBDynamicsDerivatives',{x, u, c},{Ac,Bc});

SRBFuncs.Dynamics = Dynamics;
SRBFuncs.DynamicsDerivatives = DynamicsDerivatives;
end