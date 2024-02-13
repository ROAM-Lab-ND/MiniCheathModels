function SRBFuncs = DynamicsSupportEulrate_Kino(SRB_param,mode)
% Generate mex function for kinodynamics which represents orientation using
% ZYX Euler angle reprsentation. Don't mess up with XYZ fixed-anlge.
import casadi.*

I = SRB_param.RotInertia; % Mini Cheetah body inertia
m = SRB_param.mass;
g = 9.81;

% mode selection
% Free              : All forces and velocity directly applied on the torso
% Hybrid_Redundant  : Forces apply when feet in stance and vel in flight
% Hybrid_Compact    : U acts as force in stance and vel in flight
%% Symbolic setup using Casadi


eul = SX.sym('eul', [3, 1]);        % ZYX euler anlge in body-fixed frame
euldot = SX.sym('euld', [3, 1]);    % rate of change of euler angle in body frame
p = SX.sym('p', [3, 1]);
v = SX.sym('v', [3, 1]);            % CoM velocity in world frame
pf = SX.sym('pf', [12, 1]);         % foothold location in world frame
c = SX.sym('c', [4, 1]);            % contact status
df = SX.sym('df',[4, 1]);           % Feet distance to abad joint


if mode == "Free"
    F = SX.sym('F', [12, 1]);           % GRF in global frame
    V = SX.sym('V', [12, 1]);           % Foot velocity in global frame
    u = [F; V];                         % control variable
elseif mode == "Hybrid_Redundant"
    F = SX.sym('F', [12, 1]);           % GRF in global frame
    V = SX.sym('V', [12, 1]);           % Foot velocity in global frame
    c_ = 1 - c;                         % Inverted contact status
    u = [F; V];                         % control variable
elseif mode == "Hybrid_Compact"
    U = SX.sym('U', [12, 1]);           % GRF in global frame in stance OR
    % Foot velocity in global frame in flight phase
    c_ = 1 - c;                         % Inverted contact status
    u = U;                              % control variable
end

%% Transformation matrix between euler rate and angular velocity
T = omegab2euldMat(eul);
Tinv = euld2omegabMat(eul);
Tdot = SX.zeros(3, 3);
for i = 1:3
    for j = 1:3
        Tdot(i, j) = jacobian(T(i, j), eul) * euldot;
    end
end
%% Dynamics
R_body = eul2Rot(eul); % orientation of body w.r.t. global frame (World_R_Body)
R_world = R_body.';
Rwdot = SX.zeros(3, 3); % Seems not used
for i = 1:3
    for j = 1:3
        Rwdot(i, j) = jacobian(R_world(i, j), eul) * euldot;
    end
end
g_world = [0 0 -g]';
w = Tinv * euldot;
pdot = v;
Mt = 0;
Ft = 0; % total force in global frame
vf = SX.zeros(12, 1);   % foothold velocity in world frame
for leg = 1:4
    leg_i = 3*(leg-1)+1:3*leg;
    r = R_body'*(pf(leg_i) - p);    % foot location relative to CoM in body frame
    df(leg) = norm(r-SRB_param.abadLoc{leg});
    if mode == "Free"
        Mt = Mt + skew(r) * (R_body'*F(leg_i));
        Ft = Ft + F(leg_i);
        vf(leg_i) = V(leg_i);
    elseif mode == "Hybrid_Redundant"
        Mt = Mt + c(leg) * skew(r) * (R_body'*F(leg_i));
        Ft = Ft + c(leg) * F(leg_i);
        vf(leg_i) = c_(leg)*V(leg_i);
    elseif mode == "Hybrid_Compact"
        Mt = Mt + c(leg) * skew(r) * (R_body'*U(leg_i));
        Ft = Ft + c(leg) * U(leg_i);
        vf(leg_i) = c_(leg)*U(leg_i);
    end
end
wdot = I\(-skew(w)*I*w + Mt);
vdot = g_world + Ft/m;
eulddot = Tdot*w + T * wdot;

%% Continuous-time dynamics
x = [p; eul; v; euldot; pf];       % state variable
xdot = [pdot;
        euldot;
        vdot;
        eulddot;
        vf];
y = df;                            % output variable
Ac = jacobian(xdot, x);
Bc = jacobian(xdot, u);
Cc = jacobian(y, x);
Dc = jacobian(y, u);

% Generate function handle using Casadi
Dynamics = Function('KinoSRBDynamics',{x, u, c}, {xdot});
Output = Function('KinoSRBOutput',{x}, {y});
DynamicsDerivatives = Function('KinoSRBDynamicsDerivatives',{x, u, c},{Ac,Bc,Cc,Dc});
CMatrix = Function('KinoSRBCMatrix',{x}, {Cc});


SRBFuncs.Dynamics = Dynamics;
SRBFuncs.Output   = Output;
SRBFuncs.DynamicsDerivatives = DynamicsDerivatives;
SRBFuncs.CMatrix   = CMatrix;
SRBFuncs.xlength = length(x);
SRBFuncs.ulength = length(u);
SRBFuncs.ylength = length(y);
%% Cost Computation


p_des = SX.sym('p', [3, 1]);            % Desired CoM Pos in world frame
v_des = SX.sym('v', [3, 1]);            % Desired CoM velocity in world frame
eul_des = SX.sym('eul', [3, 1]);        % Desired ZYX euler anlge in body-fixed frame
euldot_des = SX.sym('euld', [3, 1]);    % Desired rate of change of euler angle in body frame
pf_des = SX.sym('pf', [12, 1]);         % Desired foothold location in world frame
xd = [  p_des;
        eul_des;
        v_des;
        euldot_des;
        pf_des];

r_des = SX.sym('rd',[12,1]);        % Desire foot position relative to CoM in body frame

if mode == "Free"
    Fd = SX.sym('Fd', [12, 1]);     % GRF in global frame
    Vd = SX.sym('Vd', [12, 1]);     % Foot velocity in global frame
    ud = [Fd; Vd];                  % control variable
elseif mode == "Hybrid_Redundant"
    Fd = SX.sym('Fd', [12, 1]);     % GRF in global frame
    Vd = SX.sym('Vd', [12, 1]);     % Foot velocity in global frame
    ud = [Fd; Vd];                  % control variable
elseif mode == "Hybrid_Compact"
    Ud = SX.sym('Ud', [12, 1]);     % GRF in global frame in stance OR
    % Foot velocity in global frame in flight phase
    ud = Ud;                        % control variable
end

% yv = SX.sym('y',[length(y),1]);
% yd = SX.sym('yd',[length(y),1]);


Qp  = SX.sym('Qp',[3, 1]);          % Weighting for CoM Pos
Qe  = SX.sym('Qeul',[3, 1]);        % Weighting for Euler angle
Qv  = SX.sym('Qv',[3, 1]);          % Weighting for CoM Vel
Qed = SX.sym('Qeuldot',[3, 1]);     % Weighting for Euler rate
Qgs = SX.sym('Qpfgs',[3, 1]);       % Weighting for global foot pos (world frame) stance phase
Qgf = SX.sym('Qpfgf',[3, 1]);       % Weighting for global foot pos (world frame) flight phase
Qr  = SX.sym('Qpfr',[3, 1]);        % Weighting for relative foot pos (body frame)
Rf  = SX.sym('RGRF',[3, 1]);        % Weighting for GRF
Rv  = SX.sym('Rv',[3, 1]);          % Weighting for relative foot velocity (body frame)

Qvec = [Qp;Qe;Qv;Qed;Qgs;Qgf;Qr];   % Vectorized Q matrix (diagnoal)
Rvec = [Rf;Rv];                     % Vectorized R matrix (diagnoal)


%       CoM Pos         Euler Angle             CoM Vel         Rate of Euler Angle
l   = ws(p-p_des,Qp) + ws(eul-eul_des,Qe) + ws(v-v_des,Qv) + ws(euldot-euldot_des,Qed);
phi = ws(p-p_des,Qp) + ws(eul-eul_des,Qe) + ws(v-v_des,Qv) + ws(euldot-euldot_des,Qed);


for leg = 1:4
    leg_i = 3*(leg-1)+1:3*leg;      % Index to select this foot
    r = R_body'*(pf(leg_i) - p);    % foot location relative to CoM in body frame
    r_des_leg = r_des(leg_i);       % Desired foot location relative to CoM

    % Relative feet pos in body frame
    l   = l   + ws(r-r_des_leg,Qr);
    phi = phi + ws(r-r_des_leg,Qr);

    if mode == "Free"
        % Global foot pos (always use stance)
        l = l + ws(pf(leg_i) - pf_des(leg_i), Qgs);
        phi = phi + ws(pf(leg_i) - pf_des(leg_i), Qgs);
        % Input cost: GRF and foot vel
        l = l + ws( F(leg_i) -     Fd(leg_i), Rf) ...
              + ws(R_body' *(V(leg_i)-v)    , Rv);
    elseif mode == "Hybrid_Redundant"
        % Global foot pos
        l = l + ws(pf(leg_i) - pf_des(leg_i), Qgs*c(leg)+Qgf*c_(leg));
        phi = phi + ws(pf(leg_i) - pf_des(leg_i), Qgs*c(leg)+Qgf*c_(leg));
        % Input cost: GRF and foot vel
        l = l + c(leg)  * ws(F(leg_i) - Fd(leg_i), Rf) ...
              + c_(leg) * ws(R_body'*(V(leg_i)-v), Rv);
    elseif mode == "Hybrid_Compact"
        % Global foot pos
        l = l + ws(pf(leg_i) - pf_des(leg_i), Qgs*c(leg)+Qgf*c_(leg));
        phi = phi + ws(pf(leg_i) - pf_des(leg_i), Qgs*c(leg)+Qgf*c_(leg));
        % Input cost: GRF and foot vel
        l = l + c(leg)  * ws(U(leg_i) - Ud(leg_i), Rf) ...
              + c_(leg) * ws(R_body'*(U(leg_i)-v), Rv);
    end
end

lx = jacobian(l,x).';
lu = jacobian(l,u).';
% ly = jacobian(l,yv);
ly = SX.zeros(length(y),1);
lxx = jacobian(lx,x);
lux = jacobian(lu,x);
luu = jacobian(lu,u);
% lyy = jacobian(ly,x);
lyy = SX.zeros(length(y),length(y));


phi_G = jacobian(phi,x).';
phi_H = jacobian(phi_G,x);


RunningCost = Function('KSRBRunningCost',{x,u,c,xd,ud,r_des,Qvec,Rvec},{l});
RunningCostDev = Function('KSRBRunningCostDev',...
    {x,u,c,xd,ud,r_des,Qvec,Rvec},{lx,lu,ly,lxx,lux,luu,lyy});

TerminalCost = Function('KSRBTerminalCost',{x,c,xd,r_des,Qvec},{phi});
TerminalCostDev = Function('KSRBTerminalCostDev',{x,c,xd,r_des,Qvec},{phi_G,phi_H});

SRBFuncs.RunningCost        = RunningCost;
SRBFuncs.RunningCostDev     = RunningCostDev;
SRBFuncs.TerminalCost       = TerminalCost;
SRBFuncs.TerminalCostDev    = TerminalCostDev;

end




function s = ws(x,Q)
% warning("Calling a slow square function...");
sx = length(x);
if all(size(Q) == [sx,1])
    s = x.' * diag(Q) * x;
elseif all(size(Q) == [sx,sx])
    s = x.' * Q * x;
else
    error("Inconsistent size of weighting and variable");
end
end