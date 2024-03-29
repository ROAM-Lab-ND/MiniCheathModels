%% Physical parameter of float base setup
mc_param = getMiniCheetahParams();
robot_tree = buildTreeModelWithRotor(mc_param);
qB = zeros(6, 1);
qJ = [0, -pi/2, 0, 0, -pi/2, 0, 0, pi/2, 0, 0, pi/2, 0]';
q = [qB; qJ];
SRB_param = computeSRBDInertia(robot_tree, q);
% SRB_param = compute_SRBD_inertia(mc_param);
SRBFuncs = SRBDynamicsSupportEulrate(SRB_param);

%% Check with finite difference
pos = rand(3,1);
eul = rand(3,1);
v = rand(3,1);
omega = rand(3,1);
x = [pos; eul; v; omega];
F = rand(12,1);
contact = [0,1,1,0];
pFoot = rand(12,1);
xdot = SRBFuncs.Dynamics(x, F, pFoot, contact);
[Ac, Bc] = SRBFuncs.DynamicsDerivatives(x, F, pFoot, contact);
Ac_FD = zeros(12,12);
eps = 1e-8;
for i=1:12
    x_eps = zeros(12,1);
    x_eps(i) = eps;
    xdot_post = SRBFuncs.Dynamics(x + x_eps, F, pFoot, contact);
    Ac_FD(:,i) = (full(xdot_post )- full(xdot))/eps;
end

norm(Ac_FD - full(Ac))
