
%% Physical parameter of float base setup
mc_param = getMiniCheetahParams();
robot_tree = buildTreeModelWithRotor(mc_param);
qB = zeros(6, 1);
qJ = [0, -pi/2, 0, 0, -pi/2, 0, 0, pi/2, 0, 0, pi/2, 0]';
q = [qB; qJ];
SRB_param = computeSRBDInertia(robot_tree, q);
% SRB_param = compute_SRBD_inertia(mc_param);
SRB_param.abadLoc = mc_param.abadLoc;
SRBFuncs_Free = DynamicsSupportEulrate_Kino(SRB_param, "Free");
SRBFuncs_Redundant = DynamicsSupportEulrate_Kino(SRB_param, "Hybrid_Redundant");
SRBFuncs_Compact = DynamicsSupportEulrate_Kino(SRB_param, "Hybrid_Compact");

%% Check with finite difference
pos = rand(3,1);
eul = rand(3,1);
v = rand(3,1);
omega = rand(3,1);
pFoot = rand(12,1);
x = [pos; eul; v; omega; pFoot];
F = rand(12,1);
V = rand(12,1);
contact = randi([0 1],4,1)
u = [F;V];
U = F;


%% Free Dynamics Finite Difference Check
disp("Free Dynamics:")

xdot = SRBFuncs_Free.Dynamics(x, u, contact);
[Ac, Bc] = SRBFuncs_Free.DynamicsDerivatives(x, u, contact);

Ac_FD = zeros(SRBFuncs_Free.xlength,SRBFuncs_Free.xlength);
eps = 1e-8;
for i=1:SRBFuncs_Free.xlength
    x_eps = zeros(SRBFuncs_Free.xlength,1);
    x_eps(i) = eps;
    xdot_post = SRBFuncs_Free.Dynamics(x + x_eps, u, contact);
    Ac_FD(:,i) = (full(xdot_post )- full(xdot))/eps;
end

Ac_D = Ac_FD - full(Ac);


Bc_FD = zeros(SRBFuncs_Free.xlength,SRBFuncs_Free.ulength);
eps = 1e-8;
for i=1:SRBFuncs_Free.ulength
    u_eps = zeros(SRBFuncs_Free.ulength,1);
    u_eps(i) = eps;
    xdot_post = SRBFuncs_Free.Dynamics(x, u + u_eps, contact);
    Bc_FD(:,i) = (full(xdot_post )- full(xdot))/eps;
end

Bc_D = Bc_FD - full(Bc);

fprintf("Difference between CasADi and finite difference (A Matrix): %f\n", norm(Ac_D));
fprintf("Difference between CasADi and finite difference (B Matrix): %f\n", norm(Bc_D));

xdot_ = KinoSRBDynamics_Free('KinoSRBDynamics',x, u, contact);
[Ac_, Bc_] = KinoSRBDynamics_Free('KinoSRBDynamicsDerivatives',x, u, contact);
fprintf("Same between CasADi MATLAB and C++ (x dot)   : %d\n", all(full(xdot == xdot_)))
fprintf("Same between CasADi MATLAB and C++ (A Matrix): %d\n", all(all(full(Ac == Ac_))))
fprintf("Same between CasADi MATLAB and C++ (B Matrix): %d\n", all(all(full(Bc == Bc_))))


%% Redundant Dynamics Finite Difference Check
disp("Redundant Dynamics:")

xdot = SRBFuncs_Redundant.Dynamics(x, u, contact);
[Ac, Bc] = SRBFuncs_Redundant.DynamicsDerivatives(x, u, contact);

Ac_FD = zeros(SRBFuncs_Redundant.xlength,SRBFuncs_Redundant.xlength);
eps = 1e-8;
for i=1:SRBFuncs_Redundant.xlength
    x_eps = zeros(SRBFuncs_Redundant.xlength,1);
    x_eps(i) = eps;
    xdot_post = SRBFuncs_Redundant.Dynamics(x + x_eps, u, contact);
    Ac_FD(:,i) = (full(xdot_post )- full(xdot))/eps;
end

Ac_D = Ac_FD - full(Ac);

Bc_FD = zeros(SRBFuncs_Redundant.xlength,SRBFuncs_Redundant.ulength);
eps = 1e-8;
for i=1:SRBFuncs_Redundant.ulength
    u_eps = zeros(SRBFuncs_Redundant.ulength,1);
    u_eps(i) = eps;
    xdot_post = SRBFuncs_Redundant.Dynamics(x, u + u_eps, contact);
    Bc_FD(:,i) = (full(xdot_post )- full(xdot))/eps;
end

Bc_D = Bc_FD - full(Bc);

fprintf("Difference between CasADi and finite difference (A Matrix): %f\n", norm(Ac_D));
fprintf("Difference between CasADi and finite difference (B Matrix): %f\n", norm(Bc_D));

xdot_ = KinoSRBDynamics_Redundant('KinoSRBDynamics',x, u, contact);
[Ac_, Bc_] = KinoSRBDynamics_Redundant('KinoSRBDynamicsDerivatives',x, u, contact);
fprintf("Same between CasADi MATLAB and C++ (x dot)   : %d\n", all(full(xdot == xdot_)))
fprintf("Same between CasADi MATLAB and C++ (A Matrix): %d\n", all(all(full(Ac == Ac_))))
fprintf("Same between CasADi MATLAB and C++ (B Matrix): %d\n", all(all(full(Bc == Bc_))))

%% Compact Dynamics Finite Difference Check
disp("Compact Dynamics:")

xdot = SRBFuncs_Compact.Dynamics(x, U, contact);
[Ac, Bc] = SRBFuncs_Compact.DynamicsDerivatives(x, U, contact);

Ac_FD = zeros(SRBFuncs_Compact.xlength,SRBFuncs_Compact.xlength);
eps = 1e-8;
for i=1:SRBFuncs_Compact.xlength
    x_eps = zeros(SRBFuncs_Compact.xlength,1);
    x_eps(i) = eps;
    xdot_post = SRBFuncs_Compact.Dynamics(x + x_eps, U, contact);
    Ac_FD(:,i) = (full(xdot_post )- full(xdot))/eps;
end

Ac_D = Ac_FD - full(Ac);


Bc_FD = zeros(SRBFuncs_Compact.xlength,SRBFuncs_Compact.ulength);
eps = 1e-8;
for i=1:SRBFuncs_Compact.ulength
    u_eps = zeros(SRBFuncs_Compact.ulength,1);
    u_eps(i) = eps;
    xdot_post = SRBFuncs_Compact.Dynamics(x, U + u_eps, contact);
    Bc_FD(:,i) = (full(xdot_post )- full(xdot))/eps;
end

Bc_D = Bc_FD - full(Bc);

fprintf("Difference between CasADi and finite difference (A Matrix): %f\n", norm(Ac_D));
fprintf("Difference between CasADi and finite difference (B Matrix): %f\n", norm(Bc_D));

xdot_ = KinoSRBDynamics_Compact('KinoSRBDynamics',x, U, contact);
[Ac_, Bc_] = KinoSRBDynamics_Compact('KinoSRBDynamicsDerivatives',x, U, contact);
fprintf("Same between CasADi MATLAB and C++ (x dot)   : %d\n", all(full(xdot == xdot_)))
fprintf("Same between CasADi MATLAB and C++ (A Matrix): %d\n", all(all(full(Ac == Ac_))))
fprintf("Same between CasADi MATLAB and C++ (B Matrix): %d\n", all(all(full(Bc == Bc_))))


%% Cross Check Between Different Mode.

U = zeros(SRBFuncs_Compact.ulength,1);
for leg = 1:4
    if contact(leg) == 1
        V(3*(leg-1)+1:3*leg) = zeros(3,1);
        U(3*(leg-1)+1:3*leg) = F(3*(leg-1)+1:3*leg);
    else
        F(3*(leg-1)+1:3*leg) = zeros(3,1);
        U(3*(leg-1)+1:3*leg) = V(3*(leg-1)+1:3*leg);
    end
end

u_Free = [F;V];
u_Redundant = u;

xdot = SRBFuncs_Free.Dynamics(x, u_Free, contact);
[Ac, Bc] = SRBFuncs_Free.DynamicsDerivatives(x, u_Free, contact);
Dynamics_Free.xdot = full(xdot);
Dynamics_Free.Ac = full(Ac);
Dynamics_Free.Bc = full(Bc);

xdot = SRBFuncs_Redundant.Dynamics(x, u_Redundant, contact);
[Ac, Bc] = SRBFuncs_Redundant.DynamicsDerivatives(x, u_Redundant, contact);
Dynamics_Redundant.xdot = full(xdot);
Dynamics_Redundant.Ac = full(Ac);
Dynamics_Redundant.Bc = full(Bc);

xdot = SRBFuncs_Compact.Dynamics(x, U, contact);
[Ac, Bc] = SRBFuncs_Compact.DynamicsDerivatives(x, U, contact);
Dynamics_Compact.xdot = full(xdot);
Dynamics_Compact.Ac = full(Ac);
Dynamics_Compact.Bc = full(Bc);


fprintf("Same between Free and Redundant (x dot)   : %d\n", ...
    all(Dynamics_Free.xdot == Dynamics_Redundant.xdot))
fprintf("Same between Free and Redundant (A Matrix): %d\n", ...
    all(all(Dynamics_Free.Ac == Dynamics_Redundant.Ac)))

fprintf("Same between Compact and Redundant (x dot)   : %d\n", ...
    all(Dynamics_Compact.xdot == Dynamics_Redundant.xdot))
fprintf("Same between Compact and Redundant (A Matrix): %d\n", ...
    all(all(Dynamics_Compact.Ac == Dynamics_Redundant.Ac)))

Dynamics_Redundant.Bc;
Bc_Red2Com = zeros(size(Dynamics_Compact.Bc));
for leg = 1:4
    if contact(leg) == 1
        Bc_Red2Com(:,3*(leg-1)+1:3*leg) = Dynamics_Redundant.Bc(:,3*(leg-1)+1:3*leg);
    else
        Bc_Red2Com(:,3*(leg-1)+1:3*leg) = Dynamics_Redundant.Bc(:,12+3*(leg-1)+1:12+3*leg);
    end
end
fprintf("Same between Compact and Redundant (B Matrix): %d\n", ...
    all(all(Dynamics_Compact.Bc == Bc_Red2Com)))
