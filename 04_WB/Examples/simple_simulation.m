% This script simulates the WB dyamics with full stance
% This simple simulation does not incorporate a controller, and
% aims to show how to use the whole-body dynamics, and visualize the
% simulated trajectory

%% get function hanldes
params = getMiniCheetahParams();
robot = buildTreeModel(params);
dynFuncs = WBDynamicsSupport(robot);
full_contact_dyn = dynFuncs.dynamics.contact{bin2dec(num2str([1,1,1,1]))};
kinFuncs = WBKinematicsSupport(robot);
foot_Jacobian = kinFuncs.footJacobian;

%% Simulate full contact dynamics
pos = [0, 0, 0.28]';
eul = zeros(3, 1);
qJ = repmat([0, -0.8, 1.6]', [4,1]);
q = [pos; eul; qJ];
qd = .5*ones(18, 1);
u = .5*ones(12, 1);
dt = 0.005;
N = 30;
t = 0;
q_traj = [];
times = [];
[JFL, JFR, JHL, JHR] = foot_Jacobian(q);
JFL = full(JFL(:,7:9));
JFR = full(JFR(:,10:12));
JHL = full(JHL(:,12:14));
JHR = full(JHR(:,14:18));
for k = 1:N
    [qdd,~] = full_contact_dyn(q, qd, u);    
    q = q + qd * dt;
    qd = qd + qdd * dt;
    q_traj = [q_traj, q];
    times(end+1) = t;
    t = t + dt;
end
q_traj = full(q_traj);

%% visualize motion
graphics_robot = importrobot("03_Graphics/urdf/mini_cheetah_simple_correctedInertia.urdf");
graphics_robot = floatBaseQuadruped(graphics_robot);
pos_traj = q_traj(1:3, :);
eul_traj = q_traj(4:6, :);
qJ_traj = q_traj(7:end, :);
graphics_options.show_floor = true;
graphics_options.hide_legs = false; % set to true if only need to visualize the floating base
animate(graphics_robot, times, pos_traj, eul_traj, qJ_traj, dt*10, graphics_options);

