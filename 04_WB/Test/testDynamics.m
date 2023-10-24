addpath(genpath('../'));
addpath(genpath("../../../spatial_v2_extended"));
addpath('/home/heli/Sources/casadi-linux-matlabR2014b-v3.5.5', '-end');

%% get function hanldes
params = getMiniCheetahParams();
robot = buildTreeModelWithRotor(params);
% robot = buildTreeModel(params);
dynFuncs = DynamicsSupport(robot);


%% Test dynamics
q = ones(18, 1);
qd = ones(18, 1);
u = zeros(12, 1);

qdd_fly = full(dynFuncs.dynamics.free(q,qd,u));
contact = [1,1,1,1];
[qdd_contact, GRF] = dynFuncs.dynamics.contact{bin2dec(num2str(contact))}(q,qd,u);
fprintf("qdd_fly = \n"); disp(full(qdd_fly)');
fprintf("qdd_contact = \n"); disp(full(qdd_contact)');
fprintf("GRF = \n"); disp(full(GRF)');

%% Test kinematics
footPositions = [];
for foot = 1:4
    pFoot = computeFootPosition(q(1:3), q(4:6), q(3*(foot-1)+1:3*foot), foot);
    footPositions = [footPositions; pFoot];
end
fprintf("foot position = \n");disp(footPositions');

%% Compare Results of Cheetah Simulator

load("sim_state.mat");
k = 400;   % flight dynamics
[q_k,qd_k,qJdd_ref,u_k] = getOneState(k, sim_state);

dt = sim_state.time(k+1) - sim_state.time(k);
qdd_k = full(dynFuncs.dynamics.free(q_k,qd_k,u_k));
qJdd_k = qdd_k(end-11:end);

fprintf("qJdd_ref = \n"); disp(qJdd_ref');
fprintf("qJdd_k = \n"); disp(qJdd_k');
fprintf("diff = \n"); disp(qJdd_k' - qJdd_ref');

%%
function [q,qd,qJdd,u] = getOneState(k, sim_data)
eul = sim_data.rpy(k,[3,2,1])'; 
omegab = sim_data.omegab(k,:)'; 
euld = omegab2euld(eul, omegab);
pos = sim_data.p(k,:)'; 
v = sim_data.v(k,:)';
qJ = reshape(squeeze(sim_data.q(k,:,:))',[12, 1]);
qJd = reshape(squeeze(sim_data.qd(k,:,:))',[12, 1]);
qJdd = reshape(squeeze(sim_data.qdd(k,:,:))',[12, 1]);
u = reshape(squeeze(sim_data.tau(k,:,:))',[12, 1]);

% flip left and right
qJ = qJ([4:6,1:3,10:12,7:9]);
qJd = qJd([4:6,1:3,10:12,7:9]);
qJdd = qJdd([4:6,1:3,10:12,7:9]);
u = u([4:6,1:3,10:12,7:9]);

q = [pos; eul; qJ];
qd = [v; euld; qJd];
end

