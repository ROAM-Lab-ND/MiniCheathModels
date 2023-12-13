addpath(genpath('../'));
addpath(genpath("../../../spatial_v2_extended"));
% addpath('/home/heli/Sources/casadi-linux-matlabR2014b-v3.5.5', '-end');
addpath('C:\Program Files (x86)\casadi-windows-matlabR2016a-v3.5.3','-end');

%% get function hanldes
params = getMiniCheetahParams();
robot = buildTreeModel(params);
% robot = buildTreeModelWithRotor(params);
dynFuncs = DynamicsSupport(robot);


%% Simulate free fall dynamics with simple joint PD control
q = 0*ones(18, 1);
q(1) = 1;
qd = 0*ones(18, 1);
qd(7:end) = ones(12,1);
dt = 0.001;

qJdes = [0,-1.1,2.2,0,-1.1,2.2,0,-1.1,2.2,0,-1.1,2.2]';
q_array = [q];
qd_array = [qd];
for k = 1:3000
    u = -2*(q(7:end)-qJdes) - 0.2*qd(7:end);
    qdd = full(dynFuncs.dynamics.free(q,qd,u));
    q = q + qd * dt;  
    qd = qd + qdd * dt;         
    q_array(:,end+1) = q;
    qd_array(:,end+1) = qd;
end

%% Compute the total momentum along the free-fall trajectory
h = [];
for k = 1:size(q_array, 2)
    ret = EnerMo(robot, q_array(:,k), qd_array(:,k));   
    X = plux(eye(3), -ret.cm);
    h_com = X'*ret.htot;    
    
%     % Alternative to compute spatial momentum using CMM (Centroidal Momentum Matrix)
%     [A,info] = CMM(robot, q_array(:,k));
%     h_com = A*qd_array(:,k);

    h(:,end+1) = h_com(1:3);
end

figure
plot(h')