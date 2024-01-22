currentFilePath = fileparts(mfilename('fullpath'));
%% Physical parameter of float base setup
% Build a tree model using mini cheath parameters
mc_param = getMiniCheetahParams();
robot_tree = buildTreeModelWithRotor(mc_param);

% Reduce the model to SRB using the following static pose
qB = zeros(6, 1);
qJ = [0, -pi/2, 0, 0, -pi/2, 0, 0, pi/2, 0, 0, pi/2, 0]';
q = [qB; qJ];
SRB_param = computeSRBDInertia(robot_tree, q);
SRBFuncs = SRBDynamicsSupportEulrate(SRB_param);

%% Generate CPP files
import casadi.*;
gen_params.mex = true;
gen_params.with_header = true;
gen_params.cpp = true;
c = CodeGenerator('SRBDynamics.cpp',gen_params);
c.add(SRBFuncs.Dynamics);
c.add(SRBFuncs.DynamicsDerivatives);
c.generate();
movefile('*.cpp', currentFilePath+"/CPP");
movefile('*.h', currentFilePath+"/CPP");

% SRBFuncs.Dynamics.generate('test.cpp');