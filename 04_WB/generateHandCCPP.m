
%% get function hanldes
params = getMiniCheetahParams();
robot = buildTreeModel(params);
dynFuncs = WBDynamicsSupport(robot);
kinFuncs = WBKinematicsSupport(robot);


%% generate cpp files
import casadi.*;
gen_params.mex = true;
gen_params.with_header = true;
gen_params.cpp = true;
c = CodeGenerator('HandC.cpp',gen_params);
c.add(dynFuncs.massAndNle);
c.generate();

c = CodeGenerator('MC_kinematics_casadi.cpp',gen_params);
c.add(kinFuncs.footAcc);
c.add(kinFuncs.footJacobian);
c.generate();
movefile *.cpp 04_WB/CPP
movefile *.h 04_WB/CPP
