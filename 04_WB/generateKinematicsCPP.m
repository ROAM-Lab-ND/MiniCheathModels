
%% get function hanldes
params = getMiniCheetahParams();
robot = buildTreeModel(params);
kinFuncs = WBKinematicsSupport(robot);

%% generate cpp files
import casadi.*;
gen_params.mex = true;
gen_params.with_header = true;
gen_params.cpp = true;
c = CodeGenerator('MCKinematicsDerivativs.cpp',gen_params);
c.add(kinFuncs.footVelPartialDq);
c.add(kinFuncs.footAccPartialDq);
c.add(kinFuncs.footAccPartialDv);
c.add(kinFuncs.footForcePartialDq);
c.generate();
movefile *.cpp 04_WB/CPP
movefile *.h 04_WB/CPP
