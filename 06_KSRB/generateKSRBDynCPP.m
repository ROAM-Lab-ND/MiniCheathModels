currentFilePath = fileparts(mfilename('fullpath'));
%% Physical parameter of float base setup
mc_param = getMiniCheetahParams();
robot_tree = buildTreeModelWithRotor(mc_param);
qB = zeros(6, 1);
qJ = [0, -pi/2, 0, 0, -pi/2, 0, 0, pi/2, 0, 0, pi/2, 0]';
q = [qB; qJ];
SRB_param = computeSRBDInertia(robot_tree, q);
% SRB_param = compute_SRBD_inertia(mc_param);
SRB_param.abadLoc = mc_param.abadLoc;
modes = ["Free" , "Hybrid_Redundant" , "Hybrid_Compact"];
names = ["Free" , "Redundant" , "Compact"];
for i = 1:3
    fprintf("Generating Kino-SRB dynamics mode: %s...\n", names(i));
    %% Compute dynamics
    SRBFuncs = DynamicsSupportEulrate_Kino(SRB_param,modes(i));

    %% Generate configuration file
    config.xsize = SRBFuncs.xlength;
    config.usize = SRBFuncs.ulength;
    config.ysize = SRBFuncs.ylength;
    save(currentFilePath+"/DynamicsMex/KSRBconfig_"+names(i),"config");

    %% Generate CPP files
    import casadi.*;
    gen_params.mex = true;
    gen_params.with_header = true;
    gen_params.cpp = true;
    cpp_name = char("KinoSRBDynamics_"+names(i)+".cpp");
    c = CodeGenerator(cpp_name,gen_params);
    c.add(SRBFuncs.Dynamics);
    c.add(SRBFuncs.Output);
    c.add(SRBFuncs.DynamicsDerivatives);
    c.add(SRBFuncs.CMatrix);
    c.add(SRBFuncs.RunningCost);
    c.add(SRBFuncs.RunningCostDev);
    c.add(SRBFuncs.TerminalCost);
    c.add(SRBFuncs.TerminalCostDev);
    c.generate();
    
    
    mex('-largeArrayDims', cpp_name);
    
    movefile('*.cpp', currentFilePath+"/CPP");
    movefile('*.h', currentFilePath+"/CPP");
    movefile('*.mex*', currentFilePath+"/DynamicsMex");

end




% SRBFuncs.Dynamics.generate('test.cpp');


% xdot_ = SRBDynamics('SRBDynamics',x, u, contact)
% 
% [Ac_, Bc_] = SRBDynamics('SRBDynamicsDerivatives',x, u, contact)