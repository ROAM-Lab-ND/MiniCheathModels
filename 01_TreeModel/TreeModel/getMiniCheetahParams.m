function robot = getMiniCheetahParams()
% Leg Pattern
% [1,2]  -> [FL, FR]
% [3,4]     [HL, HR]   
robot = struct(...
        'bodyCoM',  zeros(3,1),...
        'hipLinkCoM', [0, 0.016, -0.02]',...
        'kneeLinkCoM',[0, 0, -0.061]',... 
        'abadLinkCoM', [0, 0.036, 0]',...
        ...
        'bodyMass', 3.3, ...
        'abadLinkMass' , 0.54,...
        'hipLinkMass' , 0.634,...
        'kneeLinkMass' , 0.064,...        
        ... % link rotational inertia w.r.t. its CoM expressed in its own frame
        'bodyRotInertia' , diag([11253,36203,42673])*1e-6,...          % trunk rotational inertia
        'abadRotInertia' , [381, 58, 0.45; 58, 560, 0.95; 0.45, 0.95, 444]*1e-6,...  % abad rotational inertia
        'hipRotInertia' , [1983, 245, 13; 245, 2103, 1.5; 13, 1.5, 408]*1e-6,...     % hip rotational inertia
        'kneeRotInertia' , diag([245, 248, 6])*1e-6,...                  % knee rotational inertia        
        ...
        'bodyLength' , 0.19 * 2,...
        'bodyWidth' , 0.049 * 2,...
        'bodyHeight' , 0.05 * 2,...
        ...
        'abadLinkLength', 0.062,...
        'hipLinkLength' , 0.209,...
        'kneeLinkLength' , 0.195,...
        ...
        'abadGearRatio', 6,...
        'hipGearRatio', 6,...
        'kneeGearRatio', 9.33,...
        ...
        'rotorRotInertiaZ', diag([33,33,63])*1e-6,... 
        'rotorMass', 0.055,...
        'rotorCoM', zeros(3,1),...
        ...
        'linkwidth', 0.03);
    
    robot.rotorRotInertiaX = ry(pi/2)*robot.rotorRotInertiaZ*ry(pi/2)'; % ry : Featherstones's convention for rotation matrices
    robot.rotorRotInertiaY = rx(pi/2)*robot.rotorRotInertiaZ*rx(pi/2)';
    
    robot.abadLoc{1} = [robot.bodyLength, robot.bodyWidth, 0]'/2;      % FL
    robot.abadLoc{2} = [robot.bodyLength, -robot.bodyWidth, 0]'/2;     % FR
    robot.abadLoc{3} = [-robot.bodyLength, robot.bodyWidth, 0]'/2;     % HL
    robot.abadLoc{4} = [-robot.bodyLength, -robot.bodyWidth, 0]'/2;    % HR
    
    robot.abadRotorLoc{1} = [0.125, 0.049, 0]';         % FL
    robot.abadRotorLoc{2} = [0.125, -0.049, 0]';        % FR
    robot.abadRotorLoc{3} = [-0.125, 0.049, 0]';        % HL
    robot.abadRotorLoc{4} = [-0.125, -0.049, 0]';       % HR

    robot.hipLoc{1} = [0, robot.abadLinkLength, 0]';     % FL
    robot.hipLoc{2} = [0, -robot.abadLinkLength, 0]';    % FR
    robot.hipLoc{3} = [0, robot.abadLinkLength, 0]';     % RL
    robot.hipLoc{4} = [0, -robot.abadLinkLength, 0]';    % RR    
    
    robot.hipRotorLoc{1} = [0, 0.04, 0]';               % FL
    robot.hipRotorLoc{2} = [0, -0.04, 0]';              % FR
    robot.hipRotorLoc{3} = [0, 0.04, 0]';               % HL
    robot.hipRotorLoc{4} = [0, -0.04, 0]';              % HR

    robot.kneeLoc = [0, 0, -robot.hipLinkLength]';        
    
    robot.kneeRotorLoc = zeros(3,1);     
end