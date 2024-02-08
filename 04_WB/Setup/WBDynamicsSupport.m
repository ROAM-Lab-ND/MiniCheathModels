function funcs = WBDynamicsSupport(robot)
import casadi.*

pos = SX.sym('p', [3, 1]);
eul = SX.sym('eul', [3, 1]);
qa = SX.sym('qa', [12, 1]);
vel = SX.sym('v', [3, 1]);
eulrate = SX.sym('eulr', [3, 1]);
qa_d = SX.sym('qad', [12, 1]);
u = SX.sym('u', [12, 1]);

q = [pos; eul; qa];
qd = [vel; eulrate; qa_d];

%% Generate Jacobian Derative funtion objects
J = cell(1,4);
Jd = cell(1,4);
pFoot = cell(1,4);
for foot = 1:4
    J{foot} = computeFootJacobian(robot, q, foot);
    Jd{foot} = reshape(jacobian(reshape(J{foot}, [numel(J{foot}),1]),q)*qd, 3, robot.NB);
    pFoot{foot} = computeFootPosition(pos, eul, q(3*(foot-1)+1:3*foot), foot);
end

%% Generate function objects of dynamics partials
S = [zeros(6,12); eye(12)]; % control selection
contactDynamics = cell(1,15);
contactDynamicsDerivatives = cell(1,15);

for i = 1:15
    ctact = arrayfun(@str2num, dec2bin(i, 4));
    cfoot_idx = find(ctact == 1); % index of foot in contact
    
    [H, C] = HandC(robot, q, qd);
    
    % Construct KKT contact dynamics
    Jc = [];
    Jcd = [];
    Pc = [];
    for foot = cfoot_idx
        Jc = [Jc; J{foot}];
        Jcd = [Jcd; Jd{foot}];
        Pc = [Pc; diag([0,0,1])*pFoot{foot}];
    end
    
    if isempty(Jc)
        K = H;
        b = S*u - C;
    else
        alpha = 10; % Baumgart stabilization (velocity level)
        beta = 0;
%         alpha = 0;
        K = [H, -Jc';Jc, zeros(size(Jc,1),size(Jc,1))];
        b = [S*u - C; -Jcd*qd - 2*alpha*Jc*qd - beta^2*Pc];
    end
    f_KKT = K\b;
    qdd = f_KKT(1:robot.NB, 1);
    lambda = SX.zeros(12, 1);
    for c = 1:length(cfoot_idx)
        fid = cfoot_idx(c);
        lambda(3*(fid-1)+1:3*fid) = f_KKT(robot.NB+ (3*(c-1)+1:3*c));
    end
    
    f = [qd; qdd];
    funcName = sprintf('contactDyn%d', i);
    contactDynamics{i} = Function(funcName, {q,qd,u},{qdd, lambda});

%     Ac = jacobian(f, [q; qd]);
%     Bc = jacobian(f, u);
%     Cc = jacobian(lambda, [q; qd]);
%     Dc = jacobian(lambda, u);    
%     funcName = sprintf('contactDynPartial%d', i);
%     contactDynamicsDerivatives{i} = Function(funcName, {q, qd, u}, {Ac, Bc, Cc, Dc});    
end
qdd_fly = H\(S*u - C);
freeDynamics = Function('freeDyn', {q,qd,u}, {qdd_fly});
% freeDynamicsPartial = Function('freeDynPartial', {q,qd,u},{jacobian(f_fly, q)});
massAndNonlinearTerms = Function('massAndNle',{q, qd}, {H, C});

funcs.dynamics.contact = contactDynamics;
funcs.dynamics.free = freeDynamics;
funcs.massAndNle = massAndNonlinearTerms;
end