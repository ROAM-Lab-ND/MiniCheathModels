function kin = WBKinematicsSupport(robot)
import casadi.*

pos = SX.sym('p', [3, 1]);
eul = SX.sym('eul', [3, 1]);
qa = SX.sym('qa', [12, 1]);
vel = SX.sym('v', [3, 1]);
eulrate = SX.sym('eulr', [3, 1]);
qa_d = SX.sym('qad', [12, 1]);

q = [pos; eul; qa];
qd = [vel; eulrate; qa_d];

%% Generate Jacobian Derative funtion objects
J = cell(1,4);
Jd = cell(1,4);
for foot = 1:4
    J{foot} = computeFootJacobian(robot, q, foot);
    Jd{foot} = reshape(jacobian(reshape(J{foot}, [numel(J{foot}),1]),q)*qd, 3, robot.NB);
end
footJacobian = Function('footJacobian', {q}, J);
footJacobianPartial = Function('footJacobDerivative', {q, qd}, Jd);

%% Jacobian of foot accleration w.r.t. q in world frame
qdd = SX.sym('qdd',size(qd));
aFoot = cell(1, 4);
aFootBG = cell(1,4);    % Foot acceleration considering Bargurt terms
vFoot = cell(1, 4);
dvFoot_dq = cell(1, 4);
daFoot_dq = cell(1, 4);
daFoot_dqd = cell(1, 4);
daFootBG_dq = cell(1, 4);
daFootBG_dqd = cell(1, 4);
alpha = 8;
for foot = 1:4
    vFoot{foot} = J{foot}*qd;
    aFoot{foot} = J{foot}*qdd + Jd{foot}*qd;
    aFootBG{foot} = aFoot{foot} + alpha * vFoot{foot};
    dvFoot_dq{foot} = jacobian(vFoot{foot}, q);
    daFoot_dq{foot} = jacobian(aFoot{foot}, q);
    daFoot_dqd{foot} = jacobian(aFoot{foot}, qd);
    daFootBG_dq{foot} = jacobian(aFootBG{foot}, q);
    daFootBG_dqd{foot} = jacobian(aFootBG{foot}, qd);
end

footAcc = Function('footAcc', {q, qd, qdd}, aFoot);
footVelPartialDq = Function('footVelPartialDq', {q, qd}, dvFoot_dq);
footAccPartialDq = Function('footAccPartialDq', {q, qd, qdd}, daFoot_dq);
footAccPartialDv = Function('footAccPartialDv', {q, qd, qdd}, daFoot_dqd);
footAccBGPartialDq = Function('footAccBGPartialDq', {q, qd, qdd}, daFootBG_dq);
footAccBGPartialDv = Function('footAccBGPartialDv', {q, qd, qdd}, daFootBG_dqd);
%% Jacobian of external force w.r.t. q in world frame
F = SX.sym('F', [12,1]);
dJTF_dq = cell(1, 4);
for foot = 1:4
    S = zeros(3, 12);
    S(1:3, 3*(foot-1)+1: 3*foot) = eye(3);
    JTF = J{foot}'*S*F;
    dJTF_dq{foot} = jacobian(JTF, q);
end
footForcePartialDq = Function('footForcePartialDq', {q, F}, dJTF_dq);

kin.footJacobian = footJacobian;
kin.footJacobianPartial = footJacobianPartial;
kin.footAcc = footAcc;
kin.footVelPartialDq = footVelPartialDq;
kin.footAccPartialDq = footAccPartialDq;
kin.footAccPartialDv = footAccPartialDv;
kin.footAccBGPartialDq = footAccBGPartialDq;
kin.footAccBGPartialDv = footAccBGPartialDv;
kin.footForcePartialDq = footForcePartialDq;
end