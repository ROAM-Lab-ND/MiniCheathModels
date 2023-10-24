function SRB_param = computeSRBDInertia(tree_model, q)
% Input: tree_model: kinematic tree model constructed with spatial_v2
%        q:          default generalized configuration for model reduction

qd = zeros(length(q), 1);
ret = EnerMo(tree_model, q, qd);

I = ret.Itot;
[mass, ~, RotInertia] = mcI(I);
SRB_param.mass = mass;
SRB_param.RotInertia = RotInertia;

% Check if mass are equal
assert(abs(mass - ret.mass) < 1e-10, "check for total mass of SRB");
end

