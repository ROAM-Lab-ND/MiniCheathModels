function I = buildSpatialInertia(P)
% P : 4x4 pseudo inertia
I = zeros(6);
m = P(4, 4);
h = P(1:3, end);
E = P(1:3, 1:3);
Ibar = trace(E) * eye(3) - E;
I(1:3, 1:3) = Ibar;
I(1:3, 4:6) = skew2(h);
I(4:6,1:3) = skew2(h)';
I(4:6,4:6) = m*eye(3);
end