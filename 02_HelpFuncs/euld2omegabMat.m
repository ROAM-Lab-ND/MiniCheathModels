function T = euld2omegabMat(eul)
% Matrix that transforms Euler rate to Angular veloctiy in body frame

% Helpful functions
s = @(x) sin(x);
c = @(x) cos(x);

pitch = eul(2);
roll = eul(3);

T = [-s(pitch),       0,    1;
     c(pitch)*s(roll), c(roll),   0;
     c(pitch)*c(roll), -s(roll),  0];
end

