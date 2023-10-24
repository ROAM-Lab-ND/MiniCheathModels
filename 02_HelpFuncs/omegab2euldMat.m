function T = omegab2euldMat(eul)
% Matrix that transforms angular velocity in body frame to Euler rate

% Helpful functions
s = @(x) sin(x);
c = @(x) cos(x);

yaw = eul(1);
pitch = eul(2);
roll = eul(3);

% Transformation matrix
T = [0, s(roll),   c(roll);
     0, c(roll)*c(pitch), -s(roll)*c(pitch);
     c(pitch),  s(roll)*s(pitch),  c(roll)*s(pitch)]/c(pitch); 

end

