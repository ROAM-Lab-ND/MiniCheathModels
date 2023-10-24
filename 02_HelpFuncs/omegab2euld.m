function euld = omegab2euld(eul, omegab)
% Matrix that transforms angular velocity in body frame to Euler rate
% eul : [yaw, pitch, roll]
% omegab : [wx, wy, wz]

% Helpful functions
s = @(x) sin(x);
c = @(x) cos(x);

p = eul(2); % pitch
r = eul(3); % roll

% Transformation matrix
T = [0, s(r),   c(r);
     0, c(r)*c(p), -s(r)*c(p);
     c(p),  s(r)*s(p),  c(r)*s(p)]/c(p); 
euld = T*omegab;
end