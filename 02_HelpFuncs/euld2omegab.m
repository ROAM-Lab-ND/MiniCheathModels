function omegab = euld2omegab(eul, euld)
% Matrix that transforms derivative of euler angle in body frame to angular velocity
% in body coordinate

T = euld2omegabMat(eul);
omegab = T * euld;
end

