function p = computeFootPosition(pos, eul, qleg, leg)
legSign = getLegSign(leg);
abadLoc = [(-1)^(floor(leg/3))*0.19, legSign*0.049, 0]';
R = eul2Rot(eul);
p = R*(abadLoc + legKinematics(qleg, leg)) + pos;
end