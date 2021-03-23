function [eqc, ineqc] = physicalConstraint(X, parameters, gamval, R)

%tmp = makehgtform('zrotate',gamval); %transform matrix
%RotMat = tmp(1:3,1:3); %take upper left 3x3

%[cst, ~, ~] = CheckCollision( 0, X, parameters.Xf, parameters.numlinks, gamval, R, RotMat, 0, 0, false);

eqc = [];
ineqc = []; %use no constraints at first
%ineqc = [-cst; X(1,:)-X(2,:); X(3,:)-X(4,:)]; %second and third rows prevent knee hyperextension

end