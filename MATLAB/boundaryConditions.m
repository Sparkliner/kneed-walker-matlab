function [ineqc, eqc] = boundaryConditions(xF,x0,parameters,R,stanceAngle,gamval)
%x0-beginning of step
%xF-end of step, just before collision
%tmp = makehgtform('zrotate',gamval); %transform matrix
%RotMat = tmp(1:3,1:3); %take upper left 3x3
xF = xF(1:parameters.numlinks*2,:);
x0 = x0(1:parameters.numlinks*2,:);
numlinks = parameters.numlinks;
Xf = parameters.Xf;

%make sure feet contact ground at beginning/end
index = 1;
footPosition = zeros(3,2);
for I = [x0,xF]
        JointAngleCell = num2cell(I(1:numlinks));

        %position of bottom of striking foot relative to foot center in tilted RF
        offset = [-R;0;0];

        %calculate using function
        jointPositions = (Xf(JointAngleCell{:}) + offset);

        footPosition(:,index) = jointPositions(:,end);
        index = index+1;
end
eqc2 = footPosition(1,:)';
eqc3 = sum(footPosition(2,:)); %gait must be symmetrical

ineqc = []; %no inequality constraint

%[ value1, ~, ~ ] = CheckCollision( 0, x0, parameters.Xf, parameters.numlinks, gamval, R, RotMat, 0, 0, false);
%[ value2, ~, ~ ] = CheckCollision( 0, xF, parameters.Xf, parameters.numlinks, gamval, R, RotMat, 0, 0, false);
%value1 = [];
[xNew, ~, ~] = HeelCollisionJacobian(xF,parameters.Mf,parameters.Jf,parameters.Xf,parameters.numlinks,R);
xNew(1:4) = wrapToPi(xNew(1:4));
eqc1 = x0 - xNew;

eqc = [eqc1; eqc2; eqc3];
end