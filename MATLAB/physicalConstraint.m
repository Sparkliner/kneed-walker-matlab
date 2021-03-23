function [ineqc, eqc] = physicalConstraint(t, X, parameters, gamval, R)
X = X(1:parameters.numlinks*2,:);
numlinks = parameters.numlinks;
Xf = parameters.Xf;

footPosition = zeros(3,size(X,2));

for I = 1:size(X,2)
        JointAngleCell = num2cell(X(1:numlinks,I));

        %position of bottom of striking foot relative to foot center in tilted RF
        offset = [-R;0;0];

        %calculate using function
        jointPositions = (Xf(JointAngleCell{:}) + offset);

        footPosition(:,I) = jointPositions(:,end);
end

x1 = min(footPosition(2,:));
x2 = max(footPosition(2,:));

stepheight = 0.03;
cst = stepheight*0.5*(1-cos(2*pi*(footPosition(2,:)-x1)/(x2-x1))) - footPosition(1,:);

%[cst, ~, ~] = CheckCollision( 0, X, parameters.Xf, parameters.numlinks, gamval, R, RotMat, 0, 0, false);

eqc = [];
%ineqc = []; %use no constraints at first
%use only floor constraint
ineqc = [cst];
ineqc = [ineqc;X(2,:)-X(1,:); X(3,:)-X(4,:)]; %second and third rows prevent knee hyperextension
end