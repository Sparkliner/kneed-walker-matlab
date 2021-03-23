function [ value, isterminal, direction ] = FootCollision( ~, X, Xf, numlinks, gamval, R, RotMat)
%FootCollision - To check if foot has touched the ground

JointAngleCell = num2cell(X(1:numlinks));

%position of bottom of striking foot relative to foot center in tilted RF
offset = [-R;0;0];
    
%calculate using function and put in flat RF
jointPositions = (Xf(JointAngleCell{:}) + offset);
jointPositions = RotMat*jointPositions;

footPosition = jointPositions(:,end);

%check if foot position crosses the equation of the ground line
%(decreasing)

value = footPosition(1) + footPosition(2)*tan(gamval);

isterminal = 1;

direction = -1;

end

