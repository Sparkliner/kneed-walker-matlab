function [ value, isterminal, direction ] = CheckCollision( ~, X, Xf, numlinks, gamval, R, RotMat, swingknee, swinghip)
%swingknee and swinghip are indices of knee joint and hip joint angles
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

value(1,1) = footPosition(1) + footPosition(2)*tan(gamval);

isterminal(1,1) = 1;

direction(1,1) = -1;

if (numlinks > 2)
    %KneeCollision - To check for knee locking
    value(2,1) = X(swingknee) - X(swinghip); %typically 4 and 3

    isterminal(2,1) = 1;

    direction(2,1) = -1;
end

end

