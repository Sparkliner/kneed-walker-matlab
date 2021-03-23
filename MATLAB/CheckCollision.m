function [ value, isterminal, direction ] = CheckCollision( ~, X, Xf, numlinks, gamval, R, RotMat, swingknee, swinghip, checkKnee)
%swingknee and swinghip are indices of knee joint and hip joint angles
%FootCollision - To check if foot has touched the ground
value = zeros(1+(checkKnee),size(X,2));
isterminal = value;
direction = value;
    for I = 1:size(X,2)
        JointAngleCell = num2cell(X(1:numlinks,I));

        %position of bottom of striking foot relative to foot center in tilted RF
        offset = [-R;0;0];

        %calculate using function and put in flat RF
        jointPositions = (Xf(JointAngleCell{:}) + offset);
        %jointPositions = RotMat*jointPositions;

        footPosition = jointPositions(:,end);

        %check if foot position crosses the equation of the ground line
        %(decreasing)
        
        value(1,I) = footPosition(1);% + footPosition(2)*tan(gamval);
        
%         if (value <= 0)
%             pause();
%         end

        isterminal(1,I) = 1;

        direction(1,I) = -1;

        if (checkKnee)
            %KneeCollision - To check for knee locking
            value(2,I) = X(swingknee,I) - X(swinghip,I); %typically 4 and 3

            isterminal(2,I) = 1;

            direction(2,I) = -1;
        end
    end
    value = squeeze(value);
    isterminal = squeeze(isterminal);
    direction = squeeze(direction);
end

