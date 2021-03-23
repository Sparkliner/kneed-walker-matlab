function [ ] = AnimateWalker(xmin,xmax,ymin,ymax,gamval,t,X,onelockdat,twolockdat,steptracker,lockflags,offset)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

    %Model positions are in terms of Location
    %Rotation matrix to bring things in terms of flat ground
    tmp = makehgtform('zrotate',gamval); %transform matrix
    RotMat = tmp(1:3,1:3); %take upper left 3x3

    ygroundleft = -xmin*tan(gamval);
    ygroundright = -xmax*tan(gamval);

    AnimFigure = figure('OuterPosition',[0,0,1000,1000]);
    AnimAxes = subplot(2,2,1);
    axis([xmin xmax ymin ymax]);
    GroundAxes = axes;
    AnimFigure.CurrentAxes = GroundAxes;
    plot(GroundAxes,[xmin, xmax],[ygroundleft,ygroundright],'--r');
    axis([xmin xmax ymin ymax]);
    set(GroundAxes,'Position',get(AnimAxes,'position'),'Color','none','XColor','none','YColor','none');
    AnimFigure.CurrentAxes = AnimAxes;

    R = onelockdat.ParameterCell{1}(1);
    
    stepvector = zeros(3,1);
    currentjointposandvel = zeros(2,3); %stance foot, hip, swing knee
    EnergyFig = figure;
    for I = 1:length(t)
        figure(AnimFigure);
        AnimFigure.CurrentAxes = AnimAxes;
        hold on;
         cla(AnimAxes); %clear axes for next frame

        stepvector = stepvector +[0;steptracker(I);0]; 

        if (lockflags(I) == 1) %knee is unlocked  
            JointAngleCell = num2cell(X(I,1:onelockdat.numlinks));

            %calculate using function and put in flat RF
            jointPositions = [[R;0;0],onelockdat.Xf(JointAngleCell{:})];

            currentjointposandvel(:,:,I) = [X(I,1:onelockdat.numlinks);X(I,onelockdat.numlinks+1:end)];

        else %knee is locked
            JointAngleCell = num2cell(X(I,1:twolockdat.numlinks));

            %calculate using function and put in flat RF
            jointPositions = [[R;0;0],twolockdat.Xf(JointAngleCell{:})];

            currentjointposandvel(:,:,I) = [X(I,1:onelockdat.numlinks);X(I,onelockdat.numlinks+1:end)];
            %currentjointposandvel(:,3) = currentjointposandvel(:,2);
        end
        %jointPositions(:,1)
        %jointPositions(:,end)
        jointPositions = RotMat*(jointPositions + stepvector + offset(:,I));

        %draw circles at feet
        rectangle('Position',[jointPositions(2,1) - R, jointPositions(1,1) - R, 2*R, 2*R],'Curvature',[1 1]);
        rectangle('Position',[jointPositions(2,end) - R, jointPositions(1,end) - R, 2*R, 2*R],'Curvature',[1 1]);
        plot(jointPositions(2,:),jointPositions(1,:)); %note x is "up" and y is "right"
        %disp(I)

        currentjointposandvel(1,1,I) = wrapToPi(currentjointposandvel(1,1,I));
        currentjointposandvel(1,2:3,I) = wrapTo2Pi(currentjointposandvel(1,2:3,I));

        %plot phase portraits
        subplot(2,2,2);
        hold on;
        plot(squeeze(currentjointposandvel(1,1,1:I)),squeeze(currentjointposandvel(2,1,1:I)),'-xb');
        xlabel('Joint Angle (rad)');
        ylabel('Joint Velocity (rad/s)');
        title('Stance foot phase portrait');

        subplot(2,2,3);
        hold on;
        plot(squeeze(currentjointposandvel(1,2,1:I)),squeeze(currentjointposandvel(2,2,1:I)),'-xb');
        xlabel('Joint Angle (rad)');
        ylabel('Joint Velocity (rad/s)');
        title('Hip phase portrait');


        subplot(2,2,4);
        hold on;
        plot(squeeze(currentjointposandvel(1,3,1:I)),squeeze(currentjointposandvel(2,3,1:I)),'-xb');
        xlabel('Joint Angle (rad)');
        ylabel('Joint Velocity (rad/s)');
        title('Swing knee phase portrait');


        set(groot,'CurrentFigure',EnergyFig);
        hold on;
        plot(t(I),dot(currentjointposandvel(2,:,I),currentjointposandvel(2,:,I)),'.b');

        pause(.01);
    end

end

