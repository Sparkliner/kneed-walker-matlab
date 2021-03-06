function [ ] = AnimateWalker(xmin,xmax,ymin,ymax,gamval,t,X,nolockdat,onelockdat,twolockdat,steptracker,lockflags,offset)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    numlinks = nolockdat.numlinks;
    X = X(:,1:2*numlinks); %strip off excess data

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
    currentjointposandvel = zeros(2,numlinks); %stance foot, hip, swing knee
    EnergyFig = figure;
    
    filename = 'animation.gif';
    
    for I = 1:length(t)
        figure(AnimFigure);
        AnimFigure.CurrentAxes = AnimAxes;
        hold on;
         cla(AnimAxes); %clear axes for next frame

        stepvector = stepvector +[0;steptracker(I);0]; 

        JointAngleCell = num2cell(X(I,1:numlinks));

        %calculate using function and put in flat RF
        jointPositions = nolockdat.Xf(JointAngleCell{:});
%         if (lockflags(I) == 1) %knee is unlocked  
%             JointAngleCell = num2cell(X(I,1:onelockdat.numlinks));
% 
%             %calculate using function and put in flat RF
%             jointPositions = onelockdat.Xf(JointAngleCell{:});
% 
%             %currentjointposandvel(:,:,I) = [X(I,1:onelockdat.numlinks);X(I,onelockdat.numlinks+1:end)];
%         elseif (lockflags(I) == 2)%knee is locked
%             JointAngleCell = num2cell(X(I,1:twolockdat.numlinks));
% 
%             %calculate using function and put in flat RF
%             jointPositions = twolockdat.Xf(JointAngleCell{:});
% 
%             %currentjointposandvel(:,:,I) = [X(I,1:twolockdat.numlinks);X(I,twolockdat.numlinks+1:end)];
%             %currentjointposandvel(:,3) = currentjointposandvel(:,2);
%         elseif (lockflags(I) == 0)%nothing is locked
%             JointAngleCell = num2cell(X(I,1:nolockdat.numlinks));
% 
%             %calculate using function and put in flat RF
%             jointPositions = nolockdat.Xf(JointAngleCell{:});
% 
%             %currentjointposandvel(:,:,I) = [X(I,1:nolockdat.numlinks);X(I,nolockdat.numlinks+1:end)];
%             %currentjointposandvel(:,3) = currentjointposandvel(:,2);
%         end
        currentjointposandvel(:,:,I) = [X(I,1:numlinks);X(I,numlinks+1:end)];
        
        %jointPositions(:,1)
        %jointPositions(:,end)
        jointPositions = [[R;0;0],jointPositions];
        footPositions = [jointPositions(:,1),jointPositions(:,end)];
        jointPositions(:,1) = jointPositions(:,1) - [R*cos(JointAngleCell{1});R*sin(JointAngleCell{1});0];
        jointPositions(:,end) = jointPositions(:,end) + [R*cos(JointAngleCell{end});R*sin(JointAngleCell{end});0];
        
        jointPositions = RotMat*(jointPositions + stepvector + offset(:,I));
        footPositions = RotMat*(footPositions + stepvector + offset(:,I));
        
        %draw circles at feet
        rectangle('Position',[footPositions(2,1) - R, footPositions(1,1) - R, 2*R, 2*R],'Curvature',[1 1],'LineStyle','-');
        rectangle('Position',[footPositions(2,end) - R, footPositions(1,end) - R, 2*R, 2*R],'Curvature',[1 1],'LineStyle','-');
        plot(jointPositions(2,:),jointPositions(1,:)); %note x is "up" and y is "right"
        %disp(I)

        currentjointposandvel(1,1,I) = wrapToPi(currentjointposandvel(1,1,I));
        currentjointposandvel(1,3:4,I) = wrapTo2Pi(currentjointposandvel(1,3:4,I));

        %plot phase portraits
        subplot(2,2,2);
        hold on;
        plot(squeeze(currentjointposandvel(1,1,1:I)),squeeze(currentjointposandvel(2,1,1:I)),'-xb');
        xlabel('Joint Angle (rad)');
        ylabel('Joint Velocity (rad/s)');
        title('Stance foot phase portrait');
        
        frame = getframe(1);
        im{I} = frame2im(frame);
        
        [A,map] = rgb2ind(im{I},256);
        
        if I ==1
            imwrite(A,map,filename,'gif','LoopCount',Inf,'DelayTime',.01);
        else
            imwrite(A,map,filename,'gif','WriteMode','append','DelayTime',.01);
        end

        subplot(2,2,3);
        hold on;
        plot(squeeze(currentjointposandvel(1,3,1:I)),squeeze(currentjointposandvel(2,3,1:I)),'-xb');
        xlabel('Joint Angle (rad)');
        ylabel('Joint Velocity (rad/s)');
        title('Hip phase portrait');


        subplot(2,2,4);
        hold on;
        plot(squeeze(currentjointposandvel(1,4,1:I)),squeeze(currentjointposandvel(2,4,1:I)),'-xb');
        xlabel('Joint Angle (rad)');
        ylabel('Joint Velocity (rad/s)');
        title('Swing knee phase portrait');


        set(groot,'CurrentFigure',EnergyFig);
        hold on;
        plot(t(I),dot(currentjointposandvel(2,:,I),currentjointposandvel(2,:,I)),'.b');

        pause(.01);
        %disp(I)
    end
    
    

end

