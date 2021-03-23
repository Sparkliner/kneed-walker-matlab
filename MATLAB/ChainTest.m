clear;
close all;
%% Generate new files
choice = questdlg('Load new parameters or use existing?','Choose model data',...
    'Load new','Use existing', 'Cancel',...
    'Use existing');
switch choice
    case 'Load new'
        ChainBuilder('TestData.txt',[],true);
    case 'Use existing'
        %do nothing
    case 'Cancel'
        return;
end
nolockdat = load('EOMs_TestData_unlocked.mat'); %load functions of theta
%% Numerical Simulation
maxsteps = 5;
R = nolockdat.ParameterCell{1}(1);
gval = 9.81; %m/s^2
gamval = 0;%rad %.02

xmin = -2;
xmax = 2;
ymin = -2;
ymax = 2;

%Model positions are in terms of Location
%Rotation matrix to bring things in terms of flat ground
tmp = makehgtform('zrotate',gamval); %transform matrix
RotMat = tmp(1:3,1:3); %take upper left 3x3

nolockdynam = @(t,X) ChainDynamics(t,X,nolockdat.Mf,nolockdat.Cf,nolockdat.Tf,gval,gamval);

X0 = [(0.75*pi)*ones(1, nolockdat.numlinks),zeros(1, nolockdat.numlinks)];
tstep = 0.01;
tspan = [0:tstep:25];

[t, X] = ode45(nolockdynam,tspan,X0);

ygroundleft = -xmin*tan(gamval);
ygroundright = -xmax*tan(gamval);

AnimFigure = figure;
AnimAxes = axes;
axis([xmin xmax ymin ymax]);
GroundAxes = axes;
AnimFigure.CurrentAxes = GroundAxes;
plot(GroundAxes,[xmin, xmax],[ygroundleft,ygroundright],'--r');
axis([xmin xmax ymin ymax]);
set(GroundAxes,'Position',get(AnimAxes,'position'),'Color','none','XColor','none','YColor','none');
AnimFigure.CurrentAxes = AnimAxes;

for I = 1:length(t)
    hold on;
    cla(AnimAxes); %clear axes for next frame
      
    
    JointAngleCell = num2cell(X(I,1:nolockdat.numlinks));

    offset = [0;R*JointAngleCell{1}(1);0];
    
    %calculate using function and put in flat RF
    jointPositions = [[R;0;0],nolockdat.Xf(JointAngleCell{:})];
    %jointPositions(:,1)
    %jointPositions(:,end)
    jointPositions = RotMat*(jointPositions + offset);
    
    %draw circles at feet
    rectangle('Position',[jointPositions(2,1) - R, jointPositions(1,1) - R, 2*R, 2*R],'Curvature',[1 1]);
    rectangle('Position',[jointPositions(2,end) - R, jointPositions(1,end) - R, 2*R, 2*R],'Curvature',[1 1]);
    plot(jointPositions(2,:),jointPositions(1,:)); %note x is "up" and y is "right"
    %disp(I)
    pause(.01);
end