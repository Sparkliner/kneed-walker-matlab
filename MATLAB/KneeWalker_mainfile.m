clear;
close all;
%% Generate new files
choice = questdlg('Load new parameters (overwrites existing) or use existing?','Choose model data',...
    'Load new','Use existing', 'Cancel',...
    'Use existing');
switch choice
    case 'Load new'
        ChainBuilder('WalkerData.txt',[],true);
        ChainBuilder('WalkerData.txt',[2],true); % stance knee locked
        ChainBuilder('WalkerData.txt',[2, 5],true); % both knees locked
        %ChainBuilder('WalkerData.txt',[5],true); % post-collision:old swing knee locked
    case 'Use existing'
        %do nothing
    case 'Cancel'
        return;
end
nolockdat = load('EOMs_WalkerData_unlocked.mat'); %load functions of theta
onelockdat = load('EOMs_WalkerData_1_locked.mat');
twolockdat = load('EOMs_WalkerData_0_3_locked.mat');
%collislockdat = load('EOMs_WalkerData_4_locked.mat');
%% Numerical Simulation
maxsteps = 3;
R = onelockdat.ParameterCell{1}(1);
gval = 9.81; %m/s^2
gamval = 0.05;%rad %.0456

%X0 = [-0.2, 3*pi/4, 5*pi/4,1.5,-2,-3]; %radius = 0.2
X0 = [-0.2, 3*pi/4, 5*pi/4,1.5,-2,-3];
tstep = 0.01;
tspan = [0:tstep:20];

xmin = -2;
xmax = 4;
ymin = -3;
ymax = 3;

% %Model positions are in terms of Location
% %Rotation matrix to bring things in terms of flat ground
% tmp = makehgtform('zrotate',gamval); %transform matrix
% RotMat = tmp(1:3,1:3); %take upper left 3x3

% iterations = 10;
% delta = 0.01;
% choice = questdlg('Solve for initial condition?','Initial conditions',...
%     'Yes','No',...
%     'No');
% switch choice
%     case 'Yes'
%         for I = 1:iterations
%             %find gradient of stride function
%             [~,~,~,~,~,Xend0] = KneeWalkerSolveStep(maxsteps,X0,tstep,tspan,R,gval,gamval,onelockdat,twolockdat);
%             Xend0 = Xend0([1,4,5]);
%             X0grad = X0;
%             gamvalgrad = gamval;
%             Xendgrad = zeros(length(Xend0),3);
%             for J = 1:3
%                 switch J
%                     case 1
%                         X0grad(4) = X0(4) + delta;
%                     case 2
%                         X0grad(5) = X0(5) + delta;
%                         X0grad(6) = X0(6) + delta;
%                     case 3
%                         gamvalgrad = gamval + delta;
%                 end
%                 
%                 [~,~,~,~,~,Xend1] = KneeWalkerSolveStep(maxsteps,X0grad,tstep,tspan,R,gval,gamvalgrad,onelockdat,twolockdat);
%                 Xend1 = Xend1([1,4,5]);
%                 Xendgrad(:,J) = (Xend1'-Xend0')/delta;
%             end
%             varchange = Xendgrad\(X0([1,4,5])'-Xend0');
%             X0(4) = X0(4) + varchange(1);
%             X0(5) = X0(5) + varchange(2);
%             X0(6) = X0(6) + varchange(2);
%             gamval = gamval + varchange(3);
%         end
%     case 'No'
%         %do nothing
% end

[t,X,steptracker,lockflags,offset,Xend] = KneeWalkerSolveStep(maxsteps,X0,tstep,tspan,R,gval,gamval,onelockdat,twolockdat);

AnimateWalker(xmin,xmax,ymin,ymax,gamval,t,X,onelockdat,twolockdat,steptracker,lockflags,offset);
