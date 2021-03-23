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
maxsteps = 2;
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

Text = @(t,X) 0;       

[t,X,steptracker,lockflags,offset,Xend] = KneeWalkerSolveStepPassive(maxsteps,X0,tstep,tspan,R,gval,gamval,onelockdat,twolockdat,Text);

AnimateWalker(xmin,xmax,ymin,ymax,gamval,t,X,nolockdat,onelockdat,twolockdat,steptracker,lockflags,offset);
