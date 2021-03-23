clear;
close all;
%% Generate new files
choice = questdlg('Load new parameters (overwrites existing) or use existing?','Choose model data',...
    'Load new','Use existing', 'Cancel',...
    'Use existing');
switch choice
    case 'Load new'
        ChainBuilder('ActiveWalkerData.txt',[],true);
        ChainBuilder('ActiveWalkerData.txt',[2],true); % stance knee locked
        ChainBuilder('ActiveWalkerData.txt',[2, 5],true); % both knees locked
        %ChainBuilder('ActiveWalkerData.txt',[5],true); % post-collision:old swing knee locked
    case 'Use existing'
        %do nothing
    case 'Cancel'
        return;
end
nolockdat = load('EOMs_ActiveWalkerData_unlocked.mat'); %load functions of theta
onelockdat = load('EOMs_ActiveWalkerData_1_locked.mat');
twolockdat = load('EOMs_ActiveWalkerData_0_3_locked.mat');
%collislockdat = load('EOMs_ActiveWalkerData_4_locked.mat');
%% Numerical Simulation
maxsteps = 3;
R = nolockdat.ParameterCell{1}(1);
gval = 9.81; %m/s^2
gamval = 0;%rad %.0456

%X0 = [-0.2, 3*pi/4, 5*pi/4,1.5,-2,-3]'; %radius = 0.2
%X0 = [-0.2,-0.2, 3*pi/4, 5*pi/4,1.5,1.5,-2,-2]';
tstep = 0.05;
tspan = [0:tstep:20];

xmin = -2;
xmax = 4;
ymin = -3;
ymax = 3;

%syms t;
% %traj = repelem([(pi/4)*sin(pi*t/4)],[4])' + [.1, .2, .3, .4]';
% traj = t*zeros(4,1);
% trajd = diff(traj,t);
% trajdd = diff(trajd,t);
% trajS.f = matlabFunction(traj,'Vars',[t]);
% trajS.d = matlabFunction(trajd,'Vars',[t]);
% trajS.dd = matlabFunction(trajdd,'Vars',[t]);
% %Text = @(t,X) [0,0,0,0]';

controller.type = 'comptorque';
%controller.qrinit = zeros(4,1);

% load('simplifiedControlSolution2.mat');
% trajS.tmax = max(soln.grid.time);
% trajS.f = @(t) subsref(soln.interp.state(t),struct('type','()','subs',{{1:4}}));
% trajS.d = @(t) subsref(soln.interp.state(t),struct('type','()','subs',{{5:8}}));
% trajS.dd = @(t) subsref(soln.interp.control(t),struct('type','()','subs',{{1:4}}));
% 
% 
% 
% dt = diff(soln.grid.time);
% v = soln.grid.state(5:8,:);
% dv = diff(v,1,2);
% a = dv./(repmat(dt,[4,1]));
% a(:,21) = a(:,20);
% 
% soln.grid.control(1:4,:) = a;

%trajS.om = @(t) subsref(soln.interp.control(t),struct('type','()','subs',{{5:8}}));
% trajS.om = mean(soln.grid.control(5:8,:),2);
% 
% X0 = soln.interp.state(0);
load('simplifiedOptimalSolution9.mat');
load('trajectorypoly.mat');
trajS.tmax = max(soln.grid.time);
trajS.f = @(t) arrayfun(@(I) polyval(poly_p(I,:),t),[1:4]');
trajS.d = @(t) arrayfun(@(I) polyval(poly_v(I,:),t),[1:4]');
trajS.dd = @(t) arrayfun(@(I) polyval(poly_a(I,:),t),[1:4]');

X0 = [trajS.f(0);trajS.d(0);zeros(4,1)];

[t,X,steptracker,lockflags,offset,Xend] = KneeWalkerSolveStepActive(maxsteps,X0,tstep,tspan,R,gval,gamval,nolockdat,twolockdat,trajS,controller);

AnimateWalker(xmin,xmax,ymin,ymax,gamval,t,X,nolockdat,onelockdat,twolockdat,steptracker,lockflags,offset);
