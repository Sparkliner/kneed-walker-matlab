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
maxsteps = 2;
R = nolockdat.ParameterCell{1}(1);
gval = 9.81; %m/s^2
gamval = 0.00;%rad %.0456

%step length = 0.65 m;
stepLen = 0.65;
%leg length = sum of shank + thigh
legLen = sum(nolockdat.ParameterCell{2}(1:2));
%use isosceles triangle
stanceAngle = asin(0.5*stepLen/legLen);
%speed
speed = 0.6*1.4;

xmin = -2;
xmax = 4;
ymin = -3;
ymax = 3;


%optimization using Matthew Kelley's MATLAB OptimTraj library and adapted
%from his Simple Walker example


trajSoln = load('simplifiedOptimalSolution9.mat');
trajSoln = trajSoln.soln;

tmax = max(trajSoln.grid.time);
f = @(t) subsref(trajSoln.interp.state(t),struct('type','()','subs',{{1:4}}));
d = @(t) subsref(trajSoln.interp.state(t),struct('type','()','subs',{{5:8}}));


problem.func.dynamics = @(t,x,u)(ChainDynamics(t,x,nolockdat.Mf,nolockdat.Cf,nolockdat.Tf,nolockdat.numlinks,struct('tmax',tmax,'f',f,'d',d),gval,gamval,struct('type',{'comptorque'}),u));

problem.func.pathObj = @(t,x,u)(costFun(x,gval,gamval,nolockdat,u));

problem.func.pathCst = @(t,x,u)(physicalConstraint(t, x, nolockdat, gamval, R));

problem.func.bndCst = @(t0,x0,tF,xF)(boundaryConditions(xF,x0,nolockdat,R,stanceAngle,gamval));

stepTime = stepLen/speed;

t0 = 0;

tF = stepTime; %step time;

problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;
problem.bounds.finalTime.low = 0.5*tF;
problem.bounds.finalTime.upp = 2*tF;

problem.bounds.state.low = [-5*pi/16; -5*pi/16; -3*pi/2; -3*pi/2; -inf(4,1);-inf(4,1)];
problem.bounds.state.upp = [5*pi/16; 5*pi/16; -11*pi/16; -11*pi/16; inf(4,1);inf(4,1)];

%should start (and end) with compass gait stance
%problem.bounds.initialState.low = [-stanceAngle; -stanceAngle; stanceAngle-pi; stanceAngle-pi; -inf(4,1)];
problem.bounds.initialState.low = [-stanceAngle*3; -stanceAngle*3; -(pi-stanceAngle); -(pi-stanceAngle); -inf(4,1);-inf(4,1)];
%problem.bounds.initialState.upp = [-stanceAngle; -stanceAngle; stanceAngle-pi; stanceAngle-pi; inf(4,1)];
problem.bounds.initialState.upp = [-stanceAngle; -stanceAngle; (stanceAngle*3-pi); (stanceAngle*3-pi)*0.99; inf(4,1);inf(4,1)];

%assume linear interpolation for first guess
% problem.guess.time = [t0,tF];
% 
% stepRate1 = (2*stanceAngle)/(tF-t0);
% stepRate2 = -2*stepRate1; %swing leg goes counterclockwise
%  x0 = [-stanceAngle; -stanceAngle; stanceAngle-pi; stanceAngle-pi; stepRate1; stepRate1; stepRate2; stepRate2];
%  xF = [stanceAngle; stanceAngle; -(stanceAngle+pi); -(stanceAngle+pi); stepRate1; stepRate1; stepRate2; stepRate2];
% problem.guess.state = [x0, xF];
% 
% problem.guess.control = zeros(3,2);

%Use previous guess from simplified problem
prevSoln = load('simplifiedOptimalSolutionNoMassNoC.mat');
prevSoln = prevSoln.soln;

problem.guess.time = prevSoln(end).grid.time;
% problem.guess.state = [prevSoln(end).grid.state;zeros(4,size(prevSoln(end).grid.control,2))];
% problem.guess.control = [zeros(1,size(prevSoln(end).grid.control,2));prevSoln(end).grid.control;50*2*pi*ones(4,size(prevSoln(end).grid.control,2))];
problem.guess.state = prevSoln(end).grid.state;
problem.guess.control = prevSoln(end).grid.control;

%use Hermite-Simpson method
% First iteration: get a more reasonable guess
problem.options(1).nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-4,...
    'MaxFunEvals',4e5);   %options for fmincon
problem.options(1).verbose = 3; % How much to print out?
problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
problem.options(1).hermiteSimpson.nSegment = 10;  %method-specific options


% Second iteration: refine guess to get precise soln
% problem.options(2).nlpOpt = optimset(...
%     'Display','iter',...   % {'iter','final','off'}
%     'TolFun',1e-4,...
%     'MaxFunEvals',6e5,...
%     'MaxIter',2000);   %options for fmincon
% problem.options(2).verbose = 3; % How much to print out?
% problem.options(2).method = 'hermiteSimpson'; % Select the transcription method
% problem.options(2).hermiteSimpson.nSegment = 8;  %method-specific options

%Solve
%soln = optimTraj(problem);

%Alternately, load previous
%soln = load('simplifiedOptimalSolution.mat');
soln = prevSoln;

% Transcription Grid points:
t = soln(end).grid.time;
u = soln(end).grid.control;

X = soln(end).grid.state;

order = 10;
for I = 1:4
    xvals = X(I,:);
    poly_p(I,:) = polyfit(t,xvals,order);
    poly_v(I,:) = poly_p(I,1:order).*(order:-1:1);
    poly_a(I,:) = poly_v(I,1:order-1).*(order-1:-1:1);
    figure;
    hold on;
    plot(t,xvals);
    plot(t,polyval(poly_p(I,:),t));
end

% Interpolated solution:
t1 = linspace(t(1),t(end),74);
X1 = soln(end).interp.state(t1);
uInt = soln(end).interp.control(t1);

X1 = X1(1:2*nolockdat.numlinks,:);

%Prepare for animation
offset = [];
t = [];
X = [];
lockflags = [];

Xe = X1(:,end);

offset = [offset,[zeros(1,length(t1)-1);R.*X1(1,1:end-1);zeros(1,length(t1)-1)]];
t = [t;t1(1:end-1)];

lockflags = [lockflags,0*ones(1,length(t1)-1)];
X = [X,[X1(:,1:end-1);zeros(size(X,1)-size(X1,1),size(X1,2)-1)]];
[Xnew, steplength,F] = HeelCollisionJacobian(Xe,nolockdat.Mf,nolockdat.Jf,nolockdat.Xf,nolockdat.numlinks,R);
steptracker(length(t)+1) = steplength;

AnimateWalker(xmin,xmax,ymin,ymax,gamval,t,X',nolockdat,onelockdat,twolockdat,steptracker,lockflags,offset);