function [ t,X,steptracker,lockflags,offset,Xend ] = KneeWalkerSolveStepActive(maxsteps,X0,tstep,tspan,R,gval,gamval,nolockdat,~,Text)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    %Model positions are in terms of Location
    %Rotation matrix to bring things in terms of flat ground
    tmp = makehgtform('zrotate',gamval); %transform matrix
    RotMat = tmp(1:3,1:3); %take upper left 3x3

    
    onelockdynam = @(t,X) ChainDynamics(t,X,nolockdat.Mf,nolockdat.Cf,nolockdat.Tf,Text,gval,gamval);
    onelockcollideevent = @(t,X) CheckCollision(t,X,nolockdat.Xf,nolockdat.numlinks,gamval,R,RotMat,3,2,false);
    oneoptions = odeset('Events',onelockcollideevent);

    %not used
    %twolockdynam = @(t,X) ChainDynamics(t,X,twolockdat.Mf,twolockdat.Cf,twolockdat.Tf,Text,gval,gamval);
    %twolockcollideevent = @(t,X) CheckCollision(t,X,twolockdat.Xf,twolockdat.numlinks,gamval,R,RotMat,3,2);
    %twooptions = odeset('Events',twolockcollideevent);

    [t1, X1, te, Xe, ~] = ode45(onelockdynam,tspan,X0,oneoptions);

    offset = [];
    t = [];
    X = [];
    lockflags = [];
    numsteps = 0;

    while(numsteps < maxsteps)
        offset = [offset,[zeros(1,length(t1)-1);R.*X1(1:end-1,1)';zeros(1,length(t1)-1)]];
        t = [t;t1(1:end-1)];
        tspan = [te:tstep:20];
        
        numsteps = numsteps + 1;
        lockflags = [lockflags,0*ones(1,length(t1)-1)];
        %[Xnew, steplength] = HeelCollision(Xe,onelockdat.Mf,onelockdat.Mf,twolockdat.Xf,twolockdat.numlinks,R);%,twolockdat.Hf,onelockdat.Hf);
        [Xnew, steplength,F] = HeelCollisionJacobian(Xe,nolockdat.Mf,nolockdat.Jf,nolockdat.Xf,nolockdat.numlinks,R);
        Xend = Xnew;
        steptracker(length(t)+1) = steplength;
        %X1 = repelem(X1,1,[1 2 1 2]); %knee and hip share position and velocity
        X = [X;[X1(1:end-1,:),zeros(size(X1,1)-1,size(X,2)-size(X1,2))]];
        [t1, X1, te, Xe, ~] = ode45(onelockdynam,tspan,Xnew,oneoptions);       
    end
end

