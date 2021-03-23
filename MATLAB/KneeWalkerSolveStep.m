function [ t,X,steptracker,lockflags,offset,Xend ] = KneeWalkerSolveStep(maxsteps,X0,tstep,tspan,R,gval,gamval,onelockdat,twolockdat)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    %Model positions are in terms of Location
    %Rotation matrix to bring things in terms of flat ground
    tmp = makehgtform('zrotate',gamval); %transform matrix
    RotMat = tmp(1:3,1:3); %take upper left 3x3

    
    onelockdynam = @(t,X) ChainDynamics(t,X,onelockdat.Mf,onelockdat.Cf,onelockdat.Tf,gval,gamval);
    onelockcollideevent = @(t,X) CheckCollision(t,X,onelockdat.Xf,onelockdat.numlinks,gamval,R,RotMat,3,2);
    oneoptions = odeset('Events',onelockcollideevent);

    twolockdynam = @(t,X) ChainDynamics(t,X,twolockdat.Mf,twolockdat.Cf,twolockdat.Tf,gval,gamval);
    twolockcollideevent = @(t,X) CheckCollision(t,X,twolockdat.Xf,twolockdat.numlinks,gamval,R,RotMat,3,2);
    twooptions = odeset('Events',twolockcollideevent);

    [t1, X1, te, Xe, ie] = ode45(onelockdynam,tspan,X0,oneoptions);

    offset = [];
    t = [];
    X = [];
    lockflags = [];
    numsteps = 0;

    while(numsteps < maxsteps)
        offset = [offset,[zeros(1,length(t1)-1);R.*X1(1:end-1,1)';zeros(1,length(t1)-1)]];
        t = [t;t1(1:end-1)];
        tspan = [te:tstep:20];
        if (ie == 2) %knee lock event
            %numsteps = numsteps + 1; %for testing only
            %position of stance foot using flat RF
            lockflags = [lockflags,1*ones(1,length(t1)-1)];
            Xnew = KneeCollision(Xe,onelockdat.Mf,twolockdat.Mf,onelockdat.numlinks);

            X = [X;[X1(1:end-1,:),zeros(size(X1,1)-1,size(X,2)-size(X1,2))]];
            [t1, X1, te, Xe, ie] = ode45(twolockdynam,tspan,Xnew,twooptions);
        elseif (ie == 1) %foot strike event
            numsteps = numsteps + 1;
            lockflags = [lockflags,2*ones(1,length(t1)-1)];
            %[Xnew, steplength] = HeelCollision(Xe,onelockdat.Mf,onelockdat.Mf,twolockdat.Xf,twolockdat.numlinks,R);%,twolockdat.Hf,onelockdat.Hf);
            [Xnew, steplength] = HeelCollision(Xe,twolockdat.Mpf,onelockdat.Mf,twolockdat.Xf,twolockdat.numlinks,R);
            Xend = Xnew;
            steptracker(length(t)+1) = steplength;

            X1 = repelem(X1,1,[1 2 1 2]); %knee and hip share position and velocity
            X = [X;[X1(1:end-1,:),zeros(size(X1,1)-1,size(X,2)-size(X1,2))]];
            [t1, X1, te, Xe, ie] = ode45(onelockdynam,tspan,Xnew,oneoptions);
        end
    end


end

