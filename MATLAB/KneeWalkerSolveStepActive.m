function [ t,X,steptracker,lockflags,offset,Xend ] = KneeWalkerSolveStepActive(maxsteps,X0,tstep,tspan,R,gval,gamval,nolockdat,~,trajS,controller)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

    %Model positions are in terms of Location
    %Rotation matrix to bring things in terms of flat ground
    tmp = makehgtform('zrotate',gamval); %transform matrix
    RotMat = tmp(1:3,1:3); %take upper left 3x3

    
    nolockdynam = @(t,X) ChainDynamics(t,X,nolockdat.Mf,nolockdat.Cf,nolockdat.Tf,nolockdat.numlinks,trajS,gval,gamval,controller);
    nolockcollideevent = @(t,X) CheckCollision(t,X,nolockdat.Xf,nolockdat.numlinks,gamval,R,RotMat,3,2,false);
    noloptions = odeset('Events',nolockcollideevent);

    %not used
    %twolockdynam = @(t,X) ChainDynamics(t,X,twolockdat.Mf,twolockdat.Cf,twolockdat.Tf,Text,gval,gamval);
    %twolockcollideevent = @(t,X) CheckCollision(t,X,twolockdat.Xf,twolockdat.numlinks,gamval,R,RotMat,3,2);
    %twooptions = odeset('Events',twolockcollideevent);
    
    if controller.type == 'comptorque'
        %X0 = [X0;controller.qrinit;zeros(4,1)]; %initialize qd_ref 
        %X0(5) = 1.5*X0(5);
    end
    [t1, X1, te, Xe, ~] = ode45(nolockdynam,tspan,X0,noloptions);

    offset = [];
    t = [];
    X = [];
    lockflags = [];
    numsteps = 0;
    tspanlast = tspan;
    tesum = 0;
    while(numsteps < maxsteps)
        offset = [offset,[zeros(1,length(t1)-1);R.*X1(1:end-1,1)';zeros(1,length(t1)-1)]];
        t = [t;t1(1:end-1)+tesum];
        tspanlast = [te:tstep:20];
        tspan = tspanlast - te;
        tesum = tesum + te;
        
        numsteps = numsteps + 1;
        lockflags = [lockflags,0*ones(1,length(t1)-1)];
        X = [X;[X1(1:end-1,:),zeros(size(X1,1)-1,size(X,2)-size(X1,2))]];
        if ~(isempty(Xe))
        %[Xnew, steplength] = HeelCollision(Xe,onelockdat.Mf,onelockdat.Mf,twolockdat.Xf,twolockdat.numlinks,R);%,twolockdat.Hf,onelockdat.Hf);
            [Xnew, steplength,F] = HeelCollisionJacobian(Xe',nolockdat.Mf,nolockdat.Jf,nolockdat.Xf,nolockdat.numlinks,R);
            Xend = Xnew;
            steptracker(length(t)+1) = steplength;
            %X1 = repelem(X1,1,[1 2 1 2]); %knee and hip share position and velocity
            if controller.type == 'comptorque'
                %Xnew = [Xnew;controller.qrinit;zeros(4,1)]; %initialize qd_ref    
                Xnew = [Xnew;X0(2*nolockdat.numlinks+1:end)]; %initialize qd_ref    
            end
            [t1, X1, te, Xe, ~] = ode45(nolockdynam,tspan,Xnew,noloptions);       
        else
            steptracker(length(t)+1) = 0;
            Xend = X1(end,:);
        end
    end
end

