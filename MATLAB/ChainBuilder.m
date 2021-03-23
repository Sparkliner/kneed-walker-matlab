function [Mf,Cf,Tf,Jf,Xf,Hf,numlinks,ParameterCell] = ChainBuilder(datafilename, jointsToLock, SaveBoolean)
    %% Initialization
    %close all;
    %clear;
    %clc;


    %% Derive EOM (by McGeer 1990)
    %Create struct of values from file
    fileID = fopen(datafilename);
    ParameterCell = textscan(fileID, '%f %f %f %f %f %f', 'Delimiter', ',',...
        'TreatAsEmpty',{'NA','na'},'CommentStyle','//');
    fclose(fileID);

    numlinks = size(ParameterCell{1},1);
    
    for I=1:length(jointsToLock)
        l1 = ParameterCell{2}(jointsToLock(I)-1);
        l2 = ParameterCell{2}(jointsToLock(I));
        m1 = ParameterCell{3}(jointsToLock(I)-1);
        m2 = ParameterCell{3}(jointsToLock(I));
        mc1 = ParameterCell{4}(jointsToLock(I)-1);
        mc2 = ParameterCell{4}(jointsToLock(I));
        mw1 = ParameterCell{5}(jointsToLock(I)-1);
        mw2 = ParameterCell{5}(jointsToLock(I));
        rg1 = ParameterCell{6}(jointsToLock(I)-1);
        rg2 = ParameterCell{6}(jointsToLock(I));
        %add length
        lnew = l1 + l2;
        %add mass
        mnew = m1 + m2;
        %find new center of mass
        mcnew = (mc1*m1 + (l1+mc2)*m2)/mnew;
        mwnew = (mw1*m1 + (mw2)*m2)/mnew;
        %find new radius of gyration
        Inew = m1*(rg1^2 + (mcnew - mc1)^2 + (mwnew - mw1)^2) + ...
            m2*(rg2^2 + (mcnew - l1 - mc2)^2 + (mwnew - mw2)^2);
        rgnew = sqrt(Inew/mnew);
        ParameterCell{2}(jointsToLock(I)-1) = lnew;
        ParameterCell{3}(jointsToLock(I)-1) = mnew;
        ParameterCell{4}(jointsToLock(I)-1) = mcnew;
        ParameterCell{5}(jointsToLock(I)-1) = mwnew;
        ParameterCell{6}(jointsToLock(I)-1) = rgnew;

        for J=1:6
            ParameterCell{J} = ParameterCell{J}([1:jointsToLock(I)-1, jointsToLock(I)+1:end]);
        end
        numlinks = numlinks - 1;
        jointsToLock = jointsToLock-1; %positions shift up by 1
    end

    g = sym('g', 'Real');
    gam = sym('gam', 'Real');
    x0 = [1; 0; 0];
    y0 = [0; 1; 0];
    z0 = [0; 0; 1];

    %'up' vector
    yg = x0*cos(gam) - y0*sin(gam);
    gvec = -g*yg;

    R = sym('R', 'Real');

    %parameters of links
    l = sym('l',[1 numlinks], 'Real');
    m = sym('m',[1 numlinks], 'Real');
    c = sym('c',[1 numlinks], 'Real');
    w = sym('w',[1 numlinks], 'Real');
    rg = sym('rg',[1 numlinks], 'Real');

    %angle of joints
    th = sym('th',[1 numlinks], 'Real');

    %angular velocity of joints
    th_d = sym('th_d',[1 numlinks], 'Real');
    th_d = [zeros(2,numlinks); th_d];

    %angular accel of joints
    th_dd = sym('th_dd',[1 numlinks], 'Real');
    th_dd = [zeros(2,numlinks); th_dd];

    % x = sym('x',[1 numvars], 'Real');
    % y = sym('y',[1 numvars], 'Real');

    %unit vectors
    x = x0*cos(th) + y0*sin(th);
    y = -x0*sin(th) + y0*cos(th);
    z = sym(repmat(z0,[1 numlinks]));

    r = sym(zeros(3,numlinks));
    lp = sym(zeros(3,numlinks));

    %vec. from joint n to com n
    r = repmat(c,[3 1]).*x;
    r(:,1) = R*x0 + (c(1) - R)*x(:,1) + w(1)*y(:,1);

    %vec. from joint n to n+1
    lp = repmat(l,[3 1]).*x;
    lp(:,1) = R*x0 + (l(1) - R)*x(:,1);

    %velocity of links
    tmp = cumsum(cross(th_d,lp),2); %sum p = 1:n
    tmp4 = [zeros(3,1), tmp(:,1:numlinks-1)]; %sum p = 1:n-1

    v = cross(th_d,r) + tmp4;
    
    %angular momentum about joint n
    tmp = repmat(m,[3 1]).*v; %m*v
    tmp = cumsum(tmp(:,2:numlinks),2,'reverse'); %sum n+1 to N
    tmp = [tmp, zeros(3,1)]; %sum n+1 to N
    tmp = cross(lp,tmp);
    
    hn_minus_hnplus1 = repmat(m.*rg.^2,[3 1]).*th_d + repmat(m,[3 1]).*cross(r,v) + tmp;
    Hsym = cumsum(hn_minus_hnplus1,2,'reverse');
    Hsym = Hsym(3,:);
    
    %accel of links
    tmp = cumsum(cross(th_dd,lp),2); %sum p = 1:n
    tmp2 = [zeros(3,1), tmp(:,1:numlinks-1)]; %sum p = 1:n-1

    tmp = cumsum(lp.*repmat(dot(th_d,th_d),[3 1]),2); %sum p = 1:n
    tmp3 = [zeros(3,1), tmp(:,1:numlinks-1)]; %sum p = 1:n-1
   
    vd = repmat(R*dot(th_d(:,1),th_d(:,1))*x0,[1 numlinks]) + cross(th_dd,r) ...
        - r.*repmat(dot(th_d,th_d),[3 1]) + tmp2 - tmp3;

    tmp = repmat(m,[3 1]).*vd; %m*vd
    tmp = cumsum(tmp(:,2:numlinks),2,'reverse'); %sum n+1 to N
    tmp = [tmp, zeros(3,1)]; %sum n+1 to N
    tmp = cross(lp,tmp);

    %change in angular momentum
    lhs = repmat(m.*rg.^2,[3 1]).*th_dd + repmat(m,[3 1]).*cross(r,vd) + tmp;

    %Hdsym = lhs(3,:);
    %Hdsym = reshape(Hdsym,numlinks,1);
    
    
    %lhs is in terms of both omega^2 and omega dot right now.

    %set omega to 0 to find M matrix
    tmp = collect(subs(lhs,th_d,zeros(3,numlinks)),th_dd);
    %each column is actually equation -> one row of a matrix. Flatten
    tmp = tmp(3,:);
    tmp = reshape(tmp, numlinks, 1);

    %set omega dot to 0 to find C matrix
    tmp2 = collect(subs(lhs,th_dd,zeros(3,numlinks)),th_d);
    %each column is actually equation -> one row of a matrix. Flatten
    tmp2 = tmp2(3,:);
    tmp2 = reshape(tmp2, numlinks, 1);

    Msym = sym(zeros(numlinks));
    Csym = Msym;

     for I = 1:numlinks
         logicmat = zeros(3,numlinks);
         logicmat(3,I) = 1;

         Msym(:,I) = subs(tmp,th_dd,logicmat);
         Csym(:,I) = subs(tmp2,th_d,logicmat);
     end

     Msym = simplify(Msym);
     Csym = simplify(Csym);

    tmp = repmat(m,[3 1]); %m
    tmp = cumsum(tmp(:,2:numlinks),2,'reverse'); %sum n+1 to N
    tmp = [tmp, zeros(3,1)].*lp; %sum n+1 to N

    %torques (currently due to weight only)
    rhs = cross((repmat(m,[3 1]).*r + tmp),repmat(gvec,[1 numlinks]));
    Tsym = rhs(3,:)';


    %Jacobian
    %Consider bottom of circular foot the end effector location for this
    lpmod = lp;
    lpmod(:,end) = lp(:,end) - R.*x(:,end) - R.*x0;

    Jw = [zeros(3,numlinks-1),z(:,end)]; %independent angles mean that only last angle affects orientation
    Jv = cross(z,lpmod);
    Jsym = [Jv; Jw];

    %Joint Position (for animation), uses modified end location (till center of
    %foot)
    lpmod = lp;
    lpmod(:,end) = lp(:,end) - R.*x(:,end);
    Xsym = cumsum(lpmod,2);

    %% Numerical Substitution
    symvars = [R, l, m, c, w, rg];
    params = [ParameterCell{1}(1), ParameterCell{2}',ParameterCell{3}',...
        ParameterCell{4}',ParameterCell{5}',ParameterCell{6}'];

    Mpsym = subs(Msym,w,(-w));
    
    M = vpa(subs(Msym,symvars,params));
    Mp = vpa(subs(Mpsym,symvars,params));
    C = vpa(subs(Csym,symvars,params));
    T = vpa(subs(Tsym,symvars,params));
    J = vpa(subs(Jsym,symvars,params));
    X = vpa(subs(Xsym,symvars,params));
    H = vpa(subs(Hsym,symvars,params));
    %Hd = vpa(subs(Hdsym,symvars,params));

    pointlocs = find(~logical(ParameterCell{2})); %find point masses

    for I=1:length(pointlocs) %angle is irrelevant, ignore
        pointloc = pointlocs(I); %remove first location

        M = M([1:pointloc - 1 pointloc+1:numlinks],[[1:pointloc-1 pointloc+1:numlinks]]);
        M = subs(M,th(pointloc+1:end),th(pointloc:end-1));
        
        Mp = Mp([1:pointloc - 1 pointloc+1:numlinks],[[1:pointloc-1 pointloc+1:numlinks]]);
        Mp = subs(Mp,th(pointloc+1:end),th(pointloc:end-1));

        C = C([1:pointloc-1 pointloc+1:numlinks],[[1:pointloc-1 pointloc+1:numlinks]]);
        C = subs(C,th(pointloc+1:end),th(pointloc:end-1));

        T = T([1:pointloc-1 pointloc+1:numlinks]);
        T = subs(T,th(pointloc+1:end),th(pointloc:end-1));

        J = J(:,[1:pointloc-1 pointloc+1:numlinks]);
        J = subs(J,th(pointloc+1:end),th(pointloc:end-1));

        X = X(:,[1:pointloc-1 pointloc+1:numlinks]);
        X = subs(X,th(pointloc+1:end),th(pointloc:end-1));

        H = H(:,[1:pointloc-1 pointloc+1:numlinks]);
        H = subs(H,th(pointloc+1:end),th(pointloc:end-1));
        H = subs(H,th_d(:,pointloc+1:end),th_d(:,pointloc:end-1));
                
        numlinks = numlinks-1;
        pointlocs = pointlocs - 1; %later locations have shifted up by one
    end

    Mf = matlabFunction(M);
    Mpf = matlabFunction(Mp);
    Cf = matlabFunction(C);
    Tf = matlabFunction(T);
    Jf = matlabFunction(J);
    Xf = matlabFunction(X);
    Hf = matlabFunction(H);

    %save for later
    if (SaveBoolean)
        if (~isempty(jointsToLock))
            savename = sprintf('EOMs_%s_%s_locked.mat',extractBefore(datafilename,'.txt'),strjoin(string(jointsToLock),'_'));
        else
            savename = sprintf('EOMs_%s_unlocked.mat',extractBefore(datafilename,'.txt'));
        end
        save(savename,'Mf','Mpf','Cf','Tf','Jf','Xf','Hf','numlinks','ParameterCell');
    end
end