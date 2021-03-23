function [Xnew,steplength] = HeelCollision(X,Mfpre,Mfpost,Xf,numlinks,R)%,Hfpre,Hfpost)
%HeelCollision calculates impact of heel strike
JointAngleCellPre = num2cell(X(1:numlinks));

jointPositions = Xf(JointAngleCellPre{:});
steplength = jointPositions(2,end) + (X(1))*R -(X(2) - pi)*R; %don't need to account for R as only y matters here

%new angles are old angles in new coordinates
theta1new = wrapToPi(X(2) - pi);
theta2new = wrapToPi(X(1) - pi);
theta3new = wrapToPi(X(1) - pi);
JointAngleCellPost = num2cell([theta1new,theta2new,theta3new]); %equivalent angles after collision (switch stance/swing)
JointAngleCellColl = num2cell([theta1new,theta2new]); %equivalent angles at instant of collision (unlocked stance knee)

Mpre = Mfpre(JointAngleCellColl{:}); %Inertia about old swing foot
Mpost = Mfpost(JointAngleCellPost{:}); %Inertia about new stance foot

%current velocities (in old frame)
omega1old_old = X(3);
omega2old_old = X(4);

%current velocities (in new coordinates, th1 is new stance/old swing leg)
omega1old_new = omega2old_old;
omega2old_new = omega1old_old;

Omegapre = [omega1old_new;omega2old_new];

H_swing_pre = cumsum(Mpre*Omegapre,1,'reverse'); %H of o. sw. leg about o. sw. foot and o. st. leg about hip

Mpost = cumsum(cumsum(Mpost,1,'reverse'),2,'reverse');
Mpost = Mpost(1:2,1:2);

Omegapost = Mpost\H_swing_pre;
Omegapost(3) = Omegapost(2);

Xnew = [theta1new,theta2new,theta3new,Omegapost'];

end

