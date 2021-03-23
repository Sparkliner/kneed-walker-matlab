function [Xnew,steplength] = HeelCollisionJacobian(X,Mfunlocked,Mfcoll,Mfpost,Jfpost,Xf,numlinks,R)%,Hfpre,Hfpost)
%HeelCollision calculates impact of heel strike
JointAngleCellPre = num2cell(X(1:numlinks));

jointPositions = Xf(JointAngleCellPre{:});
steplength = jointPositions(2,end) + (X(1))*R -(X(2) - pi)*R; %don't need to account for R as only y matters here

%new angles are old angles in new coordinates
theta1new = X(2) - pi;
theta2new = X(1) - pi;
theta3new = X(1) - pi;
JointAngleCellPost = num2cell([theta1new,theta2new,theta3new]); %equivalent angles after collision (switch stance/swing)
JointAngleCellColl = num2cell([X(1),X(1),X(2)]); %equivalent angles at instant of collision (unlocked stance knee)

Mpost = Mfcoll(JointAngleCellPost{:});

J = Jfpost(JointAngleCellPost{:});
J = J(1:3,:); %velocity Jacobian only

%current velocities (in current frame)
omega1old_old = X(3);
omega2old_old = X(4);
omega3old_old = X(4);

%current velocities (in new coordinates, th1 is new stance/old swing leg)
omega1old_new = omega3old_old;
omega2old_new = omega1old_old;
omega3old_new = omega1old_old;

Omegapre = [omega1old_new;omega2old_new;omega3old_new];

% theta1new = X(2) - pi;
% theta2new = X(1) - pi;
% theta3new = X(1) - pi;
% JointAngleCellNew = num2cell([theta1new,theta2new,theta2new]);
% 
% Mpost = Mfpost(JointAngleCellNew{:});
% 
% %equate angular velocity
% Hpre = Mpre*Omegapre;
% Omegapost = Mpost\Hpre;
% 
% omegacell = num2cell(Omegapost');
% 
% %disp(Hfpre(X(1),X(2),X(3),X(4)))
% %disp(Hfpost(theta1new,theta2new,theta3new,omegacell{:}));
% %fprintf('\n\n\n');
% %pause();
% 
Xnew = [theta1new,theta2new,theta3new,Omegapost'];

end

