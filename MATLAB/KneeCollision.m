function [Xnew] = KneeCollision(X,Mfpre,Mfpost,numlinks)
%KneeCollision calculates impact of knee locking
JointAngleCell = num2cell(X(1:numlinks));
Omegapre = X(numlinks+1:end)';

Mpre = Mfpre(JointAngleCell{:});

%find new angles using kinematics/geometry
theta1new = X(1);
theta2new = X(2);
JointAngleCellNew = num2cell([theta1new,theta2new]);

Mpost = Mfpost(JointAngleCellNew{:});

%equate angular velocity
Hpre = cumsum(Mpre*Omegapre,'reverse');
Hpre12 = Hpre(1:2); %total momentum about foot and about hip
Mpost12 = cumsum(Mpost,1,'reverse'); %about foot and hip
Omegapost = Mpost12\Hpre12;

%only theta 1,2 are relevant after knee lock
Xnew = [theta1new,theta2new,Omegapost(1),Omegapost(2)];

end

