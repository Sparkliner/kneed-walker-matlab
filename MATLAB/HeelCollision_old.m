function [Xnew,steplength] = HeelCollision(X,Mfpre,Mfpost,Xf,numlinks,R)%,Hfpre,Hfpost)
%HeelCollision calculates impact of heel strike
JointAngleCell1 = num2cell(X(1:numlinks));

jointPositions = Xf(JointAngleCell1{:});
steplength = jointPositions(2,end) + (X(1))*R -(X(2) - pi)*R; %don't need to account for R as only y matters here

theta1old = X(1);
theta2old = X(2);
theta3old = X(2);
JointAngleCell2 = num2cell([theta1old,theta2old,theta3old]);

Mpre = Mfpre(JointAngleCell2{:});

%current velocities
omega1old = X(3);
omega2old = X(4);
omega3old = X(4);
Omegapre = [omega1old;omega2old;omega3old];

%find new angles using kinematics/geometry
theta1new = X(2) - pi;
theta2new = X(1) - pi;
theta3new = X(1) - pi;
JointAngleCellNew = num2cell([theta1new,theta2new,theta2new]);

Mpost = Mfpost(JointAngleCellNew{:});

%equate angular velocity
Hpre = Mpre*Omegapre;
Omegapost = Mpost\Hpre;

omegacell = num2cell(Omegapost');

%disp(Hfpre(X(1),X(2),X(3),X(4)))
%disp(Hfpost(theta1new,theta2new,theta3new,omegacell{:}));
%fprintf('\n\n\n');
%pause();

Xnew = [theta1new,theta2new,theta3new,Omegapost'];

end

