function [Xnew,steplength,F] = HeelCollisionJacobian(X,Mf,Jf,Xf,numlinks,R)%,Hfpre,Hfpost)
%returns new configuration, velocities, and contact force
X = X(1:2*numlinks); %strip off any excess data
JointAngleCellPre = num2cell(X(1:numlinks));

jointPositions = Xf(JointAngleCellPre{:});
X(numlinks) = wrapToPi(X(numlinks));
steplength = jointPositions(2,end) + (X(1))*R -(X(numlinks) - pi)*R; %don't need to account for R as only y matters here

thetanew = flipud(wrapToPi(X(1:numlinks)-pi)); %new angle is old-pi, then flipped so first is last etc.

M = Mf(JointAngleCellPre{:}); %Inertia about old stance foot, in old coordinates

J = Jf(JointAngleCellPre{:}); %Jacobian of old stance foot, in old coordinates
J = J(1:2,:); %only consider translation in x and y for 2 dimensional system

Omegapre = X(numlinks+1:end);

lhs = [M, -J';J, zeros(size(J,1))];
rhs = [M*Omegapre;zeros(size(J,1),1)];

%infinite solutions exist to the above
%find least norm solution (system seeks to minimize energy)

omegaF = lhs'*((lhs*lhs')\rhs);

Omegapost_oldc = omegaF(1:numlinks,:); %new velocities in old coordinates;
F = [omegaF(numlinks+1:end,:); 0];

Omegapost = flipud(Omegapost_oldc); %coordinates are flipped

Xnew = [thetanew;Omegapost];

end

