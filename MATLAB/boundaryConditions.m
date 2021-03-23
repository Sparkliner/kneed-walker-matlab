function [eqc, ineqc] = boundaryConditions(xF,x0,parameters,R,stanceAngle)
%x0-beginning of step
%xF-end of step, just before collision
ineqc = []; %no inequality constraint
[xNew, ~, ~] = HeelCollisionJacobian(xF,parameters.Mf,parameters.Jf,parameters.Xf,parameters.numlinks,R);
xNew(1:4) = wrapToPi(xNew(1:4));
eqc1 = x0 - xNew;
eqc2 = (xF(1:4,:) - [2*stanceAngle; 2*stanceAngle; -2*stanceAngle; -2*stanceAngle]) - x0(1:4,:); %assure gait symmetry
eqc = [eqc1; eqc2];
end