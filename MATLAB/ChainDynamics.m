function [ X_d ] = ChainDynamics( ~, X, Mf, Cf, Tf, gval, gamval) %t is not used for now
% Currently has issues if R = cm,x for first/last link
numlinks = length(X)/2;
Theta = X(1:numlinks);
ThetaCell = num2cell(Theta);
Omega = X((numlinks+1):end);
%Insert matrices here
M = Mf(ThetaCell{:});
if (numlinks > 1)
    C = Cf(ThetaCell{:});
else
    C = 0;
end
T = Tf(gval,gamval,ThetaCell{:});

%M*Omega_d + C*Omega^2 = T
Theta_d = Omega;
Omega_d = M\(T - C*(Omega.*Omega));


X_d = [Theta_d; Omega_d];

end

