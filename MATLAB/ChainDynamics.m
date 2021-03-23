function [ X_d ] = ChainDynamics( t, X, Mf, Cf, Tf, Text, gval, gamval) %t is not used for now
    % Currently has issues if R = cm,x for first/last link
    numlinks = size(X,1)/2;
    Theta = X(1:numlinks,:);
    ThetaCell = num2cell(Theta);
    Omega = X((numlinks+1):end,:);
    for I=1:size(Theta,2)
        %Insert matrices here
        %ThC = ThetaCell(:,I);
        M = Mf(ThetaCell{:,I});
        if (numlinks > 1)
            C = Cf(ThetaCell{:,I});
        else
            C = 0;
        end

        %left multiply by matrix:
        %[1, -1,  0]
        %[0,  1, -1]
        %[0,  0,  1]
        Tjoint = Text(:,I);
        Ttemp = [Tjoint(2:end,:);0];
        Tjoint = Tjoint - Ttemp;
 
        Tjoint(1) = 0; %can't control ankle
 
        T = Tf(gval,gamval,ThetaCell{:,I});
        T = T + Tjoint;

        %M*Omega_d + C*Omega^2 = T
        Theta_d = Omega(:,I);
        Omega_d = M\(T - C*(Omega(:,I).*Omega(:,I)));

%         %simplified dynamics
%         Omega_d = T;

        X_d(:,I) = [Theta_d; Omega_d];
    end

end

