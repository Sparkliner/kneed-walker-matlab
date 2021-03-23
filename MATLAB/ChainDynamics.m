function [ X_d ] = ChainDynamics( t, X, Mf, Cf, Tf,numlinks, trajS, gval, gamval, controller)%, u) %t is not used for now
    % Currently has issues if R = cm,x for first/last link
    if (isstruct(controller)) && (isequal(controller.type,'comptorque'))
        Qr = X(2*numlinks+1:end,:); %reference variables for controller
    end
    X = X(1:2*numlinks,:);
    Theta = X(1:numlinks,:);
    ThetaCell = num2cell(Theta);
    Omega = X((numlinks+1):end,:);
    for I=1:size(Theta,2)
        %Insert matrices here
        %ThC = ThetaCell(:,I);
        M_cmp = Mf(ThetaCell{:,I});
        
        C_cmp = Cf(ThetaCell{:,I})*diag(Omega(:,I));
        
        G = -Tf(gval,gamval,ThetaCell{:,I}); %gravity torque
        %G(1) = 0;
        %G = zeros(4,1);
        
        %left multiply by matrix:
        %[1, -1,  0]
        %[0,  1, -1]
        %[0,  0,  1]
        %T = Text(:,I);
        
        %T = Text(t,X);
        %Ttemp = [T(2:end,:);0];
        %T = T - Ttemp;
 
        %test - computed torque method with known parameters
                
        %assume we can only measure position and velocity of joints;
        if isstruct(controller)
            if (controller.type == 'comptorque')       
                %calculate reference values
                qd_ref = Qr(1:4,I);
                %int_error = Qr(5:8,I);
                int_error = [];
                %estimate of parameters
                M_est = M_cmp;
                C_est = C_cmp;

                tNow = t(I);

                [T, qdd_ref, error] = computedTorque(tNow, M_est, C_est, G, Theta(:,I),Omega(:,I),qd_ref,trajS,int_error);%,u(:,I));
                error = [];
            end
        else
            T = [0;controller(:,I)];
            qdd_ref = [];
        end
        
        %T(1) = 0; %can't control ankle, this makes sure;
               
        %M*Omega_d + C*Omega^2 + G = T
        %uncertainty (if any)
%         M = [M_cmp(1,:);
%             [M_cmp(2:end,1), eye(3)]];
%         C = [C_cmp(1,:);
%             [C_cmp(2:end,1), zeros(3)]];

        M = M_cmp;
        C = C_cmp;

        Theta_d = Omega(:,I);
        Omega_d = M\(T - C*(Omega(:,I)) - G);

%         %simplified dynamics
%         Omega_d = T;

        X_d(:,I) = [Theta_d; Omega_d; qdd_ref; error];
    end

end

