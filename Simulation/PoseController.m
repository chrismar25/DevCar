function [v, gamma] = PoseController(states, align)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    k_rho = 0.5;
    k_alpha = 3;
    k_beta = -1.4;
    
    if (align == false)
        v = k_rho*states(1);
        gamma = k_alpha*states(2) + k_beta*states(3);
    else
        gamma = k_alpha*states(2) + k_beta*states(3);
        v = 0.2;
    end
end