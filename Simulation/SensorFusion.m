function s = SensorFusion(a, b, w)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik Söderlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     innov = a-b;
%     dist = innov'*inv(C)*innov;
% 
%     if (dist < 5.991)
%         w = dist/5.991;
%         s = w*a + (1-w)*b;
%     else
%         s = a;
%     end

    s = w*a + (1-w)*b;

end
