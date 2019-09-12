function C = CovarianceMatrix(V)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%   HOME EXAM PART - Telerobotics and applied sensor fusion, 7.5hp
%   Author: Henrik S�derlund
%   Date: 2018-01-19
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    C = ((V-mean(V,2))*(V-mean(V,2))')./numel(V(1,:));

end