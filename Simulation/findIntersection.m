function [x_x, y_x, flag] = findIntersection(line1, line2)
% findIntersection   Finds the intersection point between two 
%                    non-parallel lines.
% INPUT:
%   line1   The first line L1 = [x1_1 x2_1; y1_1 y2_1].
%   line2   The second line L2 = [x1_2 x2_2; y1_2 y2_2].
% OUTPUT:
%   x_x     The x-coordinate of the intersection point.
%   y_x     The y-coordinate of the intersection point.
%   flag    Indicator if the intersection was found or not. flag==-1
%           indicates intersection NOT found, flag==0 indicates
%           intersection found.

    C = [line2(:,1); 0];
    D = [line2(:,2); 0];

    A = [line1(:,1); 0];
    B = [line1(:,2); 0];

    h = @(P)cross(B-A, P-A);

    hC = h(C);
    hD = h(D);

    x_x = 0;
    y_x = 0;

    if (dot(hC,hD) < 0)

        try
            % Fit linear polynomial
            p1 = polyfit(line1(1,:),line1(2,:),1);
            p2 = polyfit(line2(1,:),line2(2,:),1);
            % Calculate intersection
            x_x = fzero(@(x) polyval(p1-p2,x),3);
            y_x = polyval(p1,x_x); 

            % If no intersection was found, output flag=-1
            if (isnan(x_x) || isnan(y_x))
                flag = -1;
               
            % Otherwise, output flag=0
            else 
                flag = 0;
            end
        catch
            flag = -1;
        end

    else
        flag = -1;
    end

end


