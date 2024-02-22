function [csv_plan] = plan_formatation(plan,file_location,final_plan_flag)
%myFun - Description
%
% Syntax: output = myFun(input)
%
% Long description

% #AssemblyArea,X,Y,Z,SizeX,SizeY,SizeZ,ColorIndex,IsSupport,CanPress,ShiftX,ShiftY
    plan_size = size(plan,1);
    csv_plan = ones(plan_size,12);
    for i = 1:plan_size
        csv_plan(i,1) = 0;
        csv_plan(i,2:3) = plan(i,2:3) - 1;
        csv_plan(i,4) = plan(i,4);
        [csv_plan(i,5),csv_plan(i,6)] = col_row_converter(plan(i,5));

        if(plan(i,6) ~= 99)
            csv_plan(i,8) = plan(i,6);
            csv_plan(i,9) = 0;
        else
            csv_plan(i,8) = 0;
        end
        csv_plan(i,10) = plan(i,7);
    end
    
    if(final_plan_flag == 1)
        shiftX = zeros(plan_size,1);
        shiftY = zeros(plan_size,1);
        for k = 1:plan(end,4)
            knob_i = knob(plan, k); % Layer knob information
            adjoin_i = adjoin(knob_i); % Layer adjoin blocks information
            for j = 1:size(adjoin_i,1)
                if(adjoin_i(j,1) == 1) %shiftX
                    block_in_the_left = adjoin_i(j,2);
                    block_in_the_right = adjoin_i(j,4);
                    if(block_in_the_left>block_in_the_right)
                        shiftX(block_in_the_left) = shiftX(block_in_the_left) - 1;
                    elseif(block_in_the_right>block_in_the_left)
                        shiftX(block_in_the_right) = shiftX(block_in_the_right) + 1;
                    end
                elseif(adjoin_i(j,1) == 2) %shiftY
                    block_in_the_front = adjoin_i(j,2);
                    block_in_the_back = adjoin_i(j,4);
                    if(block_in_the_front>block_in_the_back)
                        shiftY(block_in_the_front) = shiftY(block_in_the_front) - 1;
                    elseif(block_in_the_back>block_in_the_front)
                        shiftY(block_in_the_back) = shiftY(block_in_the_back) + 1;
                    end
                end
            end
        end

        for k = 1:plan_size
            if(shiftX(k) > 0)
                shiftX(k) = 1;
            elseif(shiftX(k) < 0)
                shiftX(k) = -1;
            end
            if(shiftY(k) > 0)
                shiftY(k) = 1;
            elseif(shiftY(k) < 0)
                shiftY(k) = -1;
            end
        end

        csv_plan(:,11) = shiftX;
        csv_plan(:,12) = shiftY;
    end
    csvwrite(strrep(file_location, 'txt' , 'csv' ), csv_plan);
end