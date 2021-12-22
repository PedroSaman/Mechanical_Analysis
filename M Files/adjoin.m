function A = adjoin(line)
%Find adjacent blocks in this layer from the block model
%input: line: each knob in a given layer. (x,y,z,block_type,color,block_number,knob_index)
%output: A: for each adjacent block there are one line in A with 7 info: 
%           (to the right(1) or top(2),own block_number, own block_type, adjacent block_number, adjacent block_type, adjacent knob_index, adjoin_type)
%           if there is no adjacent block, this function returns -1

blocks = size(line);
X = zeros(blocks(1) * 2, 7);  Y = zeros(blocks(1) * 2, 7);
count_x = 1;  count_y = 1;  same_block = 0;

for n = 1 : blocks(1) %for every knob in this layer
    x = -1 * ones(2, 5);  y = -1 * ones(2, 5); %Initialize x and y for this knob
    %x: adjoint knob information to the right (x,y,knob block number,knob block_type, knob_index in the block model)
    %y: adjoint knob information to the top (x,y,knob block number,knob block_type, knob_index in the block model)
    if(same_block ~= 0) %If the knob is from the same block, continue. This will be computed until it reaches the next block.
        same_block = same_block - 1; %Generic subtractor
    elseif(line(n, 4) == 22) %if the block is 2x2
        check = 4; %In a 2x2 block, there is only 4 knob spaces possible to be adjacent (to the top and to the right)
        same_block = 3; %In a 2x2 block, excluding the first knob, there are only 3 more knobs
        x(1, 1) = line(n, 1) + 2;  x(1, 2) = line(n, 2);    
        x(2, 1) = line(n, 1) + 2;  x(2, 2) = line(n, 2) + 1; %Coordinates of the the possible adjoint block to the right.
        y(1, 1) = line(n, 1);  y(1, 2) = line(n, 2) + 2;
        y(2, 1) = line(n, 1) + 1;  y(2, 2) = line(n, 2) + 2; %Coordinates of the the possible adjoint block to the top.
        
        for m = 1 : blocks(1) %For every knob in this layer
            if(check == 0) %If all possible adjacent knob were checked, break.
                break;
            elseif( (line(m, 1) == x(1, 1)) && (line(m, 2) == x(1, 2)) ) %This knob (m) is one of the possible adjoint knob to the right?
                x(1, 3) = line(m, 6);  x(1, 4) = line(m, 4);  x(1, 5) = line(m, 7);  check = check -1; %add information to x (block_number,block_type,knob_index)
            elseif( (line(m, 1) == x(2, 1)) && (line(m, 2) == x(2, 2)) ) %This knob (m) is one of the possible adjoint knob to the right?
                x(2, 3) = line(m, 6);  x(2, 4) = line(m, 4);  x(2, 5) = line(m, 7);  check = check -1; %add information to x (block_number,block_type,knob_index)
            elseif( (line(m, 1) == y(1, 1)) && (line(m, 2) == y(1, 2)) ) %This knob (m) is one of the possible adjoint knob to the top?
                y(1, 3) = line(m, 6);  y(1, 4) = line(m, 4);  y(1, 5) = line(m, 7);  check = check -1; %add information to y (block_number,block_type,knob_index)
            elseif( (line(m, 1) == y(2, 1)) && (line(m, 2) == y(2, 2)) ) %This knob (m) is one of the possible adjoint knob to the top?
                y(2, 3) = line(m, 6);  y(2, 4) = line(m, 4);  y(2, 5) = line(m, 7);  check = check -1; %add information to y (block_number,block_type,knob_index)
            end
        end

        %After finding which of the possible adjacent knobs in fact exist
        if( (x(1, 3) ~= -1) && (x(2, 3) ~= -1) ) %If there are two knobs to the right
            if(x(1, 3) == x(2, 3)) %If both knobs are from the same block
                X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(1, 3), x(1, 4), x(1, 5), 3]; %add info to X matrix (right)
                count_x = count_x + 1; %count the number of adjacent blocks to the right
            else %If each knob to the right are from different blocks
                X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(1, 3), x(1, 4), x(1, 5), 5]; 
                count_x = count_x + 1;
                X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(2, 3), x(2, 4), x(2, 5), 6]; 
                count_x = count_x + 1;
            end
        else %There are only one knob to the right
            if(x(1, 3) ~= -1) %Is the first knob that exists?
                X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(1, 3), x(1, 4), x(1, 5), 5];
                count_x = count_x + 1;
            elseif(x(2, 3) ~= -1) %Is the second knob that exists?
                X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(2, 3), x(2, 4), x(2, 5), 6];
                count_x = count_x + 1;
            end
        end
        if( (y(1, 3) ~= -1) && (y(2, 3) ~= -1) ) %If there are two knobs to the top 
            if(y(1, 3) == y(2, 3)) %If both knobs are from the same block
                Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(1, 3), y(1, 4), y(1, 5), 3]; %add info to Y matrix (top)
                count_y = count_y + 1; %count the number of adjacent blocks to the top
            else %If each knob to the top are from different blocks
                Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(1, 3), y(1, 4), y(1, 5), 5];
                count_y = count_y + 1;
                Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(2, 3), y(2, 4), y(2, 5), 6];
                count_y = count_y + 1;
            end
        else %There are only one knob to the top
            if(y(1, 3) ~= -1) %Is the first knob that exists?
                Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(1, 3), y(1, 4), y(1, 5), 5];
                count_y = count_x + 1;
            elseif(y(2, 3) ~= -1) %Is the second knob that exists?
                Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(2, 3), y(2, 4), y(2, 5), 6];
                count_y = count_y + 1;
            end
        end
   
    elseif(line(n, 4) == 12) %if the block is 1x2
        check = 3; %In a 1x2 block, there are only 3 knob spaces possible to be adjacent (1 to the top and 2 to the right)
        same_block = 1; %In a 1x2 block, excluding the first knob, there are only 1 more knobs
        x(1, 1) = line(n, 1) + 1;  x(1, 2) = line(n, 2);
        x(2, 1) = line(n, 1) + 1;  x(2, 2) = line(n, 2) + 1; %Coordinates of the possible adjoint block to the right.
        y(1, 1) = line(n, 1);  y(1, 2) = line(n, 2) + 2; %Coordinate of the possible adjoint block to the top.
        y(2, 1:3) = -1 * ones(1, 3);
        
        for m = 1 : blocks(1) %For every knob in this layer
            if(check == 0) %If all possible adjacent knob were checked, break.
                break;
            elseif( (line(m, 1) == x(1, 1)) && (line(m, 2) == x(1, 2)) ) %This knob (m) is one of the possible adjoint knob to the right?
                x(1, 3) = line(m, 6);  x(1, 4) = line(m, 4);  x(1, 5) = line(m, 7);  check = check -1; %add information to x (block_number,block_type,knob_index)
            elseif( (line(m, 1) == x(2, 1)) && (line(m, 2) == x(2, 2)) )
                x(2, 3) = line(m, 6);  x(2, 4) = line(m, 4);  x(2, 5) = line(m, 7);  check = check -1;
            elseif( (line(m, 1) == y(1, 1)) && (line(m, 2) == y(1, 2)) ) %This knob (m) is one of the possible adjoint knob to the top?
                y(1, 3) = line(m, 6);  y(1, 4) = line(m, 4);  y(1, 5) = line(m, 7);  check = check -1; %add information to y (block_number,block_type,knob_index)
            end
        end
        
        %After finding which of the possible adjacent knobs in fact exist
        if( (x(1, 3) ~= -1) && (x(2, 3) ~= -1) ) %If there are two knobs to the right
            if(x(1, 3) == x(2, 3)) %If both knobs are from the same block
                X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(1, 3), x(1, 4), x(1, 5), 3]; %add info to X matrix (right)
                count_x = count_x + 1; %count the number of adjacent blocks to the right
            else %If each knob to the right are from different blocks
                X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(1, 3), x(1, 4), x(1, 5), 5];
                count_x = count_x + 1;
                X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(2, 3), x(2, 4), x(2, 5), 6];
                count_x = count_x + 1;
            end
        else %There are only one knob to the right
            if(x(1, 3) ~= -1) %Is the first knob that exists?
                X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(1, 3), x(1, 4), x(1, 5), 5];
                count_x = count_x + 1;
            elseif(x(2, 3) ~= -1) %Is the second knob that exists?
                X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(2, 3), x(2, 4), x(2, 5), 6];
                count_x = count_x + 1;
            end
        end
        if(y(1, 3) ~= -1) %There are a knob to the top?
            Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(1,3), y(1, 4), y(1, 5), 1]; %add info to Y matrix (top)
            count_y = count_y + 1; %count the number of adjacent blocks to the top
        end

    elseif(line(n, 4) == 21) %if the block is 2x1
        check = 3; %In a 2x1 block, there are only 3 knob spaces possible to be adjacent (2 to the top and 1 to the right)
        same_block = 1; %In a 2x1 block, excluding the first knob, there are only 1 more knobs
        x(1, 1) = line(n, 1) + 2;  x(1, 2) = line(n, 2); %Coordinate of the possible adjoint block to the right.
        x(2, 1:3) = -1 *ones(1, 3);
        y(1, 1) = line(n, 1);  y(1, 2) = line(n, 2) + 1;
        y(2, 1) = line(n, 1) + 1;  y(2, 2) = line(n, 2) + 1; %Coordinates of the possible adjoint block to the top.
        
        for m = 1 : blocks(1) %For every knob in this layer
            if(check == 0) %If all possible adjacent knob were checked, break.
                break;
            elseif( (line(m, 1) == x(1, 1)) && (line(m, 2) == x(1, 2)) ) %This knob (m) is one of the possible adjoint knob to the right?
                x(1, 3) = line(m, 6);  x(1, 4) = line(m, 4);  x(1, 5) = line(m, 7);  check = check -1; %add information to x (block_number,block_type,knob_index)
            elseif( (line(m, 1) == y(1, 1)) && (line(m, 2) == y(1, 2)) ) %This knob (m) is one of the possible adjoint knob to the top?
                y(1, 3) = line(m, 6);  y(1, 4) = line(m, 4);  y(1, 5) = line(m, 7);  check = check -1; %add information to y (block_number,block_type,knob_index)
            elseif( (line(m, 1) == y(2, 1)) && (line(m, 2) == y(2, 2)) )
                y(2, 3) = line(m, 6);  y(2, 4) = line(m, 4);  y(2, 5) = line(m, 7);  check = check -1;
            end
        end
        
        if(x(1, 3) ~= -1) %There are a knob to the right?
            X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(1,3), x(1, 4), x(1, 5), 1]; %add info to X matrix (right)
            count_x = count_x + 1; %count the number of adjacent blocks to the right
        end
        if( (y(1, 3) ~= -1) && (y(2, 3) ~= -1) ) %If there are two knobs to the top
            if(y(1, 3) == y(2, 3)) %If both knobs are from the same block
                Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(1, 3), y(1, 4), y(1, 5), 3]; %add info to Y matrix (top)
                count_y = count_y + 1; %count the number of adjacent blocks to the top
            else %If each knob to the top are from different blocks
                Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(1, 3), y(1, 4), y(1, 5), 5];
                count_y = count_y + 1;
                Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(2, 3), y(2, 4), y(2, 5), 6];
                count_y = count_y + 1;
            end
        else %There are only one knob to the top
            if(y(1, 3) ~= -1) %Is the first knob that exists?
                Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(1, 3), y(1, 4), y(1, 5), 5];
                count_y = count_y + 1;
            elseif(y(2, 3) ~= -1) %Is the second knob that exists?
                Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(2, 3), y(2, 4), y(2, 5), 6];
                count_y = count_y + 1;
            end
        end
    elseif(line(n, 4) == 11) %if the block is 1x1
        check = 2; %In a 1x1 block, there are only 2 knob spaces possible to be adjacent (1 to the top and 1 to the right)
        x(1, 1) = line(n, 1) + 1;  x(1, 2) = line(n, 2); %Coordinate of the possible adjoint block to the right.
        x(2, 1:3) = -1 *ones(1, 3);
        y(1, 1) = line(n, 1);  y(1, 2) = line(n, 2) + 1; %Coordinate of the possible adjoint block to the top.
        y(2, 1:3) = -1 *ones(1, 3);
        
        for m = 1 : blocks(1) %For every knob in this layer
            if(check == 0) %If all possible adjacent knob were checked, break.
                break;
            elseif( (line(m, 1) == x(1, 1)) && (line(m, 2) == x(1, 2)) ) %This knob (m) is one of the possible adjoint knob to the right?
                x(1, 3) = line(m, 6);  x(1, 4) = line(m, 4);  x(1, 5) = line(m, 7);  check = check -1; %add information to x (block_number,block_type,knob_index)
            elseif( (line(m, 1) == y(1, 1)) && (line(m, 2) == y(1, 2)) ) %This knob (m) is one of the possible adjoint knob to the top?
                y(1, 3) = line(m, 6);  y(1, 4) = line(m, 4);  y(1, 5) = line(m, 7);  check = check -1; %add information to y (block_number,block_type,knob_index)
            end
        end
        
        if(x(1, 3) ~= -1) %There are a knob to the right?
            X(count_x, 1:7) = [1, line(n, 6), line(n, 4), x(1,3), x(1, 4), x(1, 5), 1]; %add info to X matrix (right)
            count_x = count_x + 1; %count the number of adjacent blocks to the right
        end
        if(y(1, 3) ~= -1) %There are a knob to the top?
            Y(count_y, 1:7) = [2, line(n, 6), line(n, 4), y(1,3), y(1, 4), y(1, 5), 1]; %add info to Y matrix (top)
            count_y = count_y + 1; %count the number of adjacent blocks to the top
        end
    end

    %optimization problem here, every loop in m it execute this if else condicions and rewrite an info that may have not changed at all
    if((count_x ~= 1) && (count_y ~= 1)) %if there is something in X or Y matrix
        A = [X(1:(count_x-1), 1:7); Y(1:(count_y-1), 1:7)];
    elseif(count_x ~= 1) %if there is something only in X matrix
        A = X(1:(count_x-1), 1:7);
    elseif(count_y ~= 1) %if there is something only in Y matrix
        A = Y(1:(count_y-1), 1:7);
    else %There are no adjacent blocks
        A = -1;
    end
end