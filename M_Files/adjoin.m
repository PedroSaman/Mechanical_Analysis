function adjoint_blocks = adjoin(line)
%Compute which blocks are adjacent to one another in the block model
%
%input: line: every knob in a given layer. (x,y,z,block_type,color,block_number,knob_index)
%output: A: for each pair of adjacent knobs there are one line in A with 7 info: 
%           (X axis(1) or Y axis(2), first block_number, first block_type, 
%           second block_number, second block_type, second knob_index, adjoin_type)
%Obs: If there is no adjacent block, this function returns -1.
%In the Y axis only checks to the top and in the X axis only checks to the right.
%adjoin_type is according with Pedro definition: Type_xy means: the first block
%is touching the second block from the knob x to knob y (knobs of the first block).

knobs = size(line);
X = zeros(knobs(1) * 2, 7);  Y = zeros(knobs(1) * 2, 7);
count_x = 1;  count_y = 1;  same_block = 0;

for n = 1 : knobs(1) % For every knob in this layer
    
    if(same_block ~= 0) % If the knob is from the same block, continue. This will be computed until it reaches the next block.
        same_block = same_block - 1;
    else % If this is the first knob in a block
        
        [col,row] = col_row_converter(line(n, 4)); % Block number of columns and rows
        x = -ones(row,5); y = -ones(col,5); % Initialize the x and y matrices
        check = row+col;
        same_block = row*col-1;

        %Determine where are the possible adjoin position for this block
        for i=0:row-1
            x(i+1,1) = line(n,1) + col; x(i+1,2) = line(n,2) + i;
        end
        for i=0:col-1
            y(i+1,1) = line(n,1) + i;   y(i+1,2) = line(n,2) + row;
        end

        %Search if there are knobs in the possible adjoin knob positions
        for m = 1 : knobs(1) % For every knob in this layer
            if(line(n,6)==line(m,6)) % Skip the knobs from the current block
                continue;
            end
            if(check == 0) % If all possible adjacent knob were checked, break.
                break;
            else
                for i=1:row
                    if((line(m, 1) == x(i, 1)) && (line(m, 2) == x(i, 2))) % This knob (m) is one of the possible adjoint knob to the right?
                        x(i, 3) = line(m, 6);  x(i, 4) = line(m, 4);  x(i, 5) = line(m, 7); check = check -1; % Add information to x (block_number,block_type,knob_index)
                    end
                end
                for i=1:col
                    if((line(m, 1) == y(i, 1)) && (line(m, 2) == y(i, 2))) % This knob (m) is one of the possible adjoint knob to the top?
                        y(i, 3) = line(m, 6);  y(i, 4) = line(m, 4);  y(i, 5) = line(m, 7); check = check -1; % Add information to y (block_number,block_type,knob_index)
                    end
                end
            end
        end

        %Save in X matrix the adjoin knobs information
        for i = 1:row
            if(x(i,3)~=-1)
                if(i == 1)
                    X(count_x,1:7) = [1, line(n, 6), line(n, 4), x(i, 3), x(i, 4), x(i, 5), i]; % Add info to X matrix
                    count_x = count_x + 1;
                elseif(x(i,3) == x((i-1),3))
                    if(X(count_x-1,7)<10)
                        X(count_x-1,7) = X(count_x-1,7)*10;
                    end
                    X(count_x-1,7) = floor(X(count_x-1,7)/10)*10 + i;
                else
                    X(count_x,1:7) = [1, line(n, 6), line(n, 4), x(i, 3), x(i, 4), x(i, 5), i]; % Add info to X matrix
                    count_x = count_x + 1;
                end
            end
        end
        
        %Save in Y matrix the adjoin knobs information
        for i = 1:col
            if(y(i,3)~=-1)
                if(i == 1)
                    Y(count_y,1:7) = [2, line(n, 6), line(n, 4), y(i, 3), y(i, 4), y(i, 5), i]; % Add info to Y matrix
                    count_y = count_y + 1;
                elseif(y(i,3) == y((i-1),3))
                    if(Y(count_y-1,7)<10)
                        Y(count_y-1,7) = Y(count_y-1,7)*10;
                    end
                    Y(count_y-1,7) = floor(Y(count_y-1,7)/10)*10 + i;
                else
                    Y(count_y,1:7) = [2, line(n, 6), line(n, 4), y(i, 3), y(i, 4), y(i, 5), i]; % Add info to Y matrix
                    count_y = count_y + 1;
                end
            end
        end
    end    
end

%Assemble A matrix from X and Y
if((count_x ~= 1) && (count_y ~= 1)) % If there is something in X or Y matrix
    adjoint_blocks = [X(1:(count_x-1), 1:7); Y(1:(count_y-1), 1:7)];
elseif(count_x ~= 1) % If there is something only in X matrix
    adjoint_blocks = X(1:(count_x-1), 1:7);
elseif(count_y ~= 1) % If there is something only in Y matrix
    adjoint_blocks = Y(1:(count_y-1), 1:7);
else % There are no adjacent blocks
    adjoint_blocks = -1;
end

%Correcting some conventions to be used in the next functions
if(adjoint_blocks(1) ~= -1)
    for i = 1 : size(adjoint_blocks,1)
        if(adjoint_blocks(i,7)<10) % If the adjoin block 1 is only one number duplicate this number (x -> xx)
            adjoint_blocks(i,7) = adjoint_blocks(i,7)*10+adjoint_blocks(i,7);
        end
        [col,row] = col_row_converter(adjoint_blocks(i, 7)); % Block number of columns and rows        
        d = adjoint_blocks(i,6) + abs(col-row);
        adjoint_blocks(i,6) = adjoint_blocks(i,6)*10 + d; % Second block knob
    end 
end