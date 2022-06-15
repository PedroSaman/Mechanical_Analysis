function Fny = Fny(adjoin)
%Compute the Y axis normal forces position for each pair of blocks in the input
%input: adjoin = (2,own block_number, own block_type, adjacent block_number, adjacent block_type, adjacent knob_index, adjoin type)
%output: Fny = (Force_Number, Block_Number 1, Knob_Number 1, x1, y1, z1, -1, Block_Number 2, Knob_Number 2, x2, y2, z2, 1)

if(adjoin == -1)
    Fny = [];
else
    loop = size(adjoin,1);
    ny = loop; %Number of contact surfaces in the y direction 
    for n = loop(1) : -1 : 1
        if(adjoin(n, 1) == 1) %Remove contact surfaces in the x direction from adjoin 
            adjoin(n, :) = [];
            ny = ny - 1;
        end
    end

    force_y = 4 * ny; %Number of Fny forces
    if(force_y == 0) %if all contact surfaces in adjoin are in the x direction 
        Fny = []; 
    else
        
        % Create the All_Forces matrix
        All_Forces = [];
        bc = zeros(9,9);
        for n = 1 : ny % For every adjoin block duo
            %First Block
            [col,row] = col_row_converter(adjoin(n, 3)); % Block number of columns and rows
            if(bc(col,row) == 0) % If the first block type is not in the All_Forces matrix
                bc(col,row) = 1; % Mark as visited
                All_Forces = [All_Forces;force_position(col,row,"fny")];            
            end
            %Second Block
            [col,row] = col_row_converter(adjoin(n, 5)); % Block number of columns and rows
            if(bc(col,row) == 0) % If the first block type is not in the All_Forces matrix
                bc(col,row) = 1; % Mark as visited
                All_Forces = [All_Forces;force_position(col,row,"fny")];
            end
        end

        Fny = zeros(force_y, 13);  Fny(1:force_y, 1) = 1:force_y; 
        Fny(:, 7:6:13) = [-ones(force_y, 1), ones(force_y, 1)];  % Set column 7 as -1 and column 13 as 1. This is the force orientation convention
        for n = 1 : ny %For each adjoin block compute the Y axis normal forces location
            Fny((4*n-3):4*n, 2:6:8) = [ones(4, 1) * adjoin(n, 2), ones(4, 1) * adjoin(n, 4)]; % Set column 2 as the lower block and column 8 as the upper block
            type_1 = adjoin(n, 3); % Lower block type
            type_2 = adjoin(n, 5); % Upper block type
            adjoin_type_1 = adjoin(n, 7); % Lower block adjoin type
            adjoin_type_2 = adjoin(n, 6); % Upper block adjoin type

            i = 1;
            while(i<size(All_Forces,1))
                if(type_1 == All_Forces(i,4) && adjoin_type_1 == All_Forces(i+1,4))
                    Fny((4*n-3):4*n, 4:6) = All_Forces(i:i+3,1:3);
                    break;
                end
                i = i + 4;
            end
            
            i = 1;
            while(i<size(All_Forces,1))
                if(type_2 == All_Forces(i,4) && adjoin_type_2 == All_Forces(i+1,4))
                    Fny((4*n-3):4*n, 10:12) = [All_Forces(i:i+3,1), -All_Forces(i:i+3,2), All_Forces(i:i+3,3)];
                    break;
                end
                i = i + 4;
            end
        end
    end
end