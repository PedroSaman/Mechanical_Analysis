function F = Ff_0_180723(model)
%Compute the friction force position for blocks in the first layer
%input: model:(Block_Number, x, y, z, type, color)
%output: Ff = (Force_Number, Block_Number, Knob_Number, x, y, z, ub/lb, ...)
%             ub/lb: upper and lower bounds information.

All_Forces = [];
bc = zeros(9,9);
loop = size(model,1);
count = 0;
for n = 1 : loop
    if(model(n, 4) == 1)
        count = count + 1;
        [col,row] = col_row_converter(model(n, 5));
        if(bc(col,row) == 0) % If the first block type is not in the All_Forces matrix yet
            bc(col,row) = 1; % Mark as visited
            All_Forces = [All_Forces;force_position(col,row,"ff")];
        end
    else
       break;
    end
end

force = 1;
max_force_number = 80; % Maximum number of forces that can appear in a block. (for a 2x9 block is 80)
F = zeros(count*max_force_number, 13);
for n = 1 : count
    [col,row] = col_row_converter(model(n, 5)); % Block number of columns and rows
    i = 1;
    while(i<size(All_Forces,1))
        if(model(n, 5) == All_Forces(i,4))
            start = force;
            j = 1;
            while(j <= col*row)
                force_number = All_Forces(i+2,4) - 1;
                F(start:start+force_number, 4:6) = All_Forces(i:i+force_number,1:3); %Force X, Y and Z Position
                F(start:start+force_number,2) = model(n, 1); %Block Number
                
                start = start + force_number + 1;
                j = j + 1;
                i = i + 4;
            end
            
            break;
        end
        i = i + 4;
    end

    if(col > row) % To correctly count the force numbers, row number needs to be higher than col
        temp_c = row;
        temp_r = col;
    else
        temp_c = col;
        temp_r = row;
    end
    
    block_force = 0; % Number of forces in the current block
    for i = 1:temp_c % Count the number of friction forces acting in the nth block
        if(col == 1 && row == 1) % 1x1 block have 8 friction forces
            block_force = 4;
            break;
        end
        for j = 1:temp_r
            if(j == 1 || j == temp_r) % The first and the last knob in a row have 6 friction forces
                block_force = block_force + 3;
            else
                block_force = block_force + 2; % The other ones have 4 friction forces
            end
        end
    end
    
    counter = 0; 
    if(row>=col) % If the block have more rows than columns or an equal number
        for i = 1:col 
            for j = 1:row
                if(row == 1 && col == 1)
                    F(force+counter:force+counter+3, 3) = ones(1,4); % Knobs that have 4 forces (1x1 blocks)
                    F(force+counter:force+counter+3, 7) = [1; 2; -2; -1];
                    counter = counter + 4;
                elseif(j == 1)
                    F(force+counter:force+counter+2, 3) = [row*(i-1)+j,row*(i-1)+j,row*(i-1)+j]; % Knobs that have 3 forces
                    F(force+counter:force+counter+2, 7) = [1; 2; -1];
                    counter = counter + 3;
                elseif(j == row)
                    F(force+counter:force+counter+2, 3) = [row*(i-1)+j,row*(i-1)+j,row*(i-1)+j]; % Knobs that have 3 forces
                    F(force+counter:force+counter+2, 7) = [1; -2; -1];
                    counter = counter + 3;
                else
                    F(force+counter:force+counter+1, 3) = [row*(i-1)+j,row*(i-1)+j]; % Knobs that have 2 forces
                    F(force+counter:force+counter+1, 7) = [1; -1];
                    counter = counter + 2;
                end
            end
        end
    else % If the block have more columns than rows
        for i = 0:row-1 
           for j = 1:col
              if(j == 1)
                  F(force+counter:force+counter+2, 3) = [(row*j)+1-1,(row*j)+1-1,(row*j)+1-1]; % Knobs that have 3 forces
                  F(force+counter:force+counter+2, 7) = [1; 2; -2];
                  counter = counter + 3;
              elseif(j == col)
                  F(force+counter:force+counter+2, 3) = [(row*j)+1-1,(row*j)+1-1,(row*j)+1-1]; % Knobs that have 3 forces
                  F(force+counter:force+counter+2, 7) = [2; -2; -1];
                  counter = counter + 3;
              else
                  F(force+counter:force+counter+1, 3) = [(row*j)+1-1,(row*j)+1-1]; % Knobs that have 2 forces
                  F(force+counter:force+counter+1, 7) = [2; -2];
                  counter = counter + 2;
              end
           end
        end
    end
    F(force+block_force:force+2*block_force-1,:) = F(force:force+block_force-1,:); %Double the friction force
    F(force+block_force:force+2*block_force-1,6) = -0.5;
    F(force:force+2*block_force-1) = 1:2*block_force;
    force = force + 2*block_force; % Update force value
end

F = F(1:force-1, :);