function Ff = Ff_0_180723(model)
%Find the friction force position in blocks that are in the first layer of 
%the block model in regard to its own coordinate axis.
%This function is working for up to 9x9 blocks
%input: model:(Block_Number, x, y, z, type, color)
%output: Ff = (Force_Number, Block_Number, Knob_Number, x, y, z, ub/lb, ...)
%             ub/lb: upper and lower bounds information.
%Obs: Pfnm_xy = position force of a block with n columns and m rows touching
%from the xth to the yth knob. Special case to 0 which represents that it is
%touching the entire block. a "_" after the Pf, mean that this position is 
%the negative one.

All_Forces = [];
bc = zeros(9,9);
loop = size(model,1);
count = 0;
for n = 1 : loop %Count the number of frictional forces for each connecting convex part
    if(model(n, 4) == 1)
        count = count + 1;
        [col,row] = col_row_converter(model(n, 5));
        if(bc(col,row) == 0) %If the first block type is not in the All_Forces matrix yet
            bc(col,row) = 1; %Mark as visited
            All_Forces = [All_Forces;force_position(col,row,"ff")];
        end
    else
       break;
    end
end

force = 1;
max_force_number = 80; %Maximum number of forces that can appear in a block. (for a 2x9 block is 80)
F = zeros(count*max_force_number, 13);
F(1:(count*max_force_number), 1) = 1: (count*max_force_number);  %Force Number
for n = 1 : count %for each block in the 1st layer
    [col,row] = col_row_converter(model(n, 5)); %block number of columns and rows
    i = 1;
    while(i<size(All_Forces,1))
        if(model(n, 5) == All_Forces(i,4))
            start = force;
            j = 1;
            while(j <= col*row)
                force_number = All_Forces(i+2,4) - 1;
                F(start:start+force_number, 4:5) = All_Forces(i:i+force_number,1:2); %Force X and Y Position
                F(start:start+force_number, 6) = -All_Forces(i:i+force_number,3);
                F(start:start+force_number,2) = model(n, 1); %Block Number
                
                start = start + force_number + 1;
                j = j + 1;
                i = i + 4;
            end
            
            break;
        end
        i = i + 4;
    end

    if(col > row) %To correctly count the force numbers, row number needs to be higher than col
        temp_c = row;
        temp_r = col;
    else
        temp_c = col;
        temp_r = row;
    end
    
    block_force = 0; %Number of forces in the current block
    for i = 1:temp_c %Count the number of friction forces acting in the nth block
        if(col == 1 && row == 1) %1x1 block have 8 friction forces
            block_force = 8;
            break;
        end
        for j = 1:temp_r
            if(j == 1 || j == temp_r) %the first and the last knob in a row have 6 friction forces
                block_force = block_force + 6;
            else
                block_force = block_force + 4; %the other ones have 4 friction forces
            end
        end
    end
    
    counter = 0; 
    if(col<=row) %Knob Number
        for i = 1:col %if the block have more rows than collumns
            for j = 1:row
                if(row == 1 && col == 1)
                    F(force+counter:force+counter+3, 3) = ones(1,4); %Knobs that have 4 forces (1x1 blocks)
                    F(force+counter:force+counter+3, 7) = [1; 2; -2; -1];
                    counter = counter + 4;
                elseif(j == 1)
                    F(force+counter:force+counter+2, 3) = [row*(i-1)+j,row*(i-1)+j,row*(i-1)+j]; %Knobs that have 3 forces
                    F(force+counter:force+counter+2, 7) = [1; 2; -1];
                    counter = counter + 3;
                elseif(j == row)
                    F(force+counter:force+counter+2, 3) = [row*(i-1)+j,row*(i-1)+j,row*(i-1)+j]; %Knobs that have 3 forces
                    F(force+counter:force+counter+2, 7) = [1; -2; -1];
                    counter = counter + 3;
                else
                    F(force+counter:force+counter+1, 3) = [row*(i-1)+j,row*(i-1)+j]; %Knobs that have 2 forces
                    F(force+counter:force+counter+1, 7) = [1; -1];
                    counter = counter + 2;
                end
            end
        end
    else 
        for i = 0:row-1 %if the block have more collumns than rows
           for j = 1:col
              if(j == 1)
                  F(force+counter:force+counter+2, 3) = [(row*j)+1-1,(row*j)+1-1,(row*j)+1-1]; %Knobs that have 3 forces
                  F(force+counter:force+counter+2, 7) = [1; 2; -2];
                  counter = counter + 3;
              elseif(j == col)
                  F(force+counter:force+counter+2, 3) = [(row*j)+1-1,(row*j)+1-1,(row*j)+1-1]; %Knobs that have 3 forces
                  F(force+counter:force+counter+2, 7) = [2; -2; -1];
                  counter = counter + 3;
              else
                  F(force+counter:force+counter+1, 3) = [(row*j)+1-1,(row*j)+1-1]; %Knobs that have 2 forces
                  F(force+counter:force+counter+1, 7) = [2; -2];
                  counter = counter + 2;
              end
           end
        end
    end
    F(force+block_force/2:force+block_force,2:7) = F(force:force+block_force/2,2:7); %Double the vector
    F(force+block_force/2:force+block_force,6) = -0.5;
    force = force + block_force; %Update force value
end

force = force - 1; %just adjustment, matlab start its vectors as 1.
Ff = F(1:force, :);