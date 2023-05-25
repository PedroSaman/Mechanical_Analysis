function Ff = Ff_180723(join)
%Compute the friction force position for blocks in the second layer or higher
%input: join = (1 or 2,upper_block_type,upper_block_number,upper_knob_index,lower_block_type,lower_block_number,lower_knob_index)
%               *: Number of other pair of snaped together knobs that are from the same block as the first pair
%output: Ff = (Force_Number, Upper Block_Number, Upper Knob_Number , x1, y1, z1, *, Lower Block_Number, Lower Knob_Number, x2, y2, z2, 0)
%             *: upper and lower limits of the normal force

All_Forces = [];
bc = zeros(9,9);
All_Forces2 = [];
bc2 = zeros(9,9);
loop = size(join,1);
for n = 1 : loop
    %Upper Block
    [col,row] = col_row_converter(join(n, 2));
    if(bc(col,row) == 0) %If the first block type is not in the All_Forces matrix yet
        bc(col,row) = 1; %Mark as visited
        All_Forces = [All_Forces;force_position(col,row,"ff")];
    end
    %Lower Block
    [col,row] = col_row_converter(join(n, 5));
    if(bc2(col,row) == 0) %If the first block type is not in the All_Forces matrix yet
        bc2(col,row) = 1; %Mark as visited
        All_Forces2 = [All_Forces2;force_position(col,row,"ffc")];
    end
end

Ff = zeros(4*loop, 13); % Maximum possible number of friction force (If all blocks were 1x1)
Ff(1:4*loop, 1) = 1 : 4*loop; % Force Number
count = 1; 
for n = 1 : loop % For each knob connection
    type_1 = join(n, 2); % Upper block type
    type_2 = join(n, 5); % Lower block type
    knob_1 = join(n, 4); % Upper knob index
    knob_2 = join(n, 7); % Lower knob index
    
    [~,row] = col_row_converter(type_1);
    upper_row_i = rem(knob_1,row);
    if(upper_row_i == 0)
        upper_row_i = row;
    end
    upper_col_i = (knob_1 - upper_row_i)/row + 1;
    knob1 = 10*upper_col_i + upper_row_i;
    
    [~,row] = col_row_converter(type_2);
    upper_row_i = rem(knob_2,row);
    if(upper_row_i == 0)
        upper_row_i = row;
    end
    upper_col_i = (knob_2 - upper_row_i)/row + 1;
    knob2 = 10*upper_col_i + upper_row_i;
    
    exclude_knob_1 = 0;
    exclude_knob_2 = 0;
    i = 1;
    while(i<size(All_Forces,1))
        force_number = All_Forces(i+2,4) - 1;
        if(type_1 == All_Forces(i,4) && knob1 == All_Forces(i+1,4))
            if(type_1 ~= 11)
                exclude_knob_1 = All_Forces(i+3,3);
                exclude_knob_2 = All_Forces(i+3,4);
                if(exclude_knob_1 == 0 && exclude_knob_2 == 3)
                    Ff(count:count+2, 7) = [1; 2; -1];
                elseif(exclude_knob_1 == 0 && exclude_knob_2 == 2)
                    Ff(count:count+2, 7) = [1; -2; -1];
                elseif(exclude_knob_1 == 0 && exclude_knob_2 == 4)
                    Ff(count:count+2, 7) = [1; 2; -2];
                elseif(exclude_knob_1 == 0 && exclude_knob_2 == 1)
                    Ff(count:count+2, 7) = [2; -2; -1];
                elseif(exclude_knob_1 == 1 && exclude_knob_2 == 4)
                    Ff(count:count+1, 7) = [2; -2];
                elseif(exclude_knob_1 == 2 && exclude_knob_2 == 3)
                    Ff(count:count+1, 7) = [1; -1];
                end
            else
                Ff(count:count+3, 7) = [1; 2; -2; -1];
            end
            for j = 0: force_number
                Ff(count+j, 2:6:8) = join(n, 3:3:6); %Block Number
                Ff(count+j, 3:6:9) = join(n, 4:3:7); %Knob Number
            end
            Ff(count:count+force_number, 4:6) = All_Forces(i:i+force_number,1:3); %Force X, Y and Z Position
            break;
        end
        if(force_number == 1)
            force_number = 2;
        end
        i = i + 4;
    end
    i = 1;
    while(i<size(All_Forces2,1))
        if(type_2 == All_Forces2(i,4) && knob2 == All_Forces2(i+1,4))
            Ff(count:count+3, 10:12) = All_Forces2(i:i+3,1:3);
            if(exclude_knob_1 == 0 && exclude_knob_2 == 4)
                Ff(count+3,10:12) = [0,0,0];
            elseif(exclude_knob_1 == 0 && exclude_knob_2 == 3)
                Ff(count+2,10:12) = Ff(count+3,10:12); 
                Ff(count+3,10:12) = [0,0,0];
            elseif(exclude_knob_1 == 0 && exclude_knob_2 == 2)
                Ff(count+1,10:12) = Ff(count+2,10:12); 
                Ff(count+2,10:12) = Ff(count+3,10:12); 
                Ff(count+3,10:12) = [0,0,0];
            elseif(exclude_knob_1 == 0 && exclude_knob_2 == 1)
                Ff(count,10:12) = Ff(count+1,10:12);
                Ff(count+1,10:12) = Ff(count+2,10:12);
                Ff(count+2,10:12) = Ff(count+3,10:12);
                Ff(count+3,10:12) = [0,0,0];
            elseif(exclude_knob_1 == 1 && exclude_knob_2 == 4)
                Ff(count,10:12) = Ff(count+1,10:12);
                Ff(count+1,10:12) = Ff(count+2,10:12);
                Ff(count+2,10:12) = [0,0,0];
                Ff(count+3,10:12) = [0,0,0];
            elseif(exclude_knob_1 == 2 && exclude_knob_2 == 3)
                Ff(count+1,10:12) = Ff(count+3,10:12);
                Ff(count+2,10:12) = [0,0,0];
                Ff(count+3,10:12) = [0,0,0];
            end
            break;
        end
        i = i + 4;
    end
    count = count + force_number + 1;
end
Ff(count:end,:) = [];