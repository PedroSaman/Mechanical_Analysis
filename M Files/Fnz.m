function Fnz = Fnz(join)
%Find the normal forces position in the z axis of blocks that are in the second layer of higher of the block model in regard to its own coordinate axis.
%This function is working for up to 9x9 blocks (remove part is missing)
%input: join:(*,upper_block_type,upper_block_number,upper_knob_index,lower_block_type,lower_block_number,lower_knob_index)
%        *: Number of other pair of snaped togheter knobs that are from the same upper and lower blocks
%output: Fnz:(Force_Number, Block_Number 1, 0, x1, y1, z1, 1, Block_Number 2, 0, x2, y2, z2, -1)

loop = size(join,1);
All_Forces = [];
bc = zeros(9,9);
for n = 1 : loop %Search every block type that exist in the join input to add to All_Forces matrix
    %Upper Block
    col = floor(join(n, 2)/10);                               %block number of collums
    row = round(10*((join(n, 2)/10)-floor(join(n, 2)/10)));   %block number of rows
    if(bc(col,row) == 0) %If the first block type is not in the All_Forces matrix yet
        bc(col,row) = 1; %Mark as visited
        Pnz = force_position(col,row,"fnz"); %Load the forces
        count = 1;
        for i = 1 : col %Find all combination of different forces that exist in a block and add a identifier in the All_Forces matrix
            for j = 1 : row 
                for l = 0 : col
                    if(i+l > col) 
                        break;
                    end
                    for k = 0 : row
                        if(k+j > row) 
                            break;
                        end
                        All_Forces = [All_Forces;Pnz(count:count+3,:),[10*col+row;10*(l+1)+i+l;10*(k+1)+j+k;0]]; %Add the force to the matrix with block type and adjoin type information
                        count = count + 4;
                    end
                end
            end
        end
    end
    %Lower Block
    col = floor(join(n, 5)/10);                             %block number of collums
    row = round(10*((join(n, 5)/10)-floor(join(n, 5)/10))); %block number of rows
    if(bc(col,row) == 0) %If the first block type is not in the All_Forces matrix yet
        bc(col,row) = 1; %Mark as visited
        Pnz = force_position(col,row,"fnz"); %Load the forces
        count = 1;
        for i = 1 : col %Find all combination of different forces that exist in a block and add a identifier in the All_Forces matrix
            for j = 1 : row 
                for l = 0 : col
                    if(i+l > col)
                        break;
                    end
                    for k = 0 : row
                        if(k+j > row)
                            break;
                        end
                        All_Forces = [All_Forces;Pnz(count:count+3,:),[10*col+row;10*(l+1)+i+l;10*(k+1)+j+k;0]]; %Add the force to the matrix with block type and adjoin type information
                        count = count + 4;
                    end
                end
            end
        end
    end
end

Z = zeros(4*loop, 2); %This Z matrix stores the block number of each connection
force_z = 1; n = 1;
while(n<=loop)
    Z(force_z:(force_z + 3), 1:2) = [join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6)];
    force_z = force_z + 4;
    n = n + join(n, 1);
end

force_z = force_z - 1;
Fnz = zeros(force_z, 13);  Fnz(1:force_z, 1) = 1:force_z; %Force_Number
Fnz(1:force_z, 7:6:13) = [ones(force_z, 1), -ones(force_z, 1)]; %Force direction for each block snaped
Fnz(1:force_z, 2:6:8) = Z(1:force_z, 1:2); %Block_Number
n = 1;
count = 1;
while(n <= loop) %Add to Fnz the correct force position of blocks touching in the Z axis
    
    upper_type = join(n, 2); %Upper block type
    upper_row = round(10*((upper_type/10)-floor(upper_type/10))); %Upper block number of rows
    lower_type = join(n, 5); %Lower block_type
    lower_row = round(10*((lower_type/10)-floor(lower_type/10))); %Lower block number of rows
    same_pair = join(n,1); %How many knobs share the same pair of connecting blocks
    upper_knob_1 = join(n, 4); %First upper knob_index
    upper_knob_2 = join(n+same_pair-1, 4); %Last upper knob_index
    lower_knob_1 = join(n, 7); %First lower knob_index
    lower_knob_2 = join(n+same_pair-1, 7); %Last lower knob_index
    
    %This functions calculate how the upper block is touching the lower 
    %block (this is necessary to find which is the correct force to add to Fnz)
    upper_row_i = rem(upper_knob_1,upper_row);
    if(upper_row_i == 0)
        upper_row_i = upper_row;
    end
    upper_col_i = (upper_knob_1 - upper_row_i)/upper_row + 1;
    
    upper_row_f = rem(upper_knob_2,upper_row);
    if(upper_row_f == 0)
        upper_row_f = upper_row;
    end
    upper_col_f = (upper_knob_2 - upper_row_f)/upper_row + 1;
    upper_width = 10*upper_col_i + upper_col_f;
    upper_height = 10*upper_row_i + upper_row_f;
    
    %After finding how the upper block is disposed, add it to Fnz
    i = 1;
    while(i<size(All_Forces,1))
        if(upper_type == All_Forces(i,4) && upper_width == All_Forces(i+1,4) && upper_height == All_Forces(i+2,4))
            Fnz(count:count+3, 4:6) = [All_Forces(i:i+3,1), All_Forces(i:i+3,2), -All_Forces(i:i+3,3)];
            break;
        end
        i = i + 4;
    end
    
    %This functions calculate how the lower block is touching the upper 
    %block (this is necessary to find which is the correct force to add to Fnz)
    lower_row_i = rem(lower_knob_1,lower_row);
    if(lower_row_i == 0)
        lower_row_i = lower_row;
    end
    lower_col_i = (lower_knob_1 - lower_row_i)/lower_row + 1;
    
    lower_row_f = rem(lower_knob_2,lower_row);
    if(lower_row_f == 0)
        lower_row_f = lower_row;
    end
    lower_col_f = (lower_knob_2 - lower_row_f)/lower_row + 1;
    lower_width = 10*lower_col_i + lower_col_f;
    lower_height = 10*lower_row_i + lower_row_f;
    
    %After finding how the lower block is disposed, add it to Fnz
    i = 1;
    while(i<size(All_Forces,1))
        if(lower_type == All_Forces(i,4) && lower_width == All_Forces(i+1,4) && lower_height == All_Forces(i+2,4))
            Fnz(count:count+3, 10:12) = All_Forces(i:i+3,1:3);
            break;
        end
        i = i + 4;
    end
    
    count = count + 4;
    n = n + same_pair;
end