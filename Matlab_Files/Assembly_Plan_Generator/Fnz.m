function Fnz = Fnz(join)
    %Compute the Z axis normal forces position for each pair of blocks in the input
    %input: join:(*,upper_block_type,upper_block_number,upper_knob_index,lower_block_type,lower_block_number,lower_knob_index)
    %             *: Number of other pair of snaped together knobs that are from the same upper and lower blocks
    %output: Fnz:(Force_Number, Block_Number 1, 0, x1, y1, z1, limitation, Block_Number 2, 0, x2, y2, z2, -1)
    %                                           limitation = -3 => this fnz
    %                                           have a upper limit.
    loop = size(join,1);
    All_Forces = [];
    bc = zeros(9,9);
    for n = 1 : loop % Search every block type that exist in the join input to add to All_Forces matrix
        %Upper Block
        [col,row] = col_row_converter(join(n, 2));
        if(bc(col,row) == 0) % If the first block type is not in the All_Forces matrix yet
            bc(col,row) = 1; % Mark as visited
            All_Forces = [All_Forces;force_position(col,row,"fnz")];
        end
        %Lower Block
        [col,row] = col_row_converter(join(n, 5));
        if(bc(col,row) == 0) % If the first block type is not in the All_Forces matrix yet
            bc(col,row) = 1; % Mark as visited
            All_Forces = [All_Forces;force_position(col,row,"fnz")];
        end
    end

    Z = zeros(4*loop, 2); % Store the block number of each connection
    force_z = 1; n = 1;
    while(n<=loop)
        Z(force_z:(force_z + 3), 1:2) = [join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6)];
        force_z = force_z + 4;
        n = n + join(n, 1);
    end

    force_z = force_z - 1;
    Fnz = zeros(force_z, 13);  Fnz(1:force_z, 1) = 1:force_z; % Force_Number
    Fnz(1:force_z, 7:6:13) = [ones(force_z, 1), -ones(force_z, 1)]; % Conventioned force direction
    Fnz(1:force_z, 2:6:8) = Z(1:force_z, 1:2); % Block_Number
    remove = zeros(1, force_z); % Some block connections have to remove one or more forces
    n = 1; count = 1; count_remove = 0;
    while(n <= loop) % For each block connection in the input

        upper_type = join(n, 2); % Upper block type
        [upper_col,upper_row] = col_row_converter(upper_type); % Upper block number of rows and columns
        lower_type = join(n, 5); % Lower block_type
        [~,lower_row] = col_row_converter(lower_type);% Lower block number of rows
        same_pair = join(n,1); % How many knobs share the same pair of connecting blocks
        upper_knob_1 = join(n, 4); % First upper knob index
        upper_knob_2 = join(n+same_pair-1, 4); % Last upper knob index
        lower_knob_1 = join(n, 7); % First lower knob index
        lower_knob_2 = join(n+same_pair-1, 7); % Last lower knob index

        %Find how the upper block is touching the lower block
        %(necessary to find the correct force to add to Fnz)
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

        i = 1;
        while(i<size(All_Forces,1))
            if(upper_type == All_Forces(i,4) && upper_width == All_Forces(i+1,4) && upper_height == All_Forces(i+2,4))
                Fnz(count:count+3, 4:6) = [All_Forces(i:i+3,1), All_Forces(i:i+3,2), -All_Forces(i:i+3,3)];

                %Verify if it is necessary to remove any force
                %This became quite complicated, but it is only searching for possible configurations of block connection
                %that one of the 4 Z axis normal forces does not exist
                if(upper_type == 22 && upper_knob_1 == upper_knob_2) % If only one knob is from the same block && upper block is 2x2
                    count_remove = count_remove + 1;
                    remove(count_remove) = count+4-upper_knob_1;
                elseif(upper_col> 1 && upper_row > 1) % If not nx1 or 1xn (has at least 2 knobs in each direction)
                    if(upper_col > 2) % If it is the columns that have more than 2 knobs (2xn)
                        if(upper_row_i == upper_row_f && upper_col_f-upper_col_i ~= upper_col - 1) % If only one row (of the two) is snapped && not all columns knobs is snaped 
                            if(upper_row_i == 1) % If the row with snaped knobs are the 1st one
                                if(upper_col_i ~= 1 && upper_col_f ~= upper_col) % If the first && the last column knobs are not snaped
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+1; % Force 2 is removed
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+3; % Force 4 is removed
                                elseif(upper_col_i ~= 1) % If the first column knob is not snaped 
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+1; % Force 2 is removed
                                elseif(upper_col_f ~= upper_col) % If the last column knob is not snaped 
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+3; % Force 4 is removed
                                end
                            elseif(upper_row_i == 2) % If the row with snaped knobs are the 2nd one
                                if(upper_col_i ~= 1 && upper_col_f ~= upper_col) % If the first && the last column knobs are not snaped
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count; % Force 1 is removed
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+2; % Force 3 is removed
                                elseif(upper_col_i ~= 1) % If the first column knob is not snaped 
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count; % Force 1 is removed
                                elseif(upper_col_f ~= upper_col) % If the last column knob is not snaped 
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+2; % Force 3 is removed
                                end
                            end
                        end
                    elseif(upper_row > 2) % If it is the rows that have more than 2 knobs
                        if(upper_col_i == upper_col_f && upper_row_f-upper_row_i ~= upper_row - 1) % If only one column (of the two) is snapped && not all rows knobs is snaped 
                            if(upper_col_i == 1) % If the column with snaped knobs are the 1st one
                                if(upper_row_i ~= 1 && upper_row_f ~= upper_row) % If the first && the last row knobs are not snaped
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+2; % Force 3 is removed
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+3; % Force 4 is removed
                                elseif(upper_row_i ~= 1) % If the first row knob is not snaped 
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+2; % Force 3 is removed
                                elseif(upper_row_f ~= upper_row) % If the last row knob is not snaped 
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+3; % Force 4 is removed
                                end
                            elseif(upper_col_i == 2) % If the column with snaped knobs are the 2nd one
                                if(upper_row_i ~= 1 && upper_row_f ~= upper_row) % If the first && the last row knob is not snaped
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count; % Force 1 is removed
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+1; % Force 2 is removed
                                elseif(upper_row_i ~= 1) % If the first row knob is not snaped 
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count; % Force 1 is removed
                                elseif(upper_row_f ~= upper_row) % If the last row knob is not snaped 
                                    count_remove = count_remove + 1;
                                    remove(count_remove) = count+1; % Force 2 is removed
                                end
                            end
                        end
                    end
                end
                break;
            end
            i = i + 4;
        end

        %Find how the lower block is touching the upper block 
        %(necessary to find the correct force to add to Fnz)
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

    %Remove the necessary Fnz forces %remove(1:end) = [];%
    remove(count_remove+1:end) = [];
    
    if(size(remove ~= 0))
        while(count_remove ~= 0)
            Fnz(remove(count_remove),3) = -3;
            count_remove = count_remove - 1;
        end
    end

    %Fnz(:,1) = 1:size(Fnz,1);
end