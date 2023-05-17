function beq = Pushing_Forces(model, N, T)
% Compute the force and torque that the model last block insertion would
% apply in the structure.
% input:  model:(block_number, x, y, z, Block_type, color)
%        N:(number of the block being inserted)
%        T:(maximum friction force to snap out one connection point)
% output: beq:(Block_Number, Fx, Fy, Fz, Mx, My, Mz)

    %Information on the block being inserted (Upper Block)
    model_pushing = model(N, 1:6); % Block being inserted information
    knob_pushing = knob(model_pushing,0); % Block being inserted knob information
    z_max = model_pushing(4); % Height of the inserted block

    %Return if the model has only blocks in the 1st layer
    if(z_max == 1)
        beq = [];
       return;
    end

    %Find out which knob from the previous layer is snapped with the block being inserted
    knob_push = knob(model, z_max - 1); % Previous layer knobs information
    knob_push = [knob_push; knob_pushing]; % Merge last layer knobs and the inserted block
    join_push = join(knob_push, z_max); % Find out the knobs that are snapped together

    %Get the upper block forces position
    [col,row] = col_row_converter(model(N, 5));
    UpperBlock_Forces = force_position(col,row,"ff");

    %Fills the All_Forces Matrix
    bc = zeros(9,9);
    loop = size(model,1);
    All_Forces = [];
    for n = 1 : loop
        if(model(n, 4) == z_max)
            break;
        elseif(model(n, 4) < z_max-1)
            continue;
        end
        [col,row] = col_row_converter(model(n, 5));
        if(bc(col,row) == 0) %If the first block type is not in the All_Forces matrix yet
            bc(col,row) = 1; %Mark as visited
            All_Forces = [All_Forces;force_position(col,row,"ffc")];
        end
    end

    %Find the number of blocks and knobs being pushed and create a matix with the block number
    block_number = 0; knob_number = size(join_push,1);
    b = zeros(knob_number,1);
    for n = 1 : knob_number
        if(n == 1)
           block_number = 1;
           b(1) = join_push(1, 6);
        else
            check = 0;
            for m = 1 : (n - 1)
               if(join_push(n, 6) == b(m))
                  check = -1; 
               end
            end
            if(check == 0)
                block_number = block_number + 1;
                b(block_number) = join_push(n, 6);
            end
        end
    end

    %Calculate the forces and momentos to all blocks caused by the insertion
    beq = zeros(block_number, 7);
    for m = 1 : block_number %for each block that is being applied force
        block = b(m);
        beq(m, 1) = block;
        Mx = 0; My = 0;
        for n = 1 : knob_number %for each knob that is being aplied force
            type_1 = join_push(n, 2); %upper block type
            knob_1 = join_push(n, 4); %upper knob_index

            [~,row] = col_row_converter(type_1);
            upper_row_i = rem(knob_1,row);
            if(upper_row_i == 0)
                upper_row_i = row;
            end
            upper_col_i = (knob_1 - upper_row_i)/row + 1;
            knob1 = 10*upper_col_i + upper_row_i;

            i = 1;
            while(i<size(UpperBlock_Forces,1))
                if(type_1 == UpperBlock_Forces(i,4) && knob1 == UpperBlock_Forces(i+1,4))
                    exclude_knob_1 = UpperBlock_Forces(i+3,3);
                    exclude_knob_2 = UpperBlock_Forces(i+3,4);
                    knobs = [1, 2, 3, 4];
                    if(exclude_knob_2 ~= 0)
                        knobs(exclude_knob_2) = [];
                        if(exclude_knob_1 ~= 0)
                            knobs(exclude_knob_1) = [];
                        end
                    end
                    T_push = UpperBlock_Forces(i+2,4);
                    break;
                end
                i = i + 4;
            end
            if(join_push(n, 6) ==  block)
                type_2 = join_push(n, 5); %lower block type
                knob_2 = join_push(n, 7); %lower knob_index
                [~,row] = col_row_converter(type_2);
                upper_row_i = rem(knob_2,row);
                if(upper_row_i == 0)
                    upper_row_i = row;
                end
                upper_col_i = (knob_2 - upper_row_i)/row + 1;
                knob2 = 10*upper_col_i + upper_row_i;

                i = 1;
                while(i<size(All_Forces,1))
                    if(type_2 == All_Forces(i,4) && knob2 == All_Forces(i+1,4))
                        Force = All_Forces(i:i+3,1:3);
                        for j = 1 : T_push
                            Mx = Mx + Force(knobs(j), 2) * T / T_push;
                            My = My - Force(knobs(j), 1) * T / T_push;
                        end
                        break;
                    end
                    i = i + 4;
                end
                beq(m, 4:6) = [beq(m, 4) + T, beq(m, 5) + Mx, beq(m, 6) + My];
                Mx = 0;  My = 0;
            end
        end
    end
end