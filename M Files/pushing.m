function beq = pushing(model, N)
%Calculate the forces and torques that the insertion of the last block
%in the model would applies in the structure.
%input:  model:(block_number, x, y, z, Block_type, color)
%        N:(number of the block being inserted) 
%output: beq:(Block_Number, Fx, Fy, Fz, Mx, My, Mz)

%constant numbers
g = 9.81; T=70*g;

%Not generic, use force_position(col,row,"ff") insted
% 1*1
pf1 = [-1.25,     0, 1.5];  pf2 = [    0, -1.25, 1.5];  pf3 = [    0,  1.25, 1.5];  pf4 = [ 1.25,     0, 1.5];
Pf11 = [pf1; pf2; pf3; pf4];
%1*2
pf1 = [-1.25,    -2, 1.5];  pf2 = [    0, -3.25, 1.5];  pf3 = [    0, -0.75, 1.5];  pf4 = [ 1.25,    -2, 1.5];
Pf12_1 = [pf1; pf2; pf3; pf4];
pf1 = [-1.25,     2, 1.5];  pf2 = [    0,  0.75, 1.5];  pf3 = [    0,  3.25, 1.5];  pf4 = [ 1.25,     2, 1.5];
Pf12_2 = [pf1; pf2; pf3; pf4];
%2*1
pf1 = [-3.25,     0, 1.5];  pf2 = [   -2, -1.25, 1.5];  pf3 = [   -2,  1.25, 1.5];  pf4 = [-0.75,     0, 1.5];
Pf21_1 = [pf1; pf2; pf3; pf4];
pf1 = [0.75,     0, 1.5];  pf2 = [   2, -1.25, 1.5];  pf3 = [   2,  1.25, 1.5];  pf4 = [3.25,     0, 1.5];
Pf21_2 = [pf1; pf2; pf3; pf4];
%2*2
pf1 = [-3.25,    -2, 1.5];  pf2 = [   -2, -3.25, 1.5];  pf3 = [   -2, -0.75, 1.5];  pf4 = [-0.75,    -2, 1.5];
Pf22_1 = [pf1; pf2; pf3; pf4];
pf1 = [-3.25,    2, 1.5];  pf2 = [   -2, 0.75, 1.5];  pf3 = [   -2, 3.25, 1.5];  pf4 = [-0.75,    2, 1.5];
Pf22_2 = [pf1; pf2; pf3; pf4];
pf1 = [0.75,    -2, 1.5];  pf2 = [   2, -3.25, 1.5];  pf3 = [   2, -0.75, 1.5];  pf4 = [3.25,    -2, 1.5];
Pf22_3 = [pf1; pf2; pf3; pf4];
pf1 = [0.75,    2, 1.5];  pf2 = [   2, 0.75, 1.5];  pf3 = [    2 3.25, 1.5];  pf4 = [3.25,    2, 1.5];
Pf22_4 = [pf1; pf2; pf3; pf4];

All_Forces = [];
bc = zeros(9,9);
loop = size(model,1);
for n = 1 : loop %Number of blocks in the model. Space to improve here, search only the blocks in the necessary layers
    [col,row] = col_row_converter(model(n, 5));
    if(bc(col,row) == 0) %If the first block type is not in the All_Forces matrix yet
        bc(col,row) = 1; %Mark as visited
        All_Forces = [All_Forces;force_position(col,row,"ff")];
    end
end
All_Forces(:,3) = -All_Forces(:,3); %All forces hardcoded is correct here (erase this after)

%Information on the block to be pushed (Upper Block)
model_pushing = model(N, 1:6); %Block being inserted information
knob_pushing = knob(model_pushing,0); %Knobs information of the block being inserted
z_max = model_pushing(4); %Height of the inserted block

%Information on the block to be pushed into (Lower Block)
knob_push = knob(model, z_max - 1); %Previous layer knobs information

%Find the convex part to be pushed in
knob_push = [knob_push; knob_pushing]; %Merge last layer knobs and the inserted block
join_push = join(knob_push, z_max); %Find out the knobs that are snaped together

%Find the number of blocks to which the pushing force is applied, block_number, and the number of convex parts knob_number (1). 
block_number = 0;  knob_number = size(join_push,1);
b = zeros(knob_number);
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

%Ask for beq 
beq = zeros(block_number, 7);
for m = 1 : block_number %for each block that is being applied force
    block = b(m);
    beq(m, 1) = block;
    Mx = 0; My = 0;
    for n = 1 : knob_number %for each knob that is being aplied force
        
        %%Start pedro way 
        type_1 = join_push(n, 2); %upper block type
        knob_1 = join_push(n, 4); %upper knob_index
        type_2 = join_push(n, 5); %lower block type
        knob_2 = join_push(n,7); %lower knob_index
        
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
        
        i = 0;
        while(i<size(All_Forces,1)) %complicado aqui. tenho q saber quais forcas pular com o type1 mas as forcas com o type2
            if(type_1 == All_Forces(i,4) && knob1 == All_Forces(i+1,4))
                T_push = All_Forces(i+2,4);
                Forces = All_Forces(i:i+T_push-1,1:3); %Select the forces that exist for this block type for this knob
                break;
            end
            i = i + 4;
        end
        
        i = 0;
        while(i<size(All_Forces,1))
            if()
                for l = 1 : T_push
                    Mx = Mx + Forces(l, 2) * T / T_push;
                    My = My - Forces(l, 1) * T / T_push;
                end
            end
        end
        %Finish pedro way
        
        type_1 = join_push(n, 2); %upper block type
        knob_1 = join_push(n, 4); %upper knob_index
        %not generic LOOK AT THIS
        if(type_1 == 11)
            T_push = 4;
            knobs = [1, 2, 3, 4];
        elseif(type_1 == 12)
            T_push = 3;
            if(knob_1 == 1) 
                knobs = [1, 2, 4];
            elseif(knob_1 == 2)
                knobs = [1, 3, 4];
            end
        elseif(type_1 == 21)
            T_push = 3;
            if(knob_1 == 1) 
                knobs = [1, 2, 3];
            elseif(knob_1 == 2)
                knobs = [2, 3, 4];
            end
        else
            T_push = 2;
            if(knob_1 == 1) 
                knobs = [1, 2];
            elseif(knob_1 == 2)
                knobs = [1, 3];
            elseif(knob_1 == 3) 
                knobs = [2, 4];
            elseif(knob_1 == 4)
                knobs = [3, 4];
            end
        end
        if(join_push(n, 6) ==  block)
            type_2 = join_push(n, 5);
            knob_2 = join_push(n,7);
            %not generic LOOK AT THIS
            if(type_2 == 11)
                for l = 1 : T_push
                    Mx = Mx + Pf11(knobs(l), 2) * T / T_push;
                    My = My - Pf11(knobs(l), 1) * T / T_push;
                end
            elseif(type_2 == 12)
                if(knob_2 == 1)
                    for l = 1 : T_push
                        Mx = Mx + Pf12_1(knobs(l), 2) * T / T_push;
                        My = My - Pf12_1(knobs(l), 1) * T / T_push;
                    end
                elseif(knob_2 == 2)
                    for l = 1 : T_push
                        Mx = Mx + Pf12_2(knobs(l), 2) * T / T_push;
                        My = My - Pf12_2(knobs(l), 1) * T / T_push;
                    end
                end
            elseif(type_2 == 21)
                if(knob_2 == 1)
                    for l = 1 : T_push
                        Mx = Mx + Pf21_1(knobs(l), 2) * T / T_push;
                        My = My - Pf21_1(knobs(l), 1) * T / T_push;
                    end
                elseif(knob_2 == 2)
                    for l = 1 : T_push
                        Mx = Mx + Pf21_2(knobs(l), 2) * T / T_push;
                        My = My - Pf21_2(knobs(l), 1) * T / T_push;
                    end
                end
            else
                if(knob_2 == 1)
                    for l = 1 : T_push
                        Mx = Mx + Pf22_1(knobs(l), 2) * T / T_push;
                        My = My - Pf22_1(knobs(l), 1) * T / T_push;
                    end
                elseif(knob_2 == 2)
                    for l = 1 : T_push
                        Mx = Mx + Pf22_2(knobs(l), 2) * T / T_push;
                        My = My - Pf22_2(knobs(l), 1) * T / T_push;
                    end
                elseif(knob_2 == 3)
                    for l = 1 : T_push
                        Mx = Mx + Pf22_3(knobs(l), 2) * T / T_push;
                        My = My - Pf22_3(knobs(l), 1) * T / T_push;
                    end
                elseif(knob_2 == 4)
                    for l = 1 : T_push
                        Mx = Mx + Pf22_4(knobs(l), 2) * T / T_push;
                        My = My - Pf22_4(knobs(l), 1) * T / T_push;
                    end
                end
            end
            beq(m, 4:6) = [beq(m, 4) + T, beq(m, 5) + Mx, beq(m, 6) + My];
            Mx = 0;  My = 0;
        end
    end
end