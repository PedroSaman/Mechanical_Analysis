function Fnz_remove = Fnz(join)
%Find the normal forces position in the z axis of blocks that are in the second layer of higher of the block model in regard to its own coordinate axis.
%input: join:(*,upper_block_type,upper_block_number,upper_knob_index,lower_block_type,lower_block_number,lower_knob_index)
%        *: Number of other pair of snaped togheter knobs that are from the same block as the first pair
%output: Fnz:(Force_Number, Block_Number 1, Knob_Number 1, x1, y1, z1, 1, Block_Number 2, Knob_Number 2, x2, y2, z2, -1)

% 1x1 Block
pn1 = [-2, -2, 1.5];  pn2 = [-2,  2, 1.5];  pn3 = [ 2, -2, 1.5];  pn4 = [ 2,  2, 1.5];
Pnz11 = [pn1; pn2; pn3; pn4];  Pnz_11 = [Pnz11(1:4, 1:2), -Pnz11(1:4, 3)];

% 1x2 Block
pn1 = [-2, -4, 1.5];  pn2 = [-2,  0, 1.5];  pn3 = [ 2, -4, 1.5];  pn4 = [ 2,  0, 1.5];
Pnz12_1 = [pn1; pn2; pn3; pn4];  Pnz_12_1 = [Pnz12_1(1:4, 1:2), -Pnz12_1(1:4, 3)];
pn1 = [-2,  0, 1.5];  pn2 = [-2,  4, 1.5];  pn3 = [ 2,  0, 1.5];  pn4 = [ 2,  4, 1.5];
Pnz12_2 = [pn1; pn2; pn3; pn4];  Pnz_12_2 = [Pnz12_2(1:4, 1:2), -Pnz12_2(1:4, 3)];
Pnz12_0 = [Pnz12_1(1, 1:3); Pnz12_2(2, 1:3); Pnz12_1(3, 1:3); Pnz12_2(4, 1:3)];
Pnz_12_0 = [Pnz12_0(1:4, 1:2), -Pnz12_0(1:4, 3)];

% 2x1 Block
pn1 = [-4, -2, 1.5];  pn2 = [-4,  2, 1.5];  pn3 = [ 0, -2, 1.5];  pn4 = [ 0,  2, 1.5];
Pnz21_1 = [pn1; pn2; pn3; pn4];  Pnz_21_1 = [Pnz21_1(1:4, 1:2), -Pnz21_1(1:4, 3)];
pn1 = [0, -2, 1.5];  pn2 = [0,  2, 1.5];  pn3 = [4, -2, 1.5];  pn4 = [4,  2, 1.5];
Pnz21_2 = [pn1; pn2; pn3; pn4];  Pnz_21_2 = [Pnz21_2(1:4, 1:2), -Pnz21_2(1:4, 3)];
Pnz21_0 = [Pnz21_1(1:2, 1:3); Pnz21_2(3:4, 1:3)];
Pnz_21_0 = [Pnz21_0(1:4, 1:2), -Pnz21_0(1:4, 3)];

% 2x2 Block
pn1 = [-4, -4, 1.5];  pn2 = [-4,  0, 1.5];  pn3 = [ 0, -4, 1.5];  pn4 = [ 0,  0, 1.5];
Pnz22_1 = [pn1; pn2; pn3; pn4];  Pnz_22_1 = [Pnz22_1(1:4, 1:2), -Pnz22_1(1:4, 3)];
pn1 = [-4, 0, 1.5];  pn2 = [-4, 4, 1.5];  pn3 = [ 0, 0, 1.5];  pn4 = [ 0, 4, 1.5]; 
Pnz22_2 = [pn1; pn2; pn3; pn4];  Pnz_22_2 = [Pnz22_2(1:4, 1:2), -Pnz22_2(1:4, 3)];
pn1 = [0, -4, 1.5];  pn2 = [0,  0, 1.5];  pn3 = [4, -4, 1.5];  pn4 = [4,  0, 1.5];
Pnz22_3 = [pn1; pn2; pn3; pn4];  Pnz_22_3 = [Pnz22_3(1:4, 1:2), -Pnz22_3(1:4, 3)];
pn1 = [0, 0, 1.5];  pn2 = [0, 4, 1.5];  pn3 = [4, 0, 1.5];  pn4 = [4, 4, 1.5];
Pnz22_4 = [pn1; pn2; pn3; pn4];  Pnz_22_4 = [Pnz22_4(1:4, 1:2), -Pnz22_4(1:4, 3)];
Pnz22_0 = [Pnz22_1(1, 1:3); Pnz22_2(2, 1:3); Pnz22_3(3, 1:3); Pnz22_4(4, 1:3)];
Pnz_22_0 = [Pnz22_0(1:4, 1:2), -Pnz22_0(1:4, 3)];
Pnz22_12 = [Pnz22_1(1, 1:3); Pnz22_2(2, 1:3); Pnz22_1(3, 1:3); Pnz22_2(4, 1:3)];
Pnz_22_12 = [Pnz22_12(1:4, 1:2), -Pnz22_12(1:4, 3)];
Pnz22_34 = [Pnz22_3(1, 1:3); Pnz22_4(2, 1:3); Pnz22_3(3, 1:3); Pnz22_4(4, 1:3)];
Pnz_22_34 = [Pnz22_34(1:4, 1:2), -Pnz22_34(1:4, 3)];
Pnz22_13 = [Pnz22_1(1:2, 1:3); Pnz22_3(3:4, 1:3)];
Pnz_22_13 = [Pnz22_13(1:4, 1:2), -Pnz22_13(1:4, 3)];
Pnz22_24 = [Pnz22_2(1:2, 1:3); Pnz22_4(3:4, 1:3)];
Pnz_22_24 = [Pnz22_24(1:4, 1:2), -Pnz22_24(1:4, 3)];

% Replacing the contents of join 
% Sort so that the same combination of blocks connected up and down is lined up 
loop = size(join);
for n = 1 : loop(1)-2
    if( (join(n, 1) == 2) && (join(n+2, 1) == 2) && (join(n+2, 2) == 22) && (join(n, 2) == 22) && (join(n, 6) == join(n+2, 6)) && (join(n,3) == join(n+2, 3)) )
        dummy = join(n+1, 1:7);
        join(n+1, 1:7) = join(n+2, 1:7);
        join(n+2, 1:7) = dummy;
    end
end

force_z =1;  count = 0;  Z = zeros(loop(1), 2);
%This Z matrix stores the block number of each connection
for n = 1 : loop(1) %For each connected pair of blocks in the layer
    if(count ~= 0)
        count = count - 1;
    elseif(join(n, 1) == 1) %Connect with one convex part
        Z(force_z:(force_z + 3), 1:2) = [join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6)];
        force_z = force_z + 4;
    elseif(join(n, 1) == 4) %Connect with 4 convex parts 
        Z(force_z:(force_z + 3), 1:2) = [join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6)];
        force_z = force_z + 4;
        count = 3;
    elseif(join(n, 1) == 2) %Connect with two convex parts 
        Z(force_z:(force_z + 3), 1:2) = [join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6); join(n, 3:3:6)];
        force_z = force_z + 4;
        count = 1;
    end
end

force_z = force_z - 1;
Fnz = zeros(force_z, 13);  Fnz(1:force_z, 1) = 1:force_z; %Force_Number
Fnz(1:force_z, 7:6:13) = [ones(force_z, 1), -ones(force_z, 1)]; %Force direction for each block snaped
Fnz(1:force_z, 2:6:8) = Z(1:force_z, 1:2); %Block_Number
count = 1;  block_1 =0;  block_2 = 0;
remove = zeros(1, force_z);  count_r = 0;
for n = 1 : loop(1) %for each pair of connected knob
    type_1 = join(n, 2); %upper block type
    type_2 = join(n, 5); %lower block_type
    knob_1 = join(n, 4); %upper knob_index
    knob_2 = join(n, 7); %lower knob_index
 %Find the coordinates of the starting point of the force for each combination of connected blocks
    if( (join(n, 3) == block_1) && (join(n, 6) == block_2) )
        %Omitted if the same combination block 
    elseif(type_1 == 11) %if the upper block is 1x1
        Fnz(count:count+3, 4:6) = Pnz_11;
        if(type_2 == 11) %if the lower block is 1x1
            Fnz(count:count+3, 10:12) = Pnz11;  count = count + 4;
        elseif(type_2 == 12)
            if(knob_2 == 1)
                Fnz(count:count+3, 10:12) = Pnz12_1;  count = count + 4;
            elseif(knob_2 == 2)
                Fnz(count:count+3, 10:12) = Pnz12_2;  count = count + 4;
            end
        elseif(type_2 == 21)
            if(knob_2 == 1)
                Fnz(count:count+3, 10:12) = Pnz21_1;  count = count + 4;
            elseif(knob_2 == 2)
                Fnz(count:count+3, 10:12) = Pnz21_2;  count = count + 4;
            end
        else
            if(knob_2 == 1)
                Fnz(count:count+3, 10:12) = Pnz22_1;  count = count + 4;
            elseif(knob_2 == 2)
                Fnz(count:count+3, 10:12) = Pnz22_2;  count = count + 4;
            elseif(knob_2 == 3)
                Fnz(count:count+3, 10:12) = Pnz22_3;  count = count + 4;
            elseif(knob_2 == 4)
                Fnz(count:count+3, 10:12) = Pnz22_4;  count = count + 4;
            end
        end
    elseif(join(n, 1) == 4) %If there are 4 knobs snaped in the same pair of connected block
        Fnz(count:count+3, 4:6) = Pnz_22_0;
        Fnz(count:count+3, 10:12) = Pnz22_0;  count = count + 4;
    elseif(join(n, 1) == 2) %If there are 2 knobs snaped in the same pair of connected block
        if(type_1 == 12)
            Fnz(count:count+3, 4:6) = Pnz_12_0;
            if(type_2 == 22)
               if(knob_2 == 1)
                  Fnz(count:count+3, 10:12) = Pnz22_12;  count = count + 4;
               elseif(knob_2 == 3)
                   Fnz(count:count+3, 10:12) = Pnz22_34;  count = count + 4;
               end
            end
        elseif(type_1 == 21)
            Fnz(count:count+3, 4:6) = Pnz_21_0;
            if(type_2 == 22)
               if(knob_2 == 1)
                  Fnz(count:count+3, 10:12) = Pnz22_13;  count = count + 4;
               elseif(knob_2 == 2)
                   Fnz(count:count+3, 10:12) = Pnz22_24;  count = count + 4;
               end
            end
        elseif(type_1 == 22)
            if( (knob_1 == 1) && (join(n+1, 4) == 2)) 
                 Fnz(count:count+3, 4:6) = Pnz_22_12; %ok
            elseif( (knob_1 == 3) && (join(n+1, 4) == 4)) 
                 Fnz(count:count+3, 4:6) = Pnz_22_34; %ok
            elseif( (knob_1 == 1) && (join(n+1, 4) == 3)) 
                 Fnz(count:count+3, 4:6) = Pnz_22_13; %ok
            elseif( (knob_1 == 2) && (join(n+1, 4) == 4)) 
                 Fnz(count:count+3, 4:6) = Pnz_22_24; %ok
            end
            if(type_2 == 22)
                if( (knob_2 == 1) && (join(n+1, 7) == 2))
                    Fnz(count:count+3, 10:12) = Pnz22_12;  count = count + 4;
                elseif( (knob_2 == 3) && (join(n+1, 7) == 4))
                    Fnz(count:count+3, 10:12) = Pnz22_34;  count = count + 4;
                elseif( (knob_2 == 1) && (join(n+1, 7) == 3))
                    Fnz(count:count+3, 10:12) = Pnz22_13;  count = count + 4;
                elseif( (knob_2 == 2) && (join(n+1, 7) == 4))
                    Fnz(count:count+3, 10:12) = Pnz22_24;  count = count + 4;
                end
            end
        end
        if(type_2 == 12)
            Fnz(count:count+3, 10:12) = Pnz12_0;  count = count + 4;
        elseif(type_2 == 21)
            Fnz(count:count+3, 10:12) = Pnz21_0;  count = count + 4;
        end
    elseif(join(n, 1) == 1)
        if(type_1 == 12)
            if(knob_1 == 1)
                Fnz(count:count+3, 4:6) = Pnz_12_1;
            elseif(knob_1 == 2)
                Fnz(count:count+3, 4:6) = Pnz_12_2;
            end
        elseif(type_1 == 21)
            if(knob_1 == 1)
                Fnz(count:count+3, 4:6) = Pnz_21_1;
            elseif(knob_1 == 2)
                Fnz(count:count+3, 4:6) = Pnz_21_2;
            end
        elseif(type_1 == 22)
            if(knob_1 == 1)
                Fnz(count:count+3, 4:6) = Pnz_22_1;
                count_r = count_r + 1;  remove(count_r) = count + 3; %ok
            elseif(knob_1 == 2)
                Fnz(count:count+3, 4:6) = Pnz_22_2;
                count_r = count_r + 1;  remove(count_r) = count + 2; %ok
            elseif(knob_1 == 3)
                Fnz(count:count+3, 4:6) = Pnz_22_3;
                count_r = count_r + 1;  remove(count_r) = count + 1; %ok
            elseif(knob_1 == 4)
                Fnz(count:count+3, 4:6) = Pnz_22_4;
                count_r = count_r + 1;  remove(count_r) = count; %ok
            end
        end
        if(type_2 == 11)
            Fnz(count:count+3, 10:12) = Pnz11;  count = count + 4;
        elseif(type_2 == 12)
            if(knob_2 == 1)
                Fnz(count:count+3, 10:12) = Pnz12_1;  count = count + 4;
            elseif(knob_2 == 2)
                Fnz(count:count+3, 10:12) = Pnz12_2;  count = count + 4;
            end
        elseif(type_2 == 21)
            if(knob_2 == 1)
                Fnz(count:count+3, 10:12) = Pnz21_1;  count = count + 4;
            elseif(knob_2 == 2)
                Fnz(count:count+3, 10:12) = Pnz21_2;  count = count + 4;
            end
        elseif(type_2 == 22)
            if(knob_2 == 1)
                Fnz(count:count+3, 10:12) = Pnz22_1;  count = count + 4;
            elseif(knob_2 == 2)
                Fnz(count:count+3, 10:12) = Pnz22_2;  count = count + 4;
            elseif(knob_2 == 3)
                Fnz(count:count+3, 10:12) = Pnz22_3;  count = count + 4;
            elseif(knob_2 == 4)
                Fnz(count:count+3, 10:12) = Pnz22_4;  count = count + 4;
            end
        end
    end
    block_1 = join(n, 3);
    block_2 = join(n, 6);
end
count = 1;
if(count_r ~= 0) %Remove from Fnz some forces
    Fnz_remove = zeros(force_z-count_r, 13);
    for n = 1 : force_z
        check = 0;
        for m = 1 : count_r
           if(Fnz(n, 1) == remove(m)) %If this force is one to be removed
               check = check - 1;
               break;
           end
        end
        if(check == 0) %If this force is not one to be removed
            Fnz_remove(count, :) = [count, Fnz(n, 2:13)]; %Add Fnz to Fnz_remove
            count = count + 1;
        end
    end
else
    Fnz_remove = Fnz;
end
