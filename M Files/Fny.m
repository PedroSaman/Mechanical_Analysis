function Fny = Fny(adjoin)
%Find the normal forces position in the y axis given the neighbors blocks found in the adjoin.m function in regard to its own coordinate axis.
%This function stores in Fny the position of this forces for both blocks.
%input: adjoin = (2,own block_number, own block_type, adjacent block_number, adjacent block_type, adjacent knob_index, adjoin type)
%output: Fny = (Force_Number, Block_Number 1, Knob_Number 1, x1, y1, z1, -1, Block_Number 2, Knob_Number 2, x2, y2, z2, 1)

if(adjoin == -1)
    Fny = -1;
else
    % 1x1 Block
    pn1 = [-2, 2, -1.5];  pn2 = [-2, 2,  1.5];  pn3 = [ 2, 2, -1.5];  pn4 = [ 2, 2,  1.5];
    Pny11 = [pn1; pn2; pn3; pn4];  Pny_11 = [Pny11(1:4, 1), -Pny11(1:4, 2), Pny11(1:4, 3)];

    % 1x2 Block
    pn1 = [-2, -4, -1.5];  pn2 = [-2, -4,  1.5];  pn3 = [ 2, -4, -1.5];  pn4 = [ 2, -4,  1.5];
    Pny12_1 = [pn1; pn2; pn3; pn4];  Pny12_2 = [Pny12_1(1:4, 1), -Pny12_1(1:4, 2), Pny12_1(1:4, 3)];

    % 2x1 Block
    pn1 = [-4, 2, -1.5];  pn2 = [-4, 2,  1.5];  pn3 = [ 4, 2, -1.5];  pn4 = [ 4, 2,  1.5];
    Pny21_0 = [pn1; pn2; pn3; pn4];  Pny_21_0 = [Pny21_0(1:4, 1), -Pny21_0(1:4, 2), Pny21_0(1:4, 3)];
    pn1 = [-4, 2, -1.5];  pn2 = [-4, 2,  1.5];  pn3 = [ 0, 2, -1.5];  pn4 = [ 0, 2,  1.5];
    Pny21_1 = [pn1; pn2; pn3; pn4];  Pny_21_1 = [Pny21_1(1:4, 1), -Pny21_1(1:4, 2), Pny21_1(1:4, 3)];
    pn1 = [0, 2, -1.5];  pn2 = [0, 2,  1.5];  pn3 = [4, 2, -1.5];  pn4 = [4, 2,  1.5];
    Pny21_2 = [pn1; pn2; pn3; pn4];  Pny_21_2 = [Pny21_2(1:4, 1), -Pny21_2(1:4, 2), Pny21_2(1:4, 3)];

    % 2x2 Block
    pn1 = [-4, 4, -1.5];  pn2 = [-4, 4,  1.5];  pn3 = [ 4, 4, -1.5];  pn4 = [ 4, 4,  1.5];
    Pny22_0 = [pn1; pn2; pn3; pn4];  Pny_22_0 = [Pny22_0(1:4, 1), -Pny22_0(1:4, 2), Pny22_0(1:4, 3)];
    pn1 = [-4, -4, -1.5];  pn2 = [-4, -4,  1.5];  pn3 = [ 0, -4, -1.5];  pn4 = [ 0, -4,  1.5];
    Pny22_1 = [pn1; pn2; pn3; pn4];  Pny22_2 = [Pny22_1(1:4, 1), -Pny22_1(1:4, 2), Pny22_1(1:4, 3)];
    pn1 = [0, -4, -1.5];  pn2 = [0, -4,  1.5];  pn3 = [4, -4, -1.5];  pn4 = [4, -4,  1.5];
    Pny22_3 = [pn1; pn2; pn3; pn4];  Pny22_4 = [Pny22_3(1:4, 1), -Pny22_3(1:4, 2), Pny22_3(1:4, 3)];

    loop = size(adjoin);
    ny = loop(1); %Number of contact surfaces in the y direction 
    for n = loop(1) : -1 : 1
        if(adjoin(n, 1) == 1) %Remove contact surfaces in the x direction from adjoin 
            adjoin(n, :) = [];
            ny = ny - 1;
        end
    end

    force_y = 4 * ny; %Number of Fny forces
    if(force_y == 0) %if all contact surfaces in adjoin are in the x direction 
        Fny = -1; 
    else
        Fny = zeros(force_y, 13);  Fny(1:force_y, 1) = 1:force_y; %create a matrix to store each force data
        Fny(:, 7:6:13) = [-ones(force_y, 1), ones(force_y, 1)];  %set column 7 as -1 and column 13 as 1
        for n = 1 : ny %For each adjoin block
            Fny((4*n-3):4*n, 2:6:8) = [ones(4, 1) * adjoin(n, 2), ones(4, 1) * adjoin(n, 4)]; %set column 2 as the lower block and column 8 as the upper block
        end
        for n = 1 : ny %for each adjoin block, this for determine where the normal forces in the Y axis is located
            type_1 = adjoin(n, 3); %lower block type
            type_2 = adjoin(n, 5); %upper block type
            adjoin_type = adjoin(n, 7); %adjoin_type

            %In this cases does not matter the adjoin type. (there is only one)
            if(type_1 == 11) %if the bottom block is a 1x1
                Fny((4*n-3):4*n, 4:6) = Pny11;
            elseif(type_1 == 12) %if the bottom block is a 1x2
                Fny((4*n-3):4*n, 4:6) = Pny12_2;
            end
            if(type_2 == 11) %if the upper block is a 1x1
                Fny((4*n-3):4*n, 10:12) = Pny_11;
            elseif(type_2 == 12) %if the upper block is a 1x2
                Fny((4*n-3):4*n, 10:12) = Pny12_1;
            end
            
            %Here the adjoin type matters
            if(adjoin_type == 3) %Adjacent with two convex parts
                if(type_1 == 21) %if the bottom block is a 2x1
                    Fny((4*n-3):4*n, 4:6) = Pny21_0;
                elseif(type_1 == 22) %if the bottom block is a 2x2
                    Fny((4*n-3):4*n, 4:6) = Pny22_0;
                end
                if(type_2 == 21) %if the upper block is a 2x1
                    Fny((4*n-3):4*n, 10:12) = Pny_21_0;
                elseif(type_2 == 22) %if the upper block is a 2x2
                    Fny((4*n-3):4*n, 10:12) = Pny_22_0;
                end
            else %Adjacent with one convex part 
                if(adjoin_type == 5) %Adjacent to the convex part on the left side of block 1 
                    if(type_1 == 21) %if the bottom block is a 2x1
                        Fny((4*n-3):4*n, 4:6) = Pny21_1;
                    elseif(type_1 == 22) %if the bottom block is a 2x2
                        Fny((4*n-3):4*n, 4:6) = Pny22_2;
                    end
                elseif(adjoin_type == 6) % Adjacent to the convex on the right side of block 1 
                    if(type_1 == 21) %if the bottom block is a 2x1
                        Fny((4*n-3):4*n, 4:6) = Pny21_2;
                    elseif(type_1 == 22) %if the bottom block is a 2x2
                        Fny((4*n-3):4*n, 4:6) = Pny22_4;
                    end
                end
                knob = adjoin(n, 6);
                if(type_2 == 21) %if the upper block is a 2x1
                    if(knob == 1) %if the knob 1 is the one that is the adjacen
                        Fny((4*n-3):4*n, 10:12) = Pny_21_1;
                    elseif(knob == 2) %if the knob 2 is the one that is the adjacen
                        Fny((4*n-3):4*n, 10:12) = Pny_21_2;
                    end
                elseif(type_2 == 22) %if the upper block is a 2x2
                    if(knob == 1) %if the knob 1 is the one that is the adjacen
                        Fny((4*n-3):4*n, 10:12) = Pny22_1;
                    elseif(knob == 3) %if the knob 3 is the one that is the adjacen
                        Fny((4*n-3):4*n, 10:12) = Pny22_3;
                    end
                end
            end
        end
    end
end