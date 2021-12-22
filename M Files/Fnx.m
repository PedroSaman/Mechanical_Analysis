function Fnx = Fnx(adjoin)
%Find the normal forces position in the x axis given the neighbors blocks found in the adjoin.m function in regard to its own coordinate axis.
%This function stores in Fnx the position of this forces for both blocks.
%input: adjoin = (1,own block_number, own block_type, adjacent block_number, adjacent block_type, adjacent knob_index, adjoin type)
%output: Fnx = (Force_Number, Block_Number 1, Knob_Number 1, x1, y1, z1, -1, Block_Number 2, Knob_Number 2, x2, y2, z2, 1)

if(adjoin == -1)
    Fnx = -1;
else
    % 1x1 Block
    pn1 = [2, -2, -1.5];  pn2 = [2,  2, -1.5];  pn3 = [2, -2,  1.5];  pn4 = [2,  2,  1.5];
    Pnx11 = [pn1; pn2; pn3; pn4];  Pnx_11 = [-Pnx11(1:4, 1), Pnx11(1:4, 2:3)];

    % 1x2 Block
    pn1 = [2, -4, -1.5];  pn2 = [2,  4, -1.5];  pn3 = [2, -4,  1.5];  pn4 = [2,  4,  1.5];
    Pnx12_0 = [pn1; pn2; pn3; pn4];  Pnx_12_0 = [-Pnx12_0(1:4, 1), Pnx12_0(1:4, 2:3)];
    pn1 = [2, -4, -1.5];  pn2 = [2,  0, -1.5];  pn3 = [2, -4,  1.5];  pn4 = [2,  0,  1.5];
    Pnx12_1 = [pn1; pn2; pn3; pn4];  Pnx_12_1 = [-Pnx12_1(1:4, 1), Pnx12_1(1:4, 2:3)];
    pn1 = [2, 0, -1.5];  pn2 = [2, 4, -1.5];  pn3 = [2, 0,  1.5];  pn4 = [2, 4,  1.5];
    Pnx12_2 = [pn1; pn2; pn3; pn4];  Pnx_12_2 = [-Pnx12_2(1:4, 1), Pnx12_2(1:4, 2:3)];

    % 2x1 Block
    pn1 = [-4, -2, -1.5];  pn2 = [-4,  2, -1.5];  pn3 = [-4, -2,  1.5];  pn4 = [-4,  2,  1.5];
    Pnx21_1 = [pn1; pn2; pn3; pn4];  Pnx21_2 = [-Pnx21_1(1:4, 1), Pnx21_1(1:4, 2:3)];

    % 2x2 Block
    pn1 = [4, -4, -1.5];  pn2 = [4,  4, -1.5];  pn3 = [4, -4,  1.5];  pn4 = [4,  4,  1.5];
    Pnx22_0 = [pn1; pn2; pn3; pn4];  Pnx_22_0 = [-Pnx22_0(1:4, 1), Pnx22_0(1:4, 2:3)];
    pn1 = [-4, -4, -1.5];  pn2 = [-4,  0, -1.5];  pn3 = [-4, -4,  1.5];  pn4 = [-4,  0,  1.5];
    Pnx22_1 = [pn1; pn2; pn3; pn4];  Pnx22_3 = [-Pnx22_1(1:4, 1), Pnx22_1(1:4, 2:3)];
    pn1 = [-4, 0, -1.5];  pn2 = [-4, 4, -1.5];  pn3 = [-4, 0,  1.5];  pn4 = [-4, 4,  1.5];
    Pnx22_2 = [pn1; pn2; pn3; pn4];  Pnx22_4 = [-Pnx22_2(1:4, 1), Pnx22_2(1:4, 2:3)];

    loop = size(adjoin);
    nx = loop(1); %Number of contact surfaces in the x direction 
    for n = loop(1) : -1 : 1
        if(adjoin(n, 1) == 2) %Remove contact surfaces in the y direction from adjoin 
            adjoin(n, :) = [];
            nx = nx - 1;
        end
    end

    force_x = 4 * nx; %Number of Fnx forces
    if(force_x == 0) %if all contact surfaces in adjoin are in the y direction 
        Fnx = -1;
    else 
        Fnx = zeros(force_x, 13);  Fnx(1:force_x, 1) = 1:force_x; %create a matrix to store each force data
        Fnx(:, 7:6:13) = [-ones(force_x, 1), ones(force_x, 1)]; %set column 7 as -1 and column 13 as 1
        for n = 1 : nx  %For each adjoin block
            Fnx((4*n-3):4*n, 2:6:8) = [ones(4, 1) * adjoin(n, 2), ones(4, 1) * adjoin(n, 4)]; %set column 2 as the left block and column 8 as the right block
        end
        for n = 1 : nx %for each adjoin block, this for determine where the normal forces in the X axis is located
            type_1 = adjoin(n, 3); %left block type
            type_2 = adjoin(n, 5); %right block type
            adjoin_type = adjoin(n, 7); %adjoin_type
            
            %In this cases does not matter the adjoin type. (there is only one)
            if(type_1 == 11) %if the left block is a 1x1
                Fnx((4*n-3):4*n, 4:6) = Pnx11;
            elseif(type_1 == 21) %if the bottom block is a 2x1
                Fnx((4*n-3):4*n, 4:6) = Pnx21_2;
            end
            if(type_2 == 11) %if the right block is a 1x1
                Fnx((4*n-3):4*n, 10:12) = Pnx_11; 
            elseif(type_2 == 21) %if the right block is a 2x1
                Fnx((4*n-3):4*n, 10:12) = Pnx21_1; 
            end
            
            %Here the adjoin type matters
            if(adjoin_type == 3) %Adjacent with two convex parts
                if(type_1 == 12) %if the left block is a 1x2
                    Fnx((4*n-3):4*n, 4:6) = Pnx12_0;
                elseif(type_1 == 22) %if the left block is a 2x2
                    Fnx((4*n-3):4*n, 4:6) = Pnx22_0;
                end
                if(type_2 == 12) %if the right block is a 1x2
                    Fnx((4*n-3):4*n, 10:12) = Pnx_12_0;
                elseif(type_2 == 22) %if the right block is a 2x2
                    Fnx((4*n-3):4*n, 10:12) = Pnx_22_0;
                end
            else %Adjacent with one convex part 
                if(adjoin_type == 5) %Adjacent to the convex part on the lower side of block 1 
                    if(type_1 == 12) %if the left block is a 1x2
                        Fnx((4*n-3):4*n, 4:6) = Pnx12_1;
                    elseif(type_1 == 22) %if the left block is a 2x2
                        Fnx((4*n-3):4*n, 4:6) = Pnx22_3;
                    end
                elseif(adjoin_type == 6) %Adjacent to the convex part on the upper side of block 1
                    if(type_1 == 12) %if the left block is a 1x2
                        Fnx((4*n-3):4*n, 4:6) = Pnx12_2;
                    elseif(type_1 == 22) %if the left block is a 2x2
                        Fnx((4*n-3):4*n, 4:6) = Pnx22_4;
                    end
                end
                knob = adjoin(n, 6);
                if(type_2 == 12) %if the left block is a 1x2
                    if(knob == 1) %if the knob 1 is the one that is the adjacent
                        Fnx((4*n-3):4*n, 10:12) = Pnx_12_1;
                    elseif(knob == 2) %if the knob 2 is the one that is the adjacent
                        Fnx((4*n-3):4*n, 10:12) = Pnx_12_2;
                    end
                elseif(type_2 == 22) %if the left block is a 2x2
                    if(knob == 1) %if the knob 1 is the one that is the adjacent
                        Fnx((4*n-3):4*n, 10:12) = Pnx22_1;
                    elseif(knob == 2) %if the knob 2 is the one that is the adjacent
                        Fnx((4*n-3):4*n, 10:12) = Pnx22_2;
                    end
                end
            end
        end
    end
end