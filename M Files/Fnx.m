function Fnx = Fnx(adjoin)
%Find the normal forces position in the x axis given the neighbors blocks found in the adjoin.m function in regard to its own coordinate axis.
%This function is working for up to 9x9 blocks
%This function stores in Fnx the position of this forces for both blocks.
%input: adjoin = (1,own block_number, own block_type, adjacent block_number, adjacent block_type, adjacent knob_index, adjoin type)
%output: Fnx = (Force_Number, Block_Number 1, Knob_Number 1, x1, y1, z1, -1, Block_Number 2, Knob_Number 2, x2, y2, z2, 1)

if(adjoin == -1)
    Fnx = -1;
else
    loop = size(adjoin,1);
    nx = loop; %Number of contact surfaces in the x direction 
    for n = loop : -1 : 1
        if(adjoin(n, 1) == 2) %Remove contact surfaces in the y direction from adjoin 
            adjoin(n, :) = [];
            nx = nx - 1;
        end
    end
    
    All_Forces = [];
    bc = zeros(9,9);
    for n = 1 : nx %for every adjoin block duo
        
        %First Block
        col = floor(adjoin(n, 3)/10);                               %block number of collums
        row = round(10*((adjoin(n, 3)/10)-floor(adjoin(n, 3)/10))); %block number of rows
        if(bc(col,row) == 0) %If the first block type is not in the All_Forces matrix
            bc(col,row) = 1; %Mark as visited
            Pnx = force_position(col,row,"fnx"); %Load the forces
            count = 1;
            for i = 1 : row %Iterate every set of 4 forces (every kind of normal force comes in sets of 4)
                for j = 1 : row 
                    if(j<i)
                        continue;
                    end
                    All_Forces = [All_Forces;Pnx(count:count+3,:),[10*col+row;10*i+j;0;0]]; %Add the force to the matrix with block type and adjoin type information
                    count = count + 4;
                end
            end
        end
        
        %Second Block
        col = floor(adjoin(n, 5)/10);                               %block number of collums
        row = round(10*((adjoin(n, 5)/10)-floor(adjoin(n, 5)/10))); %block number of rows
        if(bc(col,row) == 0) %If the first block type is not in the All_Forces matrix
            bc(col,row) = 1; %Mark as visited
            Pnx = force_position(col,row,"fnx"); %Load the forces
            count = 1;
            for i = 1 : row %Iterate every set of 4 forces (every kind of normal force comes in sets of 4)
                for j = 1 : row 
                    if(j<i)
                        continue;
                    end
                    All_Forces = [All_Forces;Pnx(count:count+3,:),[10*col+row;10*i+j;0;0]]; %Add the force to the matrix with block type and adjoin type information
                    count = count + 4;
                end
            end
        end
    end
   
    force_x = 4 * nx; %Number of Fnx forces
    if(force_x == 0) %if all contact surfaces in adjoin are in the y direction 
        Fnx = -1;
    else 
        Fnx = zeros(force_x, 13);  Fnx(1:force_x, 1) = 1:force_x; %create a matrix to store each force data
        Fnx(:, 7:6:13) = [-ones(force_x, 1), ones(force_x, 1)]; %set column 7 as -1 and column 13 as 1
        for n = 1 : nx  %For each adjoin block, determine where the normal forces in the X axis is located
            Fnx((4*n-3):4*n, 2:6:8) = [ones(4, 1) * adjoin(n, 2), ones(4, 1) * adjoin(n, 4)]; %set column 2 as the left block and column 8 as the right block
            type_1 = adjoin(n, 3); %left block type
            type_2 = adjoin(n, 5); %right block type
            adjoin_type_1 = adjoin(n, 7); %adjoin_type_1
            adjoin_type_2 = adjoin(n, 6);

            i = 1;
            while(i<size(All_Forces,1))
                if(type_1 == All_Forces(i,4) && adjoin_type_1 == All_Forces(i+1,4))
                    Fnx((4*n-3):4*n, 4:6) = All_Forces(i:i+3,1:3);
                    break;
                end
                i = i + 4;
            end
            
            i = 1;
            while(i<size(All_Forces,1))
                if(type_2 == All_Forces(i,4) && adjoin_type_2 == All_Forces(i+1,4))
                    Fnx((4*n-3):4*n, 10:12) = [-All_Forces(i:i+3,1), All_Forces(i:i+3,2:3)];
                    break;
                end
                i = i + 4;
            end
        end
    end
end