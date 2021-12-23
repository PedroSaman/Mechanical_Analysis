function A = adjoin_2(line)
%Find the pairs of adjacent knobs in this layer from the block model. Because it finds the pair of knob, in the Y axis it only checks to the top 
%and in the X axis only checks to the right.
%input: line: every knob in a given layer. (x,y,z,block_type,color,block_number,knob_index)
%output: A: for each pair of adjacent knobs there are one line in A with 7 info: 
%           (X axis(1) or Y axis(2), first block_number, first block_type, second block_number, second block_type, second knob_index, adjoin_type)
%           if there is no adjacent block, this function returns -1
%           adjoin_type is acording with the Sugimoto definition.

knobs = size(line);
X = zeros(knobs(1) * 2, 7);  Y = zeros(knobs(1) * 2, 7);
count_x = 1;  count_y = 1;  same_block = 0;

for n = 1 : knobs(1) %for every knob in this layer
    
    if(same_block ~= 0) %If the knob is from the same block, continue. This will be computed until it reaches the next block.
        same_block = same_block - 1; %Generic subtractor
    else %If this is the first knob in a block

        col = floor(line(n, 4)/10);                             %block number of collums
        row = round(10*((line(n, 4)/10)-floor(line(n, 4)/10))); %block number of rows
        
        x = -ones(row,5); y = -ones(col,5); %Initialize the x and y matrices
        check = row*col;
        same_block = check-1;

        %Determine where are the possible adjoin position for this block
        for i=0:row-1
            x(i+1,1) = line(n,1) + col; x(i+1,2) = line(n,2) + i;
        end
        for i=0:col-1
            y(i+1,1) = line(n,1) + i;   y(i+1,2) = line(n,2) + row;
        end

        %Search if there are knobs in the possible adjoin knob positions
        for m = 1 : knobs(1) %For every knob in this layer
            if(check == 0) %If all possible adjacent knob were checked, break.
                break;
            else
                for i=1:row
                    if((line(m, 1) == x(i, 1)) && (line(m, 2) == x(i, 2))) %This knob (m) is one of the possible adjoint knob to the right?
                        x(i, 3) = line(m, 6);  x(i, 4) = line(m, 4);  x(i, 5) = line(m, 7); check = check -1; %add information to x (block_number,block_type,knob_index)
                    end
                end
                for i=1:col
                    if((line(m, 1) == y(i, 1)) && (line(m, 2) == y(i, 2))) %This knob (m) is one of the possible adjoint knob to the top?
                        y(i, 3) = line(m, 6);  y(i, 4) = line(m, 4);  y(i, 5) = line(m, 7); check = check -1; %add information to y (block_number,block_type,knob_index)
                    end
                end
            end
        end

        %After finding which of the possible adjacent knobs in fact exist
        %...
    end
    A = 1;
end