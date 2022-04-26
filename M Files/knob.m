function y = knob(model, layer)
%This function is ready for every block of 1 to 9 rows and 1 to 9 columns
%Convert block model data to knob data
%input: model:(block_number, x, y, z, Block_type, color)
%       layer:(the layer that is desired to search for knobs. If 0, consider all layers) 
%output: y:(x,y,z,block_type,color,block_number,knob_index)

loop = size(model);
k = 0;  %Number of knobs

%Count how many knobs it is needed to be iterated
if(layer == 0) %if it is searching for all layers knobs
    for n = 1 : loop(1) %uses the block type nomenclature to calculate k
        [col,row] = col_row_converter(model(n, 5)); %block number of columns and rows
        k = k + col*row;
    end
    k_min = 1;
    k_max = loop(1);
else %if it is searching for only one layer knobs
    k_min = 0; %first block to be in the layer this function is searching
    for n = 1 : loop(1) %for each block in the model
        if(model(n, 4) == layer) %if the block z value is equal to the layer argument
            if(k_min == 0) 
                k_min = n; %Store the first block in the model that is in the layer
            end
            [col,row] = col_row_converter(model(n, 5)); %block number of columns and rows
            k = k + col*row;
            k_max = n; %Store the last block in the model that is in the layer so far.
        elseif(model(n, 4) > layer) %if the block model z value is less than the layer argument
            break
        end
    end
end

%Create a matrix with each knob information
k = round(k);
y = zeros(k, 7); %y(x,y,z,block_type,color,block_number,knob_index)
m = 1;  %Count the number of stored convex parts
for n = k_min : k_max %for each block
    [col,row] = col_row_converter(model(n, 5)); %block number of columns and rows
    for i = 0:col*row-1 %for each knob in the block
        y(m+i, 1:5) = model (n, 2:6); %Copy the model data to y
        y(m+i, 6) = model(n, 1); %Block number inside the block model
        y(m+i, 7) = i + 1; %Knob index inside its block
    end
    
    %Correction of X and Y values
    k = 0;
    for i = 0:col-1 %for every column
        for j = 0:row-1 %for every row
            y(m+k, 1) = y(m+k, 1) + i; %Correct X value   
            y(m+k, 2) = y(m+k, 2) + j; %Correct Y value
            k = k + 1; %Same block knobs iterator
        end
    end
    m = m + k; %Update the knob counter 
end