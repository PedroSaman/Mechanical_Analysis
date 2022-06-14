function y = knob(model, layer)
%Convert block model data to knob data
%input: model:(block_number, x, y, z, Block_type, color)
%       layer:(the layer that is desired to search for knobs. If 0, consider all layers) 
%output: y:(x,y,z,block_type,color,block_number,knob_index)

loop = size(model,1); % Number of blocks
k = 0; % Number of knobs

%Count how many knobs there are in the desired layer
if(layer == 0) % If it is searching for all layers knobs
    for n = 1 : loop
        [col,row] = col_row_converter(model(n, 5)); % Block number of columns and rows
        k = k + col*row;
    end
    k_min = 1;
    k_max = loop;
else % If it is searching only for one layer knobs
    k_min = 0; % First block of the desired layer
    for n = 1 : loop
        if(model(n, 4) == layer) % If the block z value is equal to the desired layer
            if(k_min == 0) 
                k_min = n; % Store the model first block that appear in the desired layer
            end
            [col,row] = col_row_converter(model(n, 5)); % Block number of columns and rows
            k = k + col*row;
            k_max = n; % Store the model last block that appear in the desired layer so far
        elseif(model(n, 4) > layer) % After computing every block in the desired layer
            break
        end
    end
end

%Create a matrix with each knob information
k = round(k);
y = zeros(k, 7); % y(x,y,z,block_type,color,block_number,knob_index)
m = 1;  % Count the number of stored convex parts
for n = k_min : k_max % For each block
    [col,row] = col_row_converter(model(n, 5)); % Block number of columns and rows
    for i = 0:col*row-1 % For each knob in the block
        y(m+i, 1:5) = model (n, 2:6); % Copy the model data to y
        y(m+i, 6) = model(n, 1); % Block number inside the block model
        y(m+i, 7) = i + 1; % Knob index inside its block
    end
    
    %X and Y values correction
    k = 0;
    for i = 0:col-1 % For every column
        for j = 0:row-1 % For every row
            y(m+k, 1) = y(m+k, 1) + i; %Correct X value   
            y(m+k, 2) = y(m+k, 2) + j; %Correct Y value
            k = k + 1; % Same block knobs iterator
        end
    end
    m = m + k; % Update the knob count
end