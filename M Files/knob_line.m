function y = knob_line(model, layer)
%Convert block model data to convex data 
% input: model:(Block_Number, x, y, z, type, color)
%        layer:(the layer that is desired to search for knobs) 
% output: y(x,y,z,block_type,color,block_number,knob_index)

loop = size(model); %Number of blocks
k = 0;  %Number of knobs in this layer
k_min = 0; %first block to be in the layer this function is searching

%This loop iterate the dat model until the z value is higher than the layer this function is searching
%Search the first and last block to be in the layer this function is searching and its number of knobs
for n = 1 : loop(1) %for each block in the model
    if(model(n, 4) == layer) %if the block z value is equal to the layer argument
        if(k_min == 0) 
            k_min = n; %Store the first block in the model that is in the layer
        end
        if(model(n, 5) == 22) %if the block is 2x2
            k = k + 4;
        elseif(model(n, 5) == 12) %if the block is 1x2
            k = k + 2;
        elseif(model(n, 5) == 21) %if the block is 2x1
            k = k + 2;
        elseif(model(n, 5) == 11) %if the block is 1x1
            k = k + 1;
        end
        k_max = n; %Store the last block in the model that is in the layer so far.
    elseif(model(n, 4) > layer) %if the block model z value is less than the layer argument
        break
    end
end

%For each knob in this layer, create 7 information data
y = zeros(k, 7); %y(x,y,z,block_type,color,block_number,knob_index)
m = 1;  %Count the number of knobs
for n = k_min : k_max; %only itarate the blocks in the model that is in this layer.
    if(model(n, 5) == 22) %if block is 2x2
        for i = 0:3 %for each of the 4 knobs
            y(m+i, 1:5) = model (n, 2:6); %Copy the model data to y
            y(m+i, 6) = model(n, 1); %Block number inside the block model
            y(m+i, 7) = i + 1; %Knob index inside its block
        end
        y(m+1, 2) = y(m, 2) + 1; y(m+3, 2) = y(m, 2) + 1; %Add 1 to Y index for the necessary knobs
        y(m+2, 1) = y(m, 1) + 1; y(m+3, 1) = y(m, 1) + 1; %Add 1 to X index for the necessary knobs
        m = m + 4; %When it is a 2x2 block, add 4 to the knob counter
    elseif(model(n, 5) == 12) %if block is 1x2
        for i = 0:1 %for each of the 2 knobs
            y(m+i, 1:5) = model (n, 2:6); %Copy the model data to y
            y(m+i, 6) = model(n, 1); %Block number inside the block model
            y(m+i, 7) = i + 1; %Knob index inside its block
        end
        y(m+1, 2) = y(m, 2) + 1; %Add 1 to Y index for the necessary knobs
        m = m + 2; %When it is a 1x2 block, add 2 to the knob counter
    elseif(model(n, 5) == 21) %if block is 2x1
        for i = 0:1  %for each of the 2 knobs
            y(m+i, 1:5) = model (n, 2:6); %Copy the model data to y
            y(m+i, 6) = model(n, 1); %Block number inside the block model
            y(m+i, 7) = i + 1; %Knob index inside its block
        end
        y(m+1, 1) = y(m, 1) + 1; %Add 1 to X index for the necessary knobs
        m = m + 2; %When it is a 2x1 block, add 2 to the knob counter
    elseif(model(n, 5) == 11) %if block is 1x1
        y(m, 1:5) = model (n, 2:6); %Copy the model data to y
        y(m, 6) = model(n, 1); %Block number inside the block model
        y(m, 7) = 1; %Knob index inside its block
        m = m + 1; %When it is a 1x1 block, add 1 to the knob counter
    end
end