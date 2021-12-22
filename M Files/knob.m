function y = knob(model, layer)
%Convert block model data to knob data
%input: model:(block_number, x, y, z, Block_type, color)
%       layer:(the layer that is desired to search for knobs. If 0, consider all layers) 
%output: y:(x,y,z,block_type,color,block_number,knob_index)

loop = size(model);
k = 0;  %Number of knobs

%Count how many knobs it is needed to be iterated
if(layer == 0) %if it is searching for all layers knobs
    for n = 1 : loop(1) %uses the block type nomenclature to calculate k
        a = floor(model(n,5)/10);                      %number of collums
        b = 10*((model(n,5)/10)-floor(model(n,5)/10)); %number of rows
        k = k + a*b;
    end
    k = round(k);
    k_min = 1;
    k_max = loop(1);
else %if it is searching for only one layer knobs
    k_min = 0; %first block to be in the layer this function is searching
    for n = 1 : loop(1) %for each block in the model
        if(model(n, 4) == layer) %if the block z value is equal to the layer argument
            if(k_min == 0) 
                k_min = n; %Store the first block in the model that is in the layer
            end
            if(model(n, 5) == 24)     %if the block is 2x4
                k = k + 8;
            elseif(model(n, 5) == 22) %if the block is 2x2
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
end

%Create a matrix with each knob information
y = zeros(k, 7); %y(x,y,z,block_type,color,block_number,knob_index)
m = 1;  %Count the number of stored convex parts
for n = k_min : k_max %for each block
    if(model(n,5) == 24) %if the block is 2x4
        for i = 0:7 %for each of the 8 knobs
            y(m+i, 1:5) = model (n, 2:6); %Copy the model data to y
            y(m+i, 6) = model(n, 1); %Block number inside the block model
            y(m+i, 7) = i + 1;  %Knob index inside its block
        end
    y(m+1, 2) = y(m, 2) + 1; y(m+5, 2) = y(m, 2) + 1; %Add 1 to Y index for the necessary knobs
    y(m+2, 2) = y(m, 2) + 2; y(m+6, 2) = y(m, 2) + 2; %Add 2 to Y index for the necessary knobs
    y(m+3, 2) = y(m, 2) + 3; y(m+7, 2) = y(m, 2) + 3; %Add 3 to Y index for the necessary knobs
    y(m+4, 1) = y(m, 1) + 1; y(m+5, 1) = y(m, 1) + 1; %Add 1 to X index for the necessary knobs
    y(m+6, 1) = y(m, 1) + 1; y(m+7, 1) = y(m, 1) + 1; %Add 1 to X index for the necessary knobs
    m = m + 8; %When it is a 2x4 block, add 8 to the knob counter
    elseif(model(n, 5) == 22) %if block is 2x2
        for i = 0:3 %for each of the 4 knobs
            y(m+i, 1:5) = model (n, 2:6); %Copy the model data to y
            y(m+i, 6) = model(n, 1); %Block number inside the block model
            y(m+i, 7) = i + 1;  %Knob index inside its block
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
        for i = 0:1 %for each of the 2 knobs
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