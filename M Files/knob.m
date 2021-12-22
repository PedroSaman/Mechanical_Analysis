function y = knob(model)
%Convert block model data to convex data
%input: model:(block_number, x, y, z, Block_type, color)
%output: y:(x,y,z,block_type,color,block_number,knob_index)

loop = size(model);
k = 0;  %Number of knobs
for n = 1 : loop(1)
    a = floor(model(n,5)/10);                    %number of collums
    b = 10*((model(n,5)/10)-floor(model(n,5)/10)); %number of rows
    k = k + a*b;
end
k = round(k);

%For each knob create 7 information data
y = zeros(k, 7); %y(x,y,z,block_type,color,block_number,knob_index)
m = 1;  %Count the number of stored convex parts
for n = 1 : loop(1) %for each block
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