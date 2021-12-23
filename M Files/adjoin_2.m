function A = adjoin(line)
%Find the pairs of adjacent blocks in this layer from the block model. Because it finds the pair of block, in the Y axis it only checks to the top 
%and in the X axis only checks to the right.
%input: line: each knob in a given layer. (x,y,z,block_type,color,block_number,knob_index)
%output: A: for each pair of adjacent blocks there are one line in A with 7 info: 
%           (X axis(1) or Y axis(2), first block_number, first block_type, second block_number, second block_type, second knob_index, adjoin_type)
%           if there is no adjacent block, this function returns -1
%           adjoin_type is acording with the Sugimoto definition.

blocks = size(line);
X = zeros(blocks(1) * 2, 7);  Y = zeros(blocks(1) * 2, 7);
count_x = 1;  count_y = 1;  same_block = 0;

for n = 1 : blocks(1) %for every knob in this layer
    %x: adjoint knob information in the X axis (x,y, knob block_number, knob block_type, knob_index in the block model)
    %y: adjoint knob information in the Y axis (x,y, knob block_number, knob block_type, knob_index in the block model)
    if(same_block ~= 0) %If the knob is from the same block, continue. This will be computed until it reaches the next block.
        same_block = same_block - 1; %Generic subtractor
    else
    col = floor(line(n, 4)/10);                      %number of collums
    row = 10*((line(n, 4)/10)-floor(line(n, 4)/10)); %number of rows

end