function [block_coordinate,layer_map] = insert_block(layer_map,voxel_coordinates,knob_index,block_size)
%Try to insert the input block in the voxel map. The targeted voxel
%position will be the input block knob_index. I if the block using this
%knob as targeted voxel does not fit, the output will be empty
%
%input: layer_map: To check if the block fit the voxel model
%       voxel_coordinates: [x,y,z] target voxel Catersian coordinates
%       knob_index: inside this block which knob is currently in the target
%       block position.
%       block_size: [size_x,size_y]
%output: block_coordinate: [x,y] Block x and y Cartesian coordinates 
%       layer_map: update layer map if block is inserted.

    % Kinda complicated way to localize the knob index in the cartesian space.
    if(knob_index - block_size(2) <= 0)
        displacement_x_axis = 0;
    else
        displacement_x_axis = ceil(knob_index/block_size(2)) - 1;
    end
    remainder = rem(knob_index,block_size(2));
    if(remainder ~= 0)
        displacement_y_axis = remainder - 1;
    else
        displacement_y_axis = block_size(2) - 1;
    end
    
    map_size = size(layer_map);
    occupied_voxels = zeros(block_size(1)*block_size(2),2);
    knobs_tested_counter = 0;
    % Checks the input block knobs to see if them all fit in the voxel map
    for i = 0 : block_size(1) - 1 % Iterate the block knobs columns
        for j = 0 : block_size(2) - 1 % Iterate the block knobs rows
            x = voxel_coordinates(2) + i - displacement_x_axis;
            y = voxel_coordinates(1) - j + displacement_y_axis;
            if(y <= 0 || y > map_size(1) || x <= 0 || x > map_size(2)) %this block is out of bounds
                block_coordinate = [];
                return;
            elseif(layer_map(y,x) ~= 0) %if space not free return output 0
                block_coordinate = [];
                return;
            else %This voxel is free to insert a block
                knobs_tested_counter = knobs_tested_counter + 1;
                occupied_voxels(knobs_tested_counter,1:2) = [y,x];
            end
        end
    end
    %If this loop finished, the block fits
    for i = 1 : knobs_tested_counter
        layer_map(occupied_voxels(i,1),occupied_voxels(i,2)) = 1;
    end
    %Return the block coordinate using the correct mechanical analysis Y axis orientation
    block_coordinate = [voxel_coordinates(2) - displacement_x_axis, map_size(1) - (voxel_coordinates(1) + displacement_y_axis) + 1]; 
end

