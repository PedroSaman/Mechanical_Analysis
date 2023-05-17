function block_list = create_layer(voxel_model_layer)
%Itarates the input voxel layer vector checking which voxel has not being
%occupied yet. 
%input: voxel_model_layer: [x, y, z]
%output block_list: [x, y, z, size_x, size_y, filler(1), color(0)]
    
    voxel_vector_size = size(voxel_model_layer,1);
    block_list = zeros(voxel_vector_size,7); % Preallocate the block list for this layer (X,Y,Z,block_type)
    voxel_model_layer = define_edge_voxels(voxel_model_layer); % Resort the voxel_model_layer vector to prioritise the voxels in the edges
    
    [layer_map,voxel_model_layer] = create_layer_map(voxel_model_layer); % Creates a visual representation map of the current layer
    block_counter = 0;
    z = voxel_model_layer(1,3); % Z value never changes for a single layer
    for i = 1 : voxel_vector_size % For all voxels in the input
        x = voxel_model_layer(i,1);
        y = voxel_model_layer(i,2);
        if(layer_map(y,x) ~= 0) % If this voxel is not marked as not visited, continue the loop
            continue;
        end
        block_counter = block_counter + 1; % Updates the block number
        [block_list(block_counter,:), layer_map] = populate_voxel_map(layer_map,[y,x,z]); % Search for the biggest block that can be fit in this voxel
        if(block_list(block_counter,:) == -1)
            %ERROR. populate_voxel_map did not find a block that fit one voxel
            block_list = -1;
            return;
        end
    end
    block_list(block_counter + 1:end,:) = []; % Remove unused space
    block_list(:,3) = z; % Correct blocks Z value
end