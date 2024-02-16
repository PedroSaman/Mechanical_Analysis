function range = voxel_model_range_calculator(voxels_vector)
% Given the voxels model in the input, determines the first and last blocks
% of each layer.
% input: voxels_vector:(x, y, z)
% output: range:[first_block_layer_n,last_block_layer_n]

    last_layer = voxels_vector(end,3); % Last layer
    model_size = size(voxels_vector,1);
    range = zeros(last_layer,2);
    initial_index = 1;
    j = 1;
    for i = 1:last_layer % For every layer
        while(voxels_vector(j,3) == i) % While same layer
            if(j == model_size)
                j = j + 1;
                break;
            end
            j = j + 1;
        end
        final_index = j - 1; % Correct count
        range(i,:) = [initial_index,final_index]; 
        initial_index = final_index + 1; % The next layer first block will be the next block in model.
    end
end