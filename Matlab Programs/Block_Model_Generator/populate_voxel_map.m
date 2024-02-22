function [block_to_insert, layer_map] = populate_voxel_map(layer_map,voxel_index)
%Iterate the layer_map list, use the first not populated voxel
%on edge found, if none, use the first non populated voxel in the list.
%Then, try to insert a block that agiven the priority_block_list, that
%occupy the chosen voxel.
    
    priority_block_list = mod(voxel_index(3) - 1,2); % Odd and even layers have different block list priority
    if(priority_block_list) % Set the priority list for block placement
        %block_list = [[8,2];[2,8];[4,2];[2,4];[3,2];[2,3];[4,1];[1,4];[2,2];[3,1];[1,3];[2,1];[1,2];[1,1]];
        block_list = [[2,2];[2,1];[1,2];[1,1]];
    else
       %block_list = [[2,8];[8,2];[2,4];[4,2];[2,3];[3,2];[1,4];[4,1];[2,2];[1,3];[3,1];[1,2];[2,1];[1,1]]; 
       block_list = [[2,2];[1,2];[2,1];[1,1]]; 
    end

    block_list_size = size(block_list,1);
    
    for block = 1 : block_list_size % Iterate the block priority list
        block_total_size = block_list(block,1)*block_list(block,2); % Current block size in number of knobs
        for knob_index = 1 : block_total_size % Iterate the knob number
            [block_coordinate,layer_map] = insert_block(layer_map,voxel_index,knob_index,block_list(block,:)); % Try to insert current block with current knob at the voxel_index position
            if(~isempty(block_coordinate)) % If does fit, block_coordinate will not be empty
                block_to_insert = [block_coordinate,0,block_list(block,1),block_list(block,2),1,0]; % This should be the biggest block that fit in this voxel
                return;
            end
        end
    end
    block_to_insert = -1; %should never return from here, at least the 1x1 should fit
end