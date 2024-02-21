function Block_Model_Generator(file_name)
%Creates the initial block model from the voxel representation generated in
%the web application. Input is directly the file from drububu and the output
%is ready to be used in the mechanical analysis. All blocks color are white
%as default. Can be painted in Kohama UNIX software.
%The output was formated to be exactly equal to kohama one in which x,y and
%minimum value is 0. To change the possible blocks to use change the
%block_list vector in the populate_voxel_map.m file.
%output: there is no output from this function, but the program creates a
%.txt file in the Models/Block Models/ folder.
%     each row has a block info: [x, y, z, size_x, size_y, filler, color]

    voxel_file_location = '../../Models/Voxel Models/' + file_name + '.json'; % Input file path
    save_file_location = '../../Models/Block Models/'; % Output file path
    fid = fopen(voxel_file_location); 
    raw = fread(fid,inf); 
    str = char(raw'); 
    fclose(fid); 
    val = jsondecode(str); % Import the json file data

    voxels_number = size(val.voxels,1); % Number of voxels in the file_name.json
    voxels_vector = zeros(voxels_number,3); % Preallocate voxels_vector
    Block_Model = zeros(voxels_number,7); % Preallocate Block_Model as if all voxels turned into 1x1 blocks (biggest case possible)
    collumn_to_sort = 3; % sortrows function column to sort

    for i = 1 : voxels_number % Correct the x, y, z to work with my logic. (This can be improved)
        % For some reason the output from the voxilizer program gives Z
        % values as the second position not the third. That is why Y is the
        % 3rd value and Z is the 2nd.
        voxels_vector(i,1) = str2double(val.voxels(i).x) + 1; % Width argument
        voxels_vector(i,3) = str2double(val.voxels(i).y) + 1; % Depth argument
        voxels_vector(i,2) = str2double(val.voxels(i).z) + 1; % Height argument
    end

    voxels_vector = sortrows(voxels_vector,collumn_to_sort); % Sort voxels from lowest Z to highest
    last_layer = voxels_vector(end,3); % Last layer    
    range = voxel_model_range_calculator(voxels_vector); % Each layer range (first and last voxels)
    
    block_index = 1; 
    for i = 1 : last_layer % For every layer converts voxel representation to initial block model
        Layer_Blocks = create_layer(voxels_vector(range(i,1):range(i,2),1:3)); % Function that returns the block list that represents the current layer
        if(Layer_Blocks == -1)
            fprintf("\nERROR. populate_voxel_map did not find a block that fit one voxel.\n");
            return;
        end
        added_blocks = size(Layer_Blocks,1);
        Block_Model(block_index:block_index + added_blocks - 1,:) = Layer_Blocks;
        block_index = block_index + added_blocks;
    end
    Block_Model(block_index:end,:) = []; % Erase unused space
    Block_Model(:,1:3) = Block_Model(:,1:3) - 1; % Correct Cartesian representation to be the same as Kohama structure
    Block_Model(:,7) = 0;
    dlmwrite(save_file_location + file_name + '_P.txt', Block_Model, ' '); % Create the block model using the endind "_P" to identify that this block model was created using Pedro algorithm
end