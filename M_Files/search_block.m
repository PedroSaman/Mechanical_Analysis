function [knob_map,block_number] = search_block(knobs,model,block_number)
    %Searches to knobs with free space below to insert support blocks.
    %input: knobs: Knob information from all model layers (PRECISO VERIFICAR NO LAB)
    %       model: Entire block model (PRECISO VERIFICAR NO LAB)
    %       block_number: being inserted block number
    %output: knob_map: represent the state of a knob. 1 -> not free, 0 - > free, -1 -> does not exist.
    %                  each row is a block and each column is a knob in that block.
    %       block_number: stores each knob_map row block number.

    layer = model(block_number,4); % Being inserted block layer.
    join_i = join(knobs, layer); % Connected knobs information of connections between block_number layer and the one below. 
    
    [col,row] = col_row_converter(model(block_number,5));
    
    contact_points = col*row;
    knob_map = -ones(1,16); % Create the being inserted knob_map vector assuming no knob exist. (16 is the maximum size of a block)
    for i = 1:col*row
        knob_map(i) = 0; % For each existing knob assumes that it is free.
    end
    
    below_blocks = zeros(size(model,1),1); % Possible maximum number of blocks below.
    
    % Count how many contact points 'block_number' has.
    for i = 1:size(join_i,1) % For every connection.
        if(join_i(i,3) == block_number) % If this connection is between 'block_number' block and any other.
            contact_points = contact_points - 1; % How many possible free spots the block has.
            knob_map(join_i(i,4)) = 1; % Change the map not free.
            below_blocks(join_i(i,6)) = join_i(i,6); % Store the connecting block to the below_blocks vector.
        end
    end

    % Erase empty spaces in below_blocks vector.
    i = size(below_blocks,1);
    while(i > 0)
        if(below_blocks(i) == 0)
            below_blocks(i) = [];
        end
        i = i - 1;
    end
    
    if(isempty(below_blocks)) % If this is a floating block return
        return;
    end
    
    knob_map_below = -ones(size(below_blocks,1),16); % Create the knob_map for every 'below block'.
    
    for i = 1 : size(below_blocks,1) % For each 'block below'.
        layer = model(below_blocks(i),4); % This 'below block' layer
        [col,row] = col_row_converter(model(below_blocks(i),5));
        if(layer == 1) % Blocks in the first layer don't have free space below.
            for j = 1:col*row
                knob_map_below(i,j) = 1;
            end
        else
            join_i = join(knobs, layer); % This 'below block' Connected knobs information
            for j = 1:col*row
                knob_map_below(i,j) = 0; % Assume every 'below block' knob is free
            end
            for j = 1:size(join_i,1) % Count how many contact points this 'below block' has
                if(join_i(j,3) == below_blocks(i)) % If this connection is with this 'below block'
                    knob_map_below(i,join_i(j,4)) = 1; % Mark the knob as not free.
                end
            end        
        end
    end
    
    % Merge the vectors the being inserted block and all 'below block's vectors to use in output.
    knob_map = [knob_map;knob_map_below];
    block_number = [block_number;below_blocks];
    
    % Discard blocks that do not have space below.
    i = size(block_number,1);
    while(i >= 1)
        ok = 1;
        for j = 1 : size(knob_map,2)
            if(knob_map(i,j) == 0) % If there is any free space, do not exclude
                ok = 0;
                break;
            end
            if(knob_map(i,j) == -1) % If this block has ended
                break;
            end
        end
        if(ok == 1) % If no free space was found, erase this block from the output.
            knob_map(i,:) = [];
            block_number(i) = [];
        end
        i = i - 1;
    end
end