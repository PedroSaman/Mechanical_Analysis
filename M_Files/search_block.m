function [knob_map,block_number] = search_block(knobs,model,block_number)
    %Retorna uma lista em que cada linha representa um bloco e cada coluna
    %um knob, 0 significa q esta livre e pode add uma coluna de blocos de
    %suporte, 1 significa que nao esta livre e -1 significa que nao existe
    %aquele knob

    layer = model(block_number,4); % Being Inserted block layer
    join_i = join(knobs, layer); % Connected knobs information
    
    [col,row] = col_row_converter(model(block_number,5));
    
    contact_points = col*row;
    knob_map = -ones(1,16);
    
    for i = 1:col*row
        knob_map(i) = 0;
    end
    
    below_blocks = zeros(size(model,1),1);
    
    for i = 1:size(join_i,1) % Count how many contact points 'block_number' has
        if(join_i(i,3) == block_number)
            contact_points = contact_points - 1; % How many free spots the block has
            knob_map(join_i(i,4)) = 1; % 0 Indicates the knob is free (below 'block_number')
            below_blocks(join_i(i,6)) = join_i(i,6); % Which blocks has contact with 'block_number'
        end
    end
    i = size(below_blocks,1);
    while(i > 0)
        if(below_blocks(i) == 0)
            below_blocks(i) = [];
        end
        i = i - 1;
    end
    
    if(isempty(below_blocks)) %Floating block
        return;
    end
    
    knob_map_below = -ones(size(below_blocks,1),16);
    
    for i = 1 : size(below_blocks,1)
        layer = model(below_blocks(i),4); % Being Inserted block layer
        join_i = join(knobs, layer); % Connected knobs information
        [col,row] = col_row_converter(model(below_blocks(i),5));
        for j = 1:col*row
            knob_map_below(i,j) = 0;
        end
        for j = 1:size(join_i,1) % Count how many contact points 'block_number' has
            if(join_i(j,3) == below_blocks(i))
                knob_map_below(i,join_i(j,4)) = 1; % 0 Indicates the knob is free
            end
        end        
    end
    
    knob_map = [knob_map;knob_map_below];
    block_number = [block_number;below_blocks];
    
    i = size(block_number,1);
    while(i >= 1)
        ok = 1;
        for j = 1 : size(knob_map,2)
            if(knob_map(i,j) == 0)
                ok = 0;
                break;
            end
            if(knob_map(i,j) == -1)
                break;
            end
        end
        if(ok == 1)
            knob_map(i,:) = [];
            block_number(i) = [];
        end
        i = i - 1;
    end
end