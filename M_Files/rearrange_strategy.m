function [plan,rearrange_strategy_output] = rearrange_strategy(model,layer)
    % Run the Rearranging the inserting order, sometimes is enought to stabilize the block insertion
    
    knobs = knob(model,0); % Knob information (entire block model)
    join_i = join(knobs, layer); % Connected knobs information
    plan = model;
    mx_blk_n = max(join_i(:,3)); %Maximum block number in the layer
    mn_blk_n = min(join_i(:,3)); %Minimum block number in the layer

    %Preallocation    
    block_max_size = mx_blk_n - mn_blk_n + 1; %Maximum block array size
    s_block = zeros(block_max_size,1);        %Blocks that is fully on top of another block
    s_block_i = 0;                            %Iterator
    h_block = zeros(block_max_size,1);        %Blocks that has any hanging part
    h_block_i = 0;                            %Iterator
    
    for i = mn_blk_n:mx_blk_n %For every block in layer
        [col,row] = col_row_converter(model(i,5));
        knob_number = col*row;
        for j = 1:size(join_i,1) %Search in join how many knobs are connected
            if(join_i(j,3) == i)
                knob_number = knob_number - 1;
            end
            if(knob_number == 0)
                s_block_i = s_block_i + 1;
                s_block(s_block_i) = i;
                break;
            end
        end
        if(knob_number ~= 0)
            h_block_i = h_block_i + 1;
            h_block(h_block_i) = i;
        end
    end
    
    s_block(s_block_i+1:end) = [];
    h_block(h_block_i+1:end) = [];
    
    j = mn_blk_n;
    k = 0;
    for i = 1:block_max_size
        if(i<=s_block_i)
            plan(j,2:end) = model(s_block(i),2:end);
            j = j + 1;
            k = k + 1;
        else
            plan(j,2:end) = model(h_block(i-k),2:end);
            j = j + 1;
        end
    end
    
    check = model_check(model(1:mx_blk_n,:)); % Test the integrity of the rearranged plan\
    rearrange_strategy_output = 1;
    if(check < 0) % If any problem is encountered, return error
        rearrange_strategy_output = -1;
    end
end