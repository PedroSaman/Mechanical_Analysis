function plan = myFun((model,block_number))
%myFun - Description
%
% Syntax: output = myFun(input)
%
% Long description
    
    layer = model(block_number,4); % Being Inserted block layer
    join_i = join(knobs, layer); % Connected knobs information
    
    floating = 1;
    for i = 1:size(join_i,1) % Verify if the block is floating
        if(join_i(i,3) == block_number)
            below_blocks(join_i(i,6)) = join_i(i,6); % Which blocks has contact with 'block_number'
            floating = 0;
            break;
        end
    end

    % If the problematic block is floating, the first subassembly is: 1 -> layer
    % but if not, the first subassembly is: 1 -> layer - 1
    if(floating)
        [submodel1,submodel2] = separate_model(model,layer); 
    else
        [submodel1,submodel2] = separate_model(model,layer-1);
    end

    submodel1 = clean_subassembly(submodel1);
    submodel2 = NewPlanner(submodel2); %NEED TO CORRECT THE NewPlanner function!!!!!!!!!!!!!!!

    plan = merge_subassemblies(submodel1,submodel2);
end