function [plan,subassembly_strategy_output] = subassembly_strategy(model,block_number,filename,initial_condition)
%myFun - Description
%
% Syntax: output = myFun(input)
%
% Long description
    
    fprintf("\nFrom block %d a Subassembly is necessary.\n",block_number);

    layer = model(block_number,4); % Being Inserted block layer
    knobs = knob(model,0); % Knob information (entire block model).
    join_i = join(knobs, layer); % Connected knobs information
    subassembly_strategy_output = 1;
    
    
    floating = 1;
    for i = 1:size(join_i,1) % Verify if the block is floating
        if(join_i(i,3) == block_number)
            floating = 0;
            break;
        end
    end

    % If the problematic block is floating, the first subassembly is: 1 ->
    % layer -1 but if not, the first subassembly is: 1 -> layer - 2
    if(floating)
        layer = layer - 1;
    else
        layer = layer - 2;
    end
    
    [submodel1,submodel2] = separate_model(model,layer);
    
    fprintf("\nStarted the Subassembly assembly planning. It has %d blocks\n\n",size(submodel2,1));
    i = 1;
    block = 0;
    while(model(i,4) <= submodel1(end,4))
        if(model(i,6) ~= 99)
            block = block + 1;
        end
        i = i + 1;
    end
    initial_condition = initial_condition + [layer,block];
    [submodel2,~] = NewPlanner(submodel2,filename,initial_condition);
    
    plan = merge_subassemblies(submodel1,submodel2);
end