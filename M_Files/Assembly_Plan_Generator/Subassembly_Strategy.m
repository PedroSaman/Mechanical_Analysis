function [plan,subassembly_strategy_output] = Subassembly_Strategy(model,block_number,filename,initial_condition)
% Implement the subassembly strategy. This function decides where to break
% the input model into two pieces, calls every function needed to adapt the
% subassemblies, and calls NewPlaner for the second part. This can be seen
% as recursive function. If the second subassembly need to be divided again,
% everything will be merged together in the end.
%
% input: model:(block_number ,x, y, z, block_type, color).
%       block number: is the block being inserted.
%       filename: this is needed because NewPlaner will be called again.
% output: plan: (block_number ,x, y, z, block_type, color). Updated plan. 
%        subassembly_strategy_output: how many support blocks was added. -1 is error 

    %% Preparation
    layer = model(block_number,4); % Being Inserted block layer
    break_layer = layer - 1; % Where to divide the two subassemblies
    [submodel1,submodel2] = Separate_Subassemblies(model,break_layer); % Create both subassemblies data structure
    
    fprintf("\nFrom block %d a Subassembly is necessary.\n",block_number);
    fprintf("\nStarted the Subassembly assembly planning. It has %d blocks\n\n",size(submodel2,1));
    
    %% Count support blocks in subassembly one
    i = 1;
    block = 0;
    while(model(i,4) <= submodel1(end,4))
        if(model(i,6) ~= 99)
            block = block + 1;
        end
        i = i + 1;
    end
    
    %% Call the NewPlaner for the subassembly two
    initial_condition = initial_condition + [break_layer,block];
    [submodel2,output] = NewPlanner(submodel2,filename,initial_condition);
    
    %% Evaluate the output
    if(~strcmp(output,'ok'))
        plan = model;
        subassembly_strategy_output = -1;
    else
        plan = merge_subassemblies(submodel1,submodel2);
        subassembly_strategy_output = 1;
    end
end