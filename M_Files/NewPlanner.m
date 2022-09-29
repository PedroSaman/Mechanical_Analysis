function [plan] = NewPlanner(filename)
    % NewPlanner  Calculate the assembly plan for the input block model. 
    % [plan] = NewPlanner(filename)
    %
    % Input: filename: text file name with its directory. Must be in Kohama
    % Voxel Converter output format.
    %
    % Output: plan: csv file in the necessary format to be used in the
    % next 3D block printer step (NOT IMPLEMENTED)
    %
    % This planner utilizes the Mechanical Analysis to judge the structure
    % stability for each block insertion.

    tic
    [plan,range] = model_loader(filename); % Load model and adapt it to the Stability Judge format
    z_max = plan(end,4); % Highest Z position must be from the last block in model
    offset = 0; % Keep track of support blocks number to preserve the block iteration
    
    fprintf("Blocks 1 to %d are from the 1st layer, so their insertion is stable\n",range(1,2));
    
    for layer = 2:z_max % From layer two to z_max. (First layer is always stable)
        
        [plan,rearrange_strategy_output] = rearrange_strategy(plan,layer); % Run the Rearrange Strategy
        if(rearrange_strategy_output < 0)
            fprintf("Error Occurred in Rearrange Strategy!");
            return;
        end

        for block = range(layer,1):range(layer,2) % For every block in the current layer
            evaluating_model = plan(1:block + offset,:); % Delimit the model to be from the 1st to the "block"-th of the "layer"-th layer
            [planner_output] = Planner_Stability_Judge(evaluating_model); % Evaluate the insertion Stability
            support_strategy_output = 0;
            if(~strcmp(planner_output,'safe')) % If not safe
                [plan,support_strategy_output] = support_block_strategy(plan,block + offset); % Run the Support Block Strategy
                if(support_strategy_output < 0) % If not possible, the output will be -1 indicating error ocurred
                    fprintf("Error Occurred in Support Block Strategy!");
                    return;
                end
                offset = offset + support_strategy_output; % Correct the next block to be analyzed
            end
            fprintf("\n Block %d/%d insertion is stable\n",block,range(end,2));
            if(support_strategy_output ~= 0)
                fprintf("Was necessary %d support blocks\n",support_strategy_output);
            end
        end
    end
    toc
    plan_formatation(plan,filename);
end