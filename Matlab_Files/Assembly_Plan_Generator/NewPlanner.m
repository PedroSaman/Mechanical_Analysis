function [plan,output] = NewPlanner(plan,filename,initial_condition)
% NewPlanner  Calculate the assembly plan for the input block model. 
%
% 
    %% Preparation
    range = Layer_Range_Calculator(plan);
    z_max = plan(end,4); % Highest Z position must be from the last block in model
    offset = 0; % Keep track of support blocks number to preserve the block iteration
    %max_pil_h = 24; % Maximum height a support pillar can have (Not implemented)
    max_layer_sup = 0.5; % Relative (%) number of blocks compared with the current layer number of blocks to start a sub-assembly
    incomplete_plan_location = "../../Block_Printer_System/Models/plan/Incomplete_Plans/incomplete_" + filename + "/";
    
    %% First Layer
    %fprintf("\nBlocks %d to %d are from the 1st layer and therefore insertion is stable\n",initial_condition(2)+1,range(1,2)+initial_condition(2));
    global_layer = 1 + initial_condition(1);
    local = incomplete_plan_location;
    fprintf("Layer %d completed, temporary plan available in %s\n",global_layer, local + filename + "_layer_" + num2str(global_layer) + ".csv");
    plan_formatation(plan(1:range(1,2) + offset,:),local + filename + "_layer_" + num2str(global_layer) + ".txt",0); %temporary
    
    %% Iterate the layers
    for layer = 2:z_max % From layer two to z_max.        
        %% Layer Rearrangement
        [plan,rearrange_strategy_output] = Rearrange_Strategy(plan,layer); % Rearrange Strategy
        if(rearrange_strategy_output < 0)
            fprintf("Error Occurred in Rearrange Strategy!");
            output = "rearrange";
            return;
        end
        
        %% Iterate the blocks
        layer_sup_n = 0; % Number of support blocks in this layer
        layer_max_supp_blk_numb = ceil(range(layer,3)*max_layer_sup); %max_layer_sup% times the volume of blocks in the layer rounded up
        for block = range(layer,1):range(layer,2) % For every block in the current layer
            sup_strat_output = 0;
            %global_block = block + initial_condition(2);
            evaluating_model = plan(1:block + offset,:); % Delimit the model to be from the 1st to the "block"-th of the "layer"-th layer
            [planner_output] = Planner_Stability_Judge(evaluating_model); % Evaluate the current block insertion stability
            if(strcmp(planner_output,'100safe'))
                plan(block + offset,7) = 1;
            elseif(strcmp(planner_output,'safe'))
                plan(block + offset,7) = 0;
            else % If insertion is not safe
                %% Support block strategy
                [plan,sup_strat_output] = Support_Block_Strategy(plan,block + offset); % Run the Support Block Strategy
                layer_sup_n = layer_sup_n + sup_strat_output; % Update the layer number of support blocks
                if(layer_sup_n > layer_max_supp_blk_numb || sup_strat_output < 0) % If the subassembly flag is detected
                    %% Subassembly strategy
                    %fprintf("Block %d insertion flagged a subassembly strategy. Required %d support blocks which is bigger than the maximum possible defined for this layer %d.",global_block,layer_sup_n,layer_max_supp_blk_numb);
                    [plan,subassembly_strategy_output] = Subassembly_Strategy(plan,block + offset + sup_strat_output,filename,initial_condition);
                    if(subassembly_strategy_output < 0) % If error in subassembly strategy
                        fprintf("Error Occurred in Subassembly Strategy!");
                        output = "subassembly";
                        return;
                    else % Because this function is called from within Subassembly_Strategy, when it returns here, all the block model has already been iterated.
                        output = "ok";
                        return;
                    end
                end
                offset = offset + sup_strat_output; % Correct the next block to be analyzed
            end
            fprintf("\n Block %d/%d insertion is stable\n",global_block,range(end,2)+initial_condition(2));
            if(sup_strat_output ~= 0)
                fprintf("Was necessary %d support blocks\n",sup_strat_output);
            end
        end
        %If not sub-assembly for this layer was necessary, change the 
        %support block index to 99.
        for i = 1:(block+offset)
            if(plan(i,6) == 98)
                plan(i,6) = 99;
            end
        end
        global_layer = layer + initial_condition(1);
        local = incomplete_plan_location;
        fprintf("Layer %d completed, temporary plan available in %s\n",global_layer, local + filename + "_layer_" + num2str(global_layer) + ".csv");
        plan_formatation(plan(1:block + offset,:),local + filename + "_layer_" + num2str(global_layer) + ".txt",0);
    end
    output = "ok";
end