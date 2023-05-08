function [plan,output] = NewPlanner(plan,filename,initial_condition)
% NewPlanner  Calculate the assembly plan for the input block model. 
%
% 
    
    %% Preparation
    range = Layer_Range_Calculator(plan);
    z_max = plan(end,4); % Highest Z position must be from the last block in model
    offset = 0; % Keep track of support blocks number to preserve the block iteration
    max_pil_h = 7; % Maximum height a support pillar can have
    max_layer_sup = 25; % Maximum number of support blocks in a single layer
    
    %% First Layer
    fprintf("\nBlocks %d to %d are from the 1st layer and therefore insertion is stable\n",initial_condition(2)+1,range(1,2)+initial_condition(2));
    global_layer = 1 + initial_condition(1);
    local = "../Block_Printer_System/Models/plan/Incomplete_Plans/incomplete_" + filename + "/";
    fprintf("Layer %d completed, temporary plan available in %s\n",global_layer, local + filename + "_layer_" + num2str(global_layer) + ".csv");
    plan_formatation(plan(1:range(1,2) + offset,:),local + filename + "_layer_" + num2str(global_layer) + ".txt"); %temporary
    
    %% Iterate the layers
    for layer = 2:z_max % From layer two to z_max.        
        %% Layer Rearrangement
        [plan,rearrange_strategy_output] = rearrange_strategy(plan,layer); % Rearrange Strategy
        if(rearrange_strategy_output < 0)
            fprintf("Error Occurred in Rearrange Strategy!");
            output = "rearrange";
            return;
        end
        
        %% Iterate the blocks
        layer_sup_n = 0; % Number of support blocks in this layer
        for block = range(layer,1):range(layer,2) % For every block in the current layer
            evaluating_model = plan(1:block + offset,:); % Delimit the model to be from the 1st to the "block"-th of the "layer"-th layer
            [planner_output] = Planner_Stability_Judge(evaluating_model); % Evaluate the current block insertion stability
            sup_strat_output = 0;
            if(~strcmp(planner_output,'safe')) % If insertion is not safe
                %% Support block strategy
<<<<<<< Updated upstream
                [plan,sup_strat_output] = support_block_strategy(plan,block + offset); % Run the Support Block Strategy
                layer_sup_n = layer_sup_n + sup_strat_output; % Update the layer number of support blocks
                if(layer_sup_n >= max_layer_sup || sup_strat_output >= max_pil_h) % If the subassembly flag is detected
                    %% Subassembly strategy
                    [plan,subassembly_strategy_output] = subassembly_strategy(plan,block + offset + sup_strat_output,filename,initial_condition);
                    if(subassembly_strategy_output < 0) % If error in subassembly strategy
                        fprintf("Error Occurred in Subassembly Strategy!");
                        output = "subassembly";
                        return;
                    else
                        output = "ok";
                        return;
                    end
                elseif(sup_strat_output < 0) % If any problem were detected in the support block strategy
=======
                [plan,sup_strat_output] = Support_Block_Strategy(plan,block + offset); % Run the Support Block Strategy
                if(sup_strat_output < 0) % If any problem were detected in the support block strategy
>>>>>>> Stashed changes
                    fprintf("Error Occurred in Support Block Strategy!");
                    output = "support";
                    return;
                end
                layer_sup_n = layer_sup_n + sup_strat_output; % Update the layer number of support blocks
                if(layer_sup_n >= max_layer_sup || sup_strat_output >= max_pil_h) % If the subassembly flag is detected
                    %% Subassembly strategy
                    [plan,subassembly_strategy_output] = Subassembly_Strategy(plan,block + offset + sup_strat_output,filename,initial_condition);
                    if(subassembly_strategy_output < 0) % If error in subassembly strategy
                        fprintf("Error Occurred in Subassembly Block Strategy!");
                        output = "subassembly";
                        return;
                    else % Because this function is called from within Subassembly_Strategy, when it returns here, all the block model has been iterated.
                        output = "ok";
                        return;
                    end
                end
                offset = offset + sup_strat_output; % Correct the next block to be analyzed
            end
            global_block = block + initial_condition(2);
            fprintf("\n Block %d/%d insertion is stable\n",global_block,range(end,2)+initial_condition(2));
            if(sup_strat_output ~= 0)
                fprintf("Was necessary %d support blocks\n",sup_strat_output);
            end
        end
        
        global_layer = layer + initial_condition(1);
        local = "../Block_Printer_System/Models/plan/Incomplete_Plans/incomplete_" + filename + "/";
        fprintf("Layer %d completed, temporary plan available in %s\n",global_layer, local + filename + "_layer_" + num2str(global_layer) + ".csv");
        plan_formatation(plan(1:block + offset,:),local + filename + "_layer_" + num2str(global_layer) + ".txt"); %temporary
    end
    output = "ok";
end