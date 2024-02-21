function [plan,removed_blocks_number] = Remove_Redundancies(plan,block,pillar_numb)
% Remove unnecessary support pillars from the plan. The nature of this
% strategy can sometimes insert more pillars than necessary. This function
% searches for those unnecessary and erases them. Also, this function 
% changes the support blocks "color" value to 98 to represent that it is a
% support block. After exiting the support block strategy and if a
% subassembly strategy is not necessary, this value changes to 99.
%
% input: plan: Current assembly plan.
%       block: block being inserted number uptaded (plus inserted support)
%       pillar_numb: number of inserted pillars.
% output: plan: Assemby plan updated.
%        removed_blocks_number: how many support blocks were removed in this function verification.

    removed_blocks_number = 0;
    if(pillar_numb ~= 1) % If only one pillar was inserted, it is not necessary to verify the stability without it.
        i = block; 
        while(i >= 1)
            if(plan(i,6) == -1) % If this is an unverified support block
                %Block origin cartesian coordinates
                X = plan(i,2);
                Y = plan(i,3);
                Z = plan(i,4);

                evaluating_model = plan; % Plan that will be tested (with the support pillar removed)
                %As the support strategy insert a pillar in the (X,Y) position from the Z position to the base layer where it is free
                %it is necessary to search in this (X,Y) pillar where there are support blocks and remove them.
                j = i; 
                removed_blocks = 0;
                while(j >= 1 && Z >= 1)
                    if(plan(j,6) == -1 && plan(j,2) == X && plan(j,3) == Y) % If this is a unverified support block and has the same (X,Y) than the curent block.
                        evaluating_model(j,:) = []; % Remove this block from the evaluating model.
                        removed_blocks = removed_blocks + 1;
                        Z  = Z - 1; % Step down the Z value.
                        plan(j,6) = 98; % Mark this block as visited in plan.
                    end
                    j = j - 1;
                end
                
                evaluate_range = block - removed_blocks; % Evaluating model block range
                evaluating_model = [(1:size(evaluating_model,1))' , evaluating_model(1:end,2:end)]; % Correct block numbering after removing the blocks
                merger = evaluating_model(evaluate_range+1:end,:); % Save the rest of the plan.
                evaluating_model = evaluating_model(1:evaluate_range,:); % Do not consider blocks after the one being evaluated
                evaluating_model = [(1:size(evaluating_model,1))' , evaluating_model(1:end,2:end)]; %Correct block numbering
                [planner_output] = Planner_Stability_Judge(evaluating_model); % Evaluate evaluating_model insertion stability
                if(strcmp(planner_output,'safe')||strcmp(planner_output,'100safe')) % If it safe => the removed support pillar is redundant
                    plan = [evaluating_model;merger]; % Save the plan as the one without the redundant support pillar
                    block = block - removed_blocks; % Update the number of the being inserted block.
                    i = i - removed_blocks + 1; % Update the iterator i
                    removed_blocks_number = removed_blocks_number + removed_blocks;
                end
            end
            i = i - 1;
        end
    else
        % Change the support blocks from the support pillar from -1 to 99 in case this is the only support pillar.
        for i = 1 : block
            if(plan(i,6) == -1)
                plan(i,6) = 98;
            end
        end
    end
end