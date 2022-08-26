function [plan] = remove_redundancies(plan,block)
    
    i = block; 
    while(i >= 1)
        if(plan(i,6) == -1)
            X = plan(i,2);
            Y = plan(i,3);
            Z = plan(i,4);
            
            evaluating_model = plan;
            j = i;
            removed_blocks = 0;
            while(j >= 1 && Z >= 1)
                if(plan(j,6) == -1 && plan(j,2) == X && plan(j,3) == Y)
                    evaluating_model(j,:) = [];
                    removed_blocks = removed_blocks + 1;
                    Z  = Z - 1;
                    plan(j,6) = 99;
                end
                j = j - 1;
            end
            evaluate_range = block - removed_blocks;
            evaluating_model = evaluating_model(1:evaluate_range,:);
            evaluating_model = [(1:size(evaluating_model,1))' , evaluating_model(1:end,2:end)];
            [planner_output] = Planner_Stability_Judge(evaluating_model);
            if(strcmp(planner_output,'safe'))
                plan = evaluating_model;
                block = block - removed_blocks;
                i = i - removed_blocks + 1;
            end
        end
        i = i - 1;
    end
end