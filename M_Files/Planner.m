function [plan] = Planner(filename)
    
    [model,range] = model_loader(filename);
    plan = 0;
    r = 200; %number of times to try to make stable (provisory)
    
    z_max = model(end,4);
    for layer = 2:z_max
        for j = range(layer,1):range(layer,2)
            evaluating_model = model(1:j,:);
            [Output] = Planner_Stability_Judge(evaluating_model);
            iterator = 1;
            while(Output ~= "safe" && iterator <= r)
                iterator = iterator + 1;
                evaluating_model = shuffle(model,range,layer);
                [Output] = Planner_Stability_Judge(evaluating_model);
                if(Output == "safe")
                    plan = evaluating_model;
                end
            end
        end
    end
end