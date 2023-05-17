function [evaluating_model] = shuffle(model,range,layer)
    %Not used anymore. Keeping just in case.
    %Gets the block model and the current layer to shuffle the block order.

    total_rng = range(layer,2)-range(layer,1) + 1;
    new_order = randperm(total_rng) + range(layer,1) - 1;
    evaluating_model = model(1:range(layer-1,2),:);
    evaluating_model = [evaluating_model;model(new_order(:),:)];
    
    evaluating_model(:,1) = 1:size(evaluating_model,1);
    
end