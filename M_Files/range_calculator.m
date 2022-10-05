function [range] = range_calculator(model)
    
    z_max = model(end,4); % Structure height
    model_size = size(model,1);
    range = zeros(z_max,2);   
    initial_index = 1;
    j = 1;
    for i = 1:z_max
        while(model(j,4) == i)
            if(j == model_size)
                j = j + 1;
                break;
            end
            j = j + 1;
        end
        final_index = j - 1;
        range(i,:) = [initial_index,final_index];
        initial_index = final_index + 1;
    end
end