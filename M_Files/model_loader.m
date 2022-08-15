function [model,range] = model_loader(filename)
    
    model_original = load(filename);
    k = size(model_original,1);
    model(:,1) = 1:k;
    model(:,2:4) = model_original(:,1:3) + 1;
    model(:,5) = model_original(:,4)*10 + model_original(:,5);
    model(:,6) = model_original(:,7);
    
    z_max = model(end,4); % Structure height
    model_size = size(model,1);
    range = zeros(z_max,2);   
    initial_index = 1;
    j = 1;
    for i = 1:z_max
        while(model(j,4) == i)
            j = j + 1;
            if(j == model_size)
                j = j + 1;
                break;
            end
        end
        final_index = j - 1;
        range(i,:) = [initial_index,final_index];
        initial_index = final_index + 1;
    end
    
end