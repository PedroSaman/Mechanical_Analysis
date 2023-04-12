function [submodel1,submodel2] = separate_model(model,division_layer) 
%
%input: model:
%       division_layer: last layer of the first subassembly
    
    for i = 1:size(model,1)
        if(model(i,4) > division_layer)
            break;
        end
    end
    
    submodel1 = model(1:i-1,:);
    submodel2 = model(i:end,:);
    %Clean subassembly1 (take out the non necessary support blocks)
    
    i = max(submodel1(:,1));
    
    while(submodel1(i,4) == division_layer)
        if(submodel1(i,6) == 99)
            X = submodel1(i,2);
            Y = submodel1(i,3);
            removed = 0;
            j = submodel1(i,1);
            while(j >= 1)
                if(submodel1(j,2) == X && submodel1(j,3) == Y && submodel1(j,6) == 99)
                    submodel1(j,:) = [];
                    removed = removed + 1;
                end
                j = j - 1;
            end
            i = i - removed;
            submodel1(:,1) = 1:size(submodel1);
        else
            i = i - 1;
        end
    end
    
    %Prepare subassembly2 to be iterated in New_Planner function (change Z values)
    submodel2(:,4) = submodel2(:,4) - division_layer;
    submodel2(:,1) = 1:size(submodel2);
    
end